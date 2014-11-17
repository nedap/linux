/*
 * max3107_console.c - spi uart protocol driver for Maxim 3107
 * Based on max3110.c, with console support
 *
 * Copyright (c) 2008-2010, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Note:
 * 1. From Max3107 spec, the Rx FIFO has 8 words, while the Tx FIFO only has
 *    1 word. If SPI master controller doesn't support sclk frequency change,
 *    then the char need be sent out one by one with some delay
 *
 * 2. Currently only RX available interrupt is used, no need for waiting TXE
 *    interrupt for a low speed UART device
 */

#ifdef CONFIG_MAGIC_SYSRQ
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>

#include <linux/kthread.h>
#include <asm/atomic.h>
#include <linux/spi/spi.h>

#include "max3107_console.h"

#define PR_FMT	"max3107_console: "
#define WORDS_PER_XFER  128
#define MAX_READ_LEN    128

struct uart_max3107 {
	struct uart_port port;
	struct spi_device *spi;
	char *name;

	wait_queue_head_t wq;
	struct task_struct *main_thread;
	struct task_struct *read_thread;
	int mthread_up;
	spinlock_t lock;

	u32 baud;
	u16 cur_conf;
	u8 clock;
	u8 parity, word_7bits;

	atomic_t uart_tx_need;

	/* console related */
	struct circ_buf con_xmit;
	atomic_t con_tx_need;

	/* irq related */
	u16 irq;
	atomic_t irq_pending;
};

struct uart_max3107 *pmax;
static void receive_chars(struct uart_max3107 *max,
				unsigned char *str, int len);
static int max3107_read_multi(struct uart_max3107 *max, int len, u8 *buf);
static void max3107_con_receive(struct uart_max3107 *max);

int max3107_write_then_read(struct uart_max3107 *max,
		const u8 *txbuf, u8 *rxbuf, unsigned len, int always_fast)
{
	struct spi_device *spi = max->spi;
	struct spi_message	message;
	struct spi_transfer	x;
	u8 etx[2], erx[2];
	int ret;

	// NOTE: len must always be 2

	if (!txbuf || !rxbuf)
		return -EINVAL;

	etx[0] = txbuf[1];
	etx[1] = txbuf[0];

	spi_message_init(&message);
	memset(&x, 0, sizeof x);
	x.len = 2;
	x.tx_buf = etx; //txbuf;
	x.rx_buf = erx; //rxbuf;
	spi_message_add_tail(&x, &message);

	// we cannot do spi i/o on (low) baudrate
	x.speed_hz = spi->max_speed_hz;

	/* Do the i/o */
	ret = spi_sync(spi, &message);
        rxbuf[0] = erx[1];
        rxbuf[1] = erx[0];

	return ret;
}

/* Write a u16 to the device, and return one u16 read back */
static int max3107_out(struct uart_max3107 *max, const u16 out)
{
	u16 tmp;
	int ret;

	ret = max3107_write_then_read(max, (u8 *)&out, (u8 *)&tmp, 2, 1);
	if (ret)
		return ret;

	/* If some valid data is read back */
	if (tmp) {
		if (tmp &= 0xff) {
			receive_chars(max, (unsigned char *)&tmp, 1);
		}
	}

	return ret;
}

/*
 * This is usually used to read data from SPIC RX FIFO, which doesn't
 * need any delay like flushing character out.
 * Returns how many valide bytes are read back
 */
static int max3107_read_multi(struct uart_max3107 *max, int len, u8 *buf)
{
	u16 out[1], in[1];
	u8 *pbuf, valid_str[MAX_READ_LEN+2];
	int i, j, bytelen;
	u8 byte;

	if (len > MAX_READ_LEN) {
		pr_err(PR_FMT "read len %d is too large\n", len);
		return 0;
	}

	bytelen = len;

	memset(out, 0, 1);
	memset(in, 0, 1);

	i=0;
	for(j=0;j<bytelen;j++) {
		// read byte for byte, do not use spi burst mode

		if (max3107_write_then_read(max, (u8 *)out, (u8 *)in, 2, 1))
			return 0;

		/* If caller doesn't provide a buffer, then handle received char */
		pbuf = buf ? buf : valid_str;

		byte = (u8)(in[0] & 0xff);
		if (byte) {
			pbuf[i++] = byte;
		}
	}

	if (i && (pbuf == valid_str)) {
		receive_chars(max, valid_str, i);
	}
	return i;
}

static void serial_m3107_con_putchar(struct uart_port *port, int ch)
{
	struct uart_max3107 *max =
		container_of(port, struct uart_max3107, port);
	struct circ_buf *xmit = &max->con_xmit;

	if (uart_circ_chars_free(xmit)) {
		xmit->buf[xmit->head] = (char)ch;
		xmit->head = (xmit->head + 1) & (PAGE_SIZE - 1);
	}

	if (!atomic_read(&max->con_tx_need)) {
		atomic_set(&max->con_tx_need, 1);
		wake_up_process(max->main_thread);
	}
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void serial_m3107_con_write(struct console *co,
				const char *s, unsigned int count)
{
	if (!pmax)
		return;

	uart_console_write(&pmax->port, s, count, serial_m3107_con_putchar);
}

static int __init
serial_m3107_con_setup(struct console *co, char *options)
{
	struct uart_max3107 *max = pmax;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	pr_info(PR_FMT "setting up console\n");

	if (!max) {
		pr_err(PR_FMT "pmax is NULL, return");
		return -ENODEV;
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&max->port, co, baud, parity, bits, flow);
}

static struct tty_driver *serial_m3107_con_device(struct console *co,
							int *index)
{
	struct uart_driver *p = co->data;
	*index = co->index;
	return p->tty_driver;
}

static struct uart_driver serial_m3107_reg;
static struct console serial_m3107_console = {
	.name		= "ttyMAX",
	.write		= serial_m3107_con_write,
	.device		= serial_m3107_con_device,
	.setup		= serial_m3107_con_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_m3107_reg,
};

#define MRST_CONSOLE	(&serial_m3107_console)

static unsigned int serial_m3107_tx_empty(struct uart_port *port)
{
	return 1;
}

static void serial_m3107_stop_tx(struct uart_port *port)
{
	return;
}

/* stop_rx will be called in spin_lock env */
static void serial_m3107_stop_rx(struct uart_port *port)
{
	return;
}

static inline void send_circ_buf(struct uart_max3107 *max,
				struct circ_buf *xmit)
{
	int len, left = 0;
	u16 obuf[1], ibuf[1];
	u8 valid_str[WORDS_PER_XFER+2];
	int k;
	u8 byte;
	u16 config, tmp;

	while (!uart_circ_empty(xmit)) {
		left = uart_circ_chars_pending(xmit);

		// check how many bytes fit into the tx fifo
                config = MAX3107_TXFIFOLVL_REG;
                max3107_write_then_read(max, (u8 *)&config, (u8 *)&tmp, 2, 1);
                len = (tmp & MAX3107_SPI_RX_DATA_MASK);

		if (left>(128-len)) { // max3107 has 128 word fifo
			left=(128-len);
		}

		while (left) {

			memset(obuf, 0, 1);
			memset(ibuf, 0, 1);

			obuf[0] = (u8)xmit->buf[xmit->tail] | (MAX3107_WRITE_BIT | MAX3107_THR_REG);

			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

			max3107_write_then_read(max, (u8 *)obuf, (u8 *)ibuf, 2, 1);

			k=0;
			if (ibuf[0]) {

				byte = (u8)(ibuf[0] & 0xff);
                                if (byte) {
					valid_str[k++] = byte;
				}
			}
			if (k) {
				receive_chars(max, valid_str, k);
			}
			max->port.icount.tx += 1;
			left -= 1;
		}

	}

}

static void transmit_char(struct uart_max3107 *max)
{
	struct uart_port *port = &max->port;
	struct circ_buf *xmit = &port->state->xmit;

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;

	send_circ_buf(max, xmit);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		serial_m3107_stop_tx(port);
}

/*
 * This will be called by uart_write() and tty_write, can't
 * go to sleep
 */
static void serial_m3107_start_tx(struct uart_port *port)
{
	struct uart_max3107 *max =
		container_of(port, struct uart_max3107, port);

	if (!atomic_read(&max->uart_tx_need)) {
		atomic_set(&max->uart_tx_need, 1);
		wake_up_process(max->main_thread);
	}
}

static void receive_chars(struct uart_max3107 *max, unsigned char *str, int len)
{
	struct uart_port *port = &max->port;
	struct tty_struct *tty;
	int usable;

	/* If uart is not opened, just return */
	if (!port->state)
		return;

	tty = port->state->port.tty;
	if (!tty)
		return;

	while (len) {
		usable = tty_buffer_request_room(tty, len);
		if (usable) {
			tty_insert_flip_string(tty, str, usable);
			str += usable;
			port->icount.rx += usable;
			tty_flip_buffer_push(tty);
		}
		len -= usable;
	}
}

static void max3107_con_receive(struct uart_max3107 *max)
{
	int loop = 1, num, total = 0;
	u8 recv_buf[512];
        u8 *pbuf;
	u16 config;
	int len;
	u16 tmp;

	pbuf = recv_buf;
	do {
		/* Start by reading current RX FIFO level */
		config = MAX3107_RXFIFOLVL_REG;
		max3107_write_then_read(max, (u8 *)&config, (u8 *)&tmp, 2, 1);
		len = (tmp & MAX3107_SPI_RX_DATA_MASK);

		/* 3107 RX buffer is 128 words */
		num = max3107_read_multi(max, len, pbuf);
		if (num) {
			loop = 150; // stay in this function for (300 loops) 10ms after last byte, to avoid buffer overrun
			pbuf += num;
			total += num;

			if (total >= 500) {
				receive_chars(max, recv_buf, total);
				pbuf = recv_buf;
				total = 0;
			}
		}
	} while (--loop);

	if (total) {
		receive_chars(max, recv_buf, total);
	}

if (max->irq) {
        /* Read IRQ status register to clear int */
        config = MAX3107_IRQSTS_REG;
        max3107_write_then_read(max, (u8 *)&config, (u8 *)&tmp, 2, 1);
}

}

static int max3107_main_thread(void *_max)
{
	struct uart_max3107 *max = _max;
	wait_queue_head_t *wq = &max->wq;
	int ret = 0;
	struct circ_buf *xmit = &max->con_xmit;

	init_waitqueue_head(wq);
	pr_info(PR_FMT "start main thread\n");

	do {
		wait_event_interruptible(*wq, (atomic_read(&max->irq_pending) ||
					       atomic_read(&max->con_tx_need) ||
					     atomic_read(&max->uart_tx_need)) ||
					     kthread_should_stop());
		max->mthread_up = 1;

if (max->irq) {
		if (atomic_read(&max->irq_pending)) {
			max3107_con_receive(max);
			atomic_set(&max->irq_pending, 0);
		}
}

		/* first handle console output */
		if (atomic_read(&max->con_tx_need)) {
			send_circ_buf(max, xmit);
			atomic_set(&max->con_tx_need, 0);
		}

		/* handle uart output */
		if (atomic_read(&max->uart_tx_need)) {
			transmit_char(max);
			atomic_set(&max->uart_tx_need, 0);
		}
		max->mthread_up = 0;
	} while (!kthread_should_stop());

	return ret;
}


irqreturn_t static serial_m3107_irq(int irq, void *dev_id)
{
	struct uart_max3107 *max = dev_id;

	/* Read IRQ status register */

	// do not read spi bus in isr!!!

	/* max3107's irq is a falling edge, not level triggered,
	  * so no need to disable the irq */
	if (!atomic_read(&max->irq_pending)) {
		atomic_inc(&max->irq_pending);
		wake_up_process(max->main_thread);
	}

	return IRQ_HANDLED;
}

/* if don't use RX IRQ, then need a thread to polling read */
static int max3107_read_thread(void *_max)
{
	struct uart_max3107 *max = _max;

	pr_info(PR_FMT "start read thread\n");
	do {
		if (!max->mthread_up)
			max3107_con_receive(max);

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 20);
	} while (!kthread_should_stop());

	return 0;
}


static int m3107_ext_clk(struct uart_max3107 *max)
{
	u16 config = 0;
	u16 tmp;

	/* Uboot has set the clock already, use it here */
	config = MAX3107_CLKSRC_REG;
	max3107_write_then_read(max, (u8 *)&config, (u8 *)&tmp, 2, 1);
	return (tmp & 0x10) ? 1 : 0;
}

static int serial_m3107_startup(struct uart_port *port)
{
	struct uart_max3107 *max =
		container_of(port, struct uart_max3107, port);
	u16 config = 0;
	int ret = 0;
        u16 tmp;

	if (port->line != 0)
		pr_err(PR_FMT "uart port startup failed\n");

	/* Check REV ID to ensure we are talking to what we expect */
	config = MAX3107_REVID_REG;

        ret = max3107_write_then_read(max, (u8 *)&config, (u8 *)&tmp, 2, 1);

	if ((tmp & MAX3107_SPI_RX_DATA_MASK) != MAX3107_REVID1 &&
             (tmp & MAX3107_SPI_RX_DATA_MASK) != MAX3107_REVID2) {
		pr_info(PR_FMT "REVID %x does not match\n",
		 (ret & MAX3107_SPI_RX_DATA_MASK));
		return -ENODEV;
	}

	/* Disable all interrupts */
	config = (MAX3107_WRITE_BIT | MAX3107_IRQEN_REG);

	/* Perform SPI transfer */
	ret = max3107_out(max, config);

	/* Configure clock source */
	config = (MAX3107_WRITE_BIT | MAX3107_CLKSRC_REG);
	if (m3107_ext_clk(max)) {
		/* External clock */
		config |= MAX3107_CLKSRC_EXTCLK_BIT;
		config |= MAX3107_CLKSRC_PLLBYP_BIT;
		pr_info(PR_FMT "Using external 25 MHz clock\n");
	} else {
		/* Internal clock */
		config |= MAX3107_CLKSRC_INTOSC_BIT;
		config |= MAX3107_CLKSRC_PLL_BIT;
		pr_info(PR_FMT "Using internal 3.7252 MHz clock\n");
	}

	/* Perform SPI transfer */
	ret = max3107_out(max, config);

	/* as we use thread to handle tx/rx, need set low latency */
	port->state->port.tty->low_latency = 1;

if (max->irq) {
	ret = request_irq(max->irq, serial_m3107_irq,
				IRQF_TRIGGER_FALLING, "max3107", max);
	if (ret)
		return ret;
} else {
	/* if IRQ is disabled, start a read thread for input data */
	max->read_thread =
		kthread_run(max3107_read_thread, max, "max3107_read");
}

	max->cur_conf = config;
	return 0;
}

static void serial_m3107_shutdown(struct uart_port *port)
{
	struct uart_max3107 *max =
		container_of(port, struct uart_max3107, port);

	if (max->read_thread) {
		kthread_stop(max->read_thread);
		max->read_thread = NULL;
	}

	if (max->irq) {
		free_irq(max->irq, max);
	}

}

static void serial_m3107_release_port(struct uart_port *port)
{
}

static int serial_m3107_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_m3107_config_port(struct uart_port *port, int flags)
{
	/* Give it fake type */
	port->type = PORT_PXA;
}

static int
serial_m3107_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}


static const char *serial_m3107_type(struct uart_port *port)
{
	struct uart_max3107 *max =
		container_of(port, struct uart_max3107, port);
	return max->name;
}

static void
serial_m3107_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_max3107 *max =
		container_of(port, struct uart_max3107, port);
	unsigned char cval;
	unsigned int baud, parity = 0;
	int clk_div = -1;
	u16 config;
	u16 new_conf = max->cur_conf;
	u16 lcr_reg, mode1_reg, irqen_reg;
	int loopback=0;

	switch (termios->c_cflag & CSIZE) {
	case CS7:
		cval = UART_LCR_WLEN7;
		new_conf |= WC_7BIT_WORD;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		new_conf |= WC_8BIT_WORD;
		break;
	}

	baud = uart_get_baud_rate(port, termios, old, 0, 230400);
	
	if (m3107_ext_clk(max)) {
		switch (baud) {
		case 300:
			clk_div = MAX3107_EXT_B300;
			break;
		case 600:
			clk_div = MAX3107_EXT_B600;
			break;
		case 1200:
			clk_div = MAX3107_EXT_B1200;
			break;
		case 2400:
			clk_div = MAX3107_EXT_B2400;
			break;
		case 4800:
			clk_div = MAX3107_EXT_B4800;
			break;
		case 9600:
			clk_div = MAX3107_EXT_B9600;
			break;
		case 19200:
			clk_div = MAX3107_EXT_B19200;
			break;
		case 38400:
			clk_div = MAX3107_EXT_B38400;
			break;
		case 57600:
			clk_div = MAX3107_EXT_B57600;
			break;
		case 115200:
			clk_div = MAX3107_EXT_B115200;
			break;
		case 230400:
			if (max->clock & MAX3107_HIGH_CLK)
				break;
		default:
			/* pick the previous baud rate */
			baud = max->baud;
			clk_div = max->cur_conf & WC_BAUD_DIV_MASK;
			tty_termios_encode_baud_rate(termios, baud, baud);
		}
	} else {
		switch (baud) {
		case 300:
			clk_div = MAX3107_INT_B300;
			break;
		case 600:
			clk_div = MAX3107_INT_B600;
			break;
		case 1200:
			clk_div = MAX3107_INT_B1200;
			break;
		case 2400:
			clk_div = MAX3107_INT_B2400;
			break;
		case 4800:
			clk_div = MAX3107_INT_B4800;
			break;
		case 9600:
			clk_div = MAX3107_INT_B9600;
			break;
		case 19200:
			clk_div = MAX3107_INT_B19200;
			break;
		case 38400:
			clk_div = MAX3107_INT_B38400;
			break;
		case 57600:
			clk_div = MAX3107_INT_B57600;
			break;
		case 115200:
			clk_div = MAX3107_INT_B115200;
			break;
		case 230400:
			if (max->clock & MAX3107_HIGH_CLK)
				break;
		default:
			/* pick the previous baud rate */
			baud = max->baud;
			clk_div = max->cur_conf & WC_BAUD_DIV_MASK;
			tty_termios_encode_baud_rate(termios, baud, baud);
		}
	}

	config = (MAX3107_WRITE_BIT | MAX3107_BRGDIVMSB_REG)
		| ((clk_div >> 16) & MAX3107_SPI_TX_DATA_MASK);

	/* Perform SPI transfer */
	max3107_out(max, config);

	config = (MAX3107_WRITE_BIT | MAX3107_BRGDIVLSB_REG)
		| ((clk_div >> 8) & MAX3107_SPI_TX_DATA_MASK);

	/* Perform SPI transfer */
	max3107_out(max, config);

	config = (MAX3107_WRITE_BIT | MAX3107_BRGCFG_REG)
		| ((clk_div) & 0xff);

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 2. Configure LCR register, 8N1 mode by default */
	lcr_reg = MAX3107_LCR_WORD_LEN_8;
	config = (MAX3107_WRITE_BIT | MAX3107_LCR_REG)
		| lcr_reg;

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 3. Configure MODE 1 register */
	mode1_reg = 0;
	if (max->irq) {
		/* Enable IRQ pin */
		mode1_reg |= MAX3107_MODE1_IRQSEL_BIT;
	}
	/* Disable TX */
	//mode1_reg |= MAX3107_MODE1_TXDIS_BIT;
	//tx_enabled = 0;
	/* RX is enabled */
	//rx_enabled = 1;
	config = (MAX3107_WRITE_BIT | MAX3107_MODE1_REG)
		| mode1_reg;

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 4. Configure MODE 2 register */
	config = (MAX3107_WRITE_BIT | MAX3107_MODE2_REG);

	if (loopback) {
		/* Enable loopback */
		config |= MAX3107_MODE2_LOOPBACK_BIT;
	}
	/* Reset FIFOs */
	config |= MAX3107_MODE2_FIFORST_BIT;
	//tx_fifo_empty = 1;

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 5. Configure FIFO trigger level register */
	config = (MAX3107_WRITE_BIT | MAX3107_FIFOTRIGLVL_REG);
	/* RX FIFO trigger for 16 words, TX FIFO trigger not used */
	config |= (MAX3107_FIFOTRIGLVL_RX(8) | MAX3107_FIFOTRIGLVL_TX(0));

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 6. Configure flow control levels */
	config = (MAX3107_WRITE_BIT | MAX3107_FLOWLVL_REG);
	/* Flow control halt level 96, resume level 48 */
	config |= (MAX3107_FLOWLVL_RES(48) | MAX3107_FLOWLVL_HALT(96));

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 7. Configure flow control */
	config = (MAX3107_WRITE_BIT | MAX3107_FLOWCTRL_REG);
	/* Enable auto CTS and auto RTS flow control */
	config |= (MAX3107_FLOWCTRL_AUTOCTS_BIT | MAX3107_FLOWCTRL_AUTORTS_BIT);

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 8. Configure RX timeout register */
	config = (MAX3107_WRITE_BIT | MAX3107_RXTO_REG);
	/* Timeout after 48 character intervals */
	config |= 0x0030;

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 9. Configure LSR interrupt enable register */
	//config = (MAX3107_WRITE_BIT | MAX3107_LSR_IRQEN_REG);
	/* Enable RX timeout interrupt */
	//config |= MAX3107_LSR_RXTO_BIT;

	/* Perform SPI transfer */
	//max3107_out(max, config);

	/* 10. Clear IRQ status register by reading it */
	config = MAX3107_IRQSTS_REG;

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 11. Configure interrupt enable register */
	/* Enable LSR interrupt */
	irqen_reg = MAX3107_IRQ_LSR_BIT;
	/* Enable RX FIFO interrupt */
	irqen_reg |= (MAX3107_IRQ_RXFIFO_BIT /*| MAX3107_IRQ_TXEMPTY_BIT*/);
	config = (MAX3107_WRITE_BIT | MAX3107_IRQEN_REG)
		| irqen_reg;

	/* Perform SPI transfer */
	max3107_out(max, config);

	/* 12. Clear FIFO reset that was set in step 6 */
	config = (MAX3107_WRITE_BIT | MAX3107_MODE2_REG);
	if (loopback) {
		/* Keep loopback enabled */
		config |= MAX3107_MODE2_LOOPBACK_BIT;
	}

	/* Perform SPI transfer */
	max3107_out(max, config);

	if (max->clock & MAX3107_HIGH_CLK) {
		clk_div += 1;
		/* high clk version max3107 doesn't support B300 */
		if (baud == 300)
			baud = 600;
		if (baud == 230400)
			clk_div = WC_BAUD_DR1;
		tty_termios_encode_baud_rate(termios, baud, baud);
	}

	new_conf = (new_conf & ~WC_BAUD_DIV_MASK) | clk_div;
	if (termios->c_cflag & CSTOPB)
		new_conf |= WC_2_STOPBITS;
	else
		new_conf &= ~WC_2_STOPBITS;

	if (termios->c_cflag & PARENB) {
		new_conf |= WC_PARITY_ENABLE;
		parity |= UART_LCR_PARITY;
	} else
		new_conf &= ~WC_PARITY_ENABLE;

	if (!(termios->c_cflag & PARODD))
		parity |= UART_LCR_EPAR;
	max->parity = parity;

	uart_update_timeout(port, termios->c_cflag, baud);

	new_conf |= WC_TAG;
	if (new_conf != max->cur_conf) {
		//max3107_out(max, new_conf);
		max->cur_conf = new_conf;
		max->baud = baud;
	}
}

/* Don't handle hw handshaking */
static unsigned int serial_m3107_get_mctrl(struct uart_port *port)
{
	return TIOCM_DSR | TIOCM_CAR | TIOCM_DSR;
}

static void serial_m3107_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void serial_m3107_break_ctl(struct uart_port *port, int break_state)
{
}

static void serial_m3107_pm(struct uart_port *port, unsigned int state,
			unsigned int oldstate)
{
}

static void serial_m3107_enable_ms(struct uart_port *port)
{
}

struct uart_ops serial_m3107_ops = {
	.tx_empty	= serial_m3107_tx_empty,
	.set_mctrl	= serial_m3107_set_mctrl,
	.get_mctrl	= serial_m3107_get_mctrl,
	.stop_tx	= serial_m3107_stop_tx,
	.start_tx	= serial_m3107_start_tx,
	.stop_rx	= serial_m3107_stop_rx,
	.enable_ms	= serial_m3107_enable_ms,
	.break_ctl	= serial_m3107_break_ctl,
	.startup	= serial_m3107_startup,
	.shutdown	= serial_m3107_shutdown,
	.set_termios	= serial_m3107_set_termios,
	.pm		= serial_m3107_pm,
	.type		= serial_m3107_type,
	.release_port	= serial_m3107_release_port,
	.request_port	= serial_m3107_request_port,
	.config_port	= serial_m3107_config_port,
	.verify_port	= serial_m3107_verify_port,
};

static struct uart_driver serial_m3107_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "MAX3107 console",
	.dev_name	= "ttyMAX",
	.major		= 204,
	.minor		= 209,
	.nr		= 1,
	.cons		= MRST_CONSOLE,
};

static int serial_m3107_suspend(struct spi_device *spi, pm_message_t state)
{
	return 0;
}

static int serial_m3107_resume(struct spi_device *spi)
{
	return 0;
}

#ifdef CONFIG_MAX3107_DESIGNWARE
static struct dw_spi_chip spi_uart = {
	.poll_mode = 1,
	.enable_dma = 0,
	.type = SPI_FRF_SPI,
};
#endif

static int __devinit serial_m3107_probe(struct spi_device *spi)
{
	struct uart_max3107 *max;
	int ret;
	unsigned char *buffer;

	max = kzalloc(sizeof(*max), GFP_KERNEL);
	if (!max)
		return -ENOMEM;

	/* set spi info */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 16;
#ifdef CONFIG_MAX3107_DESIGNWARE
	spi->controller_data = &spi_uart;
#endif
	spi_setup(spi);

	max->clock = MAX3107_HIGH_CLK;
	max->port.type = PORT_PXA;	/* Need apply for a max3107 type */
	max->port.fifosize = 128;		/* 128 word buffer */
	max->port.ops = &serial_m3107_ops;
	max->port.line = 0;
	max->port.dev = &spi->dev;
	max->port.uartclk = 115200;

	max->spi = spi;
	max->name = spi->modalias;	/* use spi name as the name */
	max->irq = (u16)spi->irq;

	spin_lock_init(&max->lock);

	max->word_7bits = 0;
	max->parity = 0;
	max->baud = 0;

	max->cur_conf = 0;
	atomic_set(&max->irq_pending, 0);

	buffer = (unsigned char *)__get_free_page(GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto err_get_page;
	}
	max->con_xmit.buf = (unsigned char *)buffer;
	max->con_xmit.head = max->con_xmit.tail = 0;

	max->main_thread = kthread_run(max3107_main_thread,
					max, "max3107_main");
	if (IS_ERR(max->main_thread)) {
		ret = PTR_ERR(max->main_thread);
		goto err_kthread;
	}

	pmax = max;
	/* Give membase a psudo value to pass serial_core's check */
	max->port.membase = (void *)0xff110000;
	uart_add_one_port(&serial_m3107_reg, &max->port);

	return 0;

err_kthread:
	free_page((unsigned long)buffer);
err_get_page:
	pmax = NULL;
	kfree(max);
	return ret;
}

static int __devexit serial_max3107_remove(struct spi_device *dev)
{
	struct uart_max3107 *max = spi_get_drvdata(dev);

	if (!max)
		return 0;

	uart_remove_one_port(&serial_m3107_reg, &max->port);

	free_page((unsigned long)max->con_xmit.buf);

	if (max->main_thread)
		kthread_stop(max->main_thread);

	kfree(max);
	return 0;
}

static struct spi_driver uart_max3107_driver = {
	.driver = {
			.name	= "max3107_console",
			.bus	= &spi_bus_type,
			.owner	= THIS_MODULE,
	},
	.probe		= serial_m3107_probe,
	.remove		= __devexit_p(serial_max3107_remove),
	.suspend	= serial_m3107_suspend,
	.resume		= serial_m3107_resume,
};

static int __init serial_m3107_init(void)
{
	int ret = 0;

	ret = uart_register_driver(&serial_m3107_reg);
	if (ret)
		return ret;

	ret = spi_register_driver(&uart_max3107_driver);
	if (ret)
		uart_unregister_driver(&serial_m3107_reg);

	return ret;
}

static void __exit serial_m3107_exit(void)
{
	spi_unregister_driver(&uart_max3107_driver);
	uart_unregister_driver(&serial_m3107_reg);
}

module_init(serial_m3107_init);
module_exit(serial_m3107_exit);

MODULE_ALIAS("max3107-uart");
MODULE_AUTHOR("Feng Tang <feng.tang@xxxxxxxxx>");
