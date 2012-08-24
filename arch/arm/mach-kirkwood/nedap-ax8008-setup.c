/*
 * arch/arm/mach-kirkwood/nedap_ax8008-bp-setup.c
 *
 * Marvell DB-88F6281-BP Development Board Setup
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/ata_platform.h>
#include <linux/mv643xx_eth.h>
#include <linux/gpio.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/spi/orion_spi.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/spi/74hc4094.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/kirkwood.h>
#include <mach/irqs.h>

#include <plat/mvsdio.h>
#include "common.h"
#include "mpp.h"

#define SPI_CS0		0
#define SPI_CS1		7
#define MAX3107_INT	35
#define MV_SDA_PIN	43
#define MV_SCL_PIN	18
#define STS_LED         39
#define GPIO0		4
#define SHIFT_DATA_PIN	36
#define SHIFT_CLK_PIN	37
#define SHIFT_STRB_PIN	38
#define GPIO5		40
#define GPIO8		41
#define GPIO9		42
#define GPIO14		44

static struct mv643xx_eth_platform_data nedap_ax8008_ge00_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(8),
};

static struct mvsdio_platform_data nedap_ax8008_mvsdio_data = {
};

static struct flash_platform_data nedap_ax8008_spi_slave_data = {
        .type	= "m25p128",
};

static struct spi_board_info __initdata nedap_ax8008_spi_slave_info[] = {
        {
                .modalias       = "m25p80",
                .platform_data  = &nedap_ax8008_spi_slave_data,
                .irq            = -1,
                .max_speed_hz   = 50000000,
                .bus_num        = 0,
                .chip_select    = 0,
        },
};

static struct spi_board_info nedap_ax8008_spi_uart_info[] = {
	{
		.modalias	= "max3107_console",
		.irq		= (IRQ_KIRKWOOD_GPIO_START + MAX3107_INT),
		.max_speed_hz   = 26000000,
		.bus_num	= 0,
		.chip_select	= 1,
	},
};

static struct gen_74hc4094_chip_platform_data nedap_ax8008_gpio_shift_register = {
	.ngpio           = 8,
	.base            = 100,
	.mask            = 0xE0,
	.gpio_pin_data   = SHIFT_DATA_PIN,
	.gpio_pin_clk    = SHIFT_CLK_PIN,
	.gpio_pin_strobe = SHIFT_STRB_PIN,
};

static struct platform_device nedap_ax8008_gpio_info = {
	.name	= "74hc4094",
	.id	= 0,
	.dev	= {
		.platform_data  = &nedap_ax8008_gpio_shift_register,
	},
};

static struct i2c_gpio_platform_data nedap_ax8008_i2c_gpio_data = {
        .sda_pin        = MV_SDA_PIN,
        .scl_pin        = MV_SCL_PIN,
        .scl_is_output_only = 1,
};

static struct platform_device nedap_ax8008_i2c_gpio = {
	.name           = "i2c-gpio",
	.id             = 0,
	.dev     = {
		.platform_data  = &nedap_ax8008_i2c_gpio_data,
	},
};

static struct platform_device *nedap_ax8008_devices[] __initdata = {
	&nedap_ax8008_i2c_gpio,
	&nedap_ax8008_gpio_info,
};

static unsigned int nedap_ax8008_mpp_config[] __initdata = {
	MPP0_GPIO,  	/* SPI FLASH CS */
        MPP1_SPI_MOSI,
        MPP2_SPI_SCK,
        MPP3_SPI_MISO,
        MPP4_GPIO,  	/* GPIO0 */
        MPP5_GPO,   	/* not used */
        MPP6_SYSRST_OUTn,
        MPP7_GPO,   	/* SPI UART CS */
        MPP8_UART0_RTS,
        MPP9_UART0_CTS,
        MPP10_UART0_TXD,
        MPP11_UART0_RXD,
        MPP12_SD_CLK,
        MPP13_SD_CMD,
        MPP14_SD_D0,
        MPP15_SD_D1,
        MPP16_SD_D2,
        MPP17_SD_D3,
        MPP18_GPO,  	/* (GPIO6) I2C SCL */
        MPP19_GPO,  	/* not used */

        MPP35_GPIO, 	/* (GPIO4) SPI UART IRQ */
        MPP36_GPIO, 	/* SHIFT_DATA_PIN */
        MPP37_GPIO, 	/* SHIFT_CLK_PIN */
        MPP38_GPIO, 	/* SHIFT_STRB_PIN */
        MPP39_GPIO, 	/* (GPIO12) STS LED */
        MPP40_GPIO, 	/* GPIO5 */
        MPP41_GPIO, 	/* GPIO8 */
        MPP42_GPIO, 	/* GPIO9 */
        MPP43_GPIO, 	/* (GPIO7) I2C SDA */
        MPP44_GPIO, 	/* GPIO14 */
        0
};

static void __init nedap_ax8008_init(void)
{
	/*
	 * Basic setup. Needs to be called early.
	 */
	kirkwood_init();
	kirkwood_mpp_conf(nedap_ax8008_mpp_config);

	gpio_request(MAX3107_INT, "max3107_int");
	gpio_direction_input(MAX3107_INT);
	gpio_request(SPI_CS0, "spi_cs0");
	gpio_direction_output(SPI_CS0, 1);
	gpio_request(SPI_CS1, "spi_cs1");
	gpio_direction_output(SPI_CS1, 1);

	gpio_request(GPIO0, "gpio0");
	gpio_direction_input(GPIO0);
	gpio_request(GPIO5, "gpio5");
	gpio_direction_input(GPIO5);
	gpio_request(GPIO8, "gpio8");
	gpio_direction_input(GPIO8);
	gpio_request(GPIO9, "gpio9");
	gpio_direction_input(GPIO9);
	gpio_request(GPIO14, "gpio14");
	gpio_direction_input(GPIO14);

	kirkwood_ehci_init();
	kirkwood_ge00_init(&nedap_ax8008_ge00_data);

	spi_register_board_info(nedap_ax8008_spi_slave_info,
                                ARRAY_SIZE(nedap_ax8008_spi_slave_info));
	spi_register_board_info(nedap_ax8008_spi_uart_info,
                                ARRAY_SIZE(nedap_ax8008_spi_uart_info));

	kirkwood_spi_init();
	platform_add_devices(nedap_ax8008_devices, ARRAY_SIZE(nedap_ax8008_devices));
	kirkwood_uart0_init();
	kirkwood_sdio_init(&nedap_ax8008_mvsdio_data);
}

static int __init nedap_ax8008_pci_init(void)
{
	if (machine_is_nedap_ax8008()) {
		u32 dev, rev;

		kirkwood_pcie_id(&dev, &rev);
		if (dev == MV88F6282_DEV_ID)
			kirkwood_pcie_init(KW_PCIE1 | KW_PCIE0);
		else
			kirkwood_pcie_init(KW_PCIE0);
	}
	return 0;
}
subsys_initcall(nedap_ax8008_pci_init);

MACHINE_START(NEDAP_AX8008, "Nedap AX8008 Board")
	/* Maintainer: Dirkjan Bussink <dirkjan.bussink@nedap.com> */
	.atag_offset	= 0x100,
	.init_machine	= nedap_ax8008_init,
	.map_io		= kirkwood_map_io,
	.init_early	= kirkwood_init_early,
	.init_irq	= kirkwood_init_irq,
	.timer		= &kirkwood_timer,
	.restart	= kirkwood_restart,
MACHINE_END
