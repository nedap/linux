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
#define PFEW		4
#define FACTRST		36
#define DSR0		37
#define DTR0		38
#define RI0		41
#define DCD0		42
#define STS_LED		39

static struct mv643xx_eth_platform_data nedap_ax8008_ge00_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(8),
};

static struct mvsdio_platform_data nedap_ax8008_mvsdio_data = {
};

static const struct flash_platform_data nedap_ax8008_spi_slave_data = {
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
		//.irq		= (IRQ_KIRKWOOD_GPIO_START + MAX3107_INT),
		.max_speed_hz   = 26000000,
		.bus_num	= 0,
		.chip_select	= 1,
	},
};

static unsigned int nedap_ax8008_mpp_config[] __initdata = {
	MPP0_GPIO,  /* SPI FLASH CS */
        MPP4_GPIO,  /* PFEW */
	MPP7_GPO, /*MPP7_SPI_SCn,*/ /* SPI UART CS */
	MPP18_GPO,  /* I2C SCL */
	MPP35_GPIO, /* SPI UART IRQ */
        MPP36_GPIO, /* FACT RST */
	MPP37_GPIO,
	MPP38_GPIO,
	MPP39_GPIO, /* STS LED */
	MPP41_GPIO,
	MPP42_GPIO,
	MPP43_GPIO, /* I2C SDA */
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

	gpio_request(DTR0, "dtr0");
	gpio_direction_input(DTR0);
	gpio_request(DSR0, "dsr0");
	gpio_direction_input(DSR0);
	gpio_request(RI0, "ri0");
	gpio_direction_input(RI0);
	gpio_request(DCD0, "dcd0");
	gpio_direction_input(DCD0);

	kirkwood_ehci_init();
	kirkwood_ge00_init(&nedap_ax8008_ge00_data);

	spi_register_board_info(nedap_ax8008_spi_slave_info,
                                ARRAY_SIZE(nedap_ax8008_spi_slave_info));
	spi_register_board_info(nedap_ax8008_spi_uart_info,
                                ARRAY_SIZE(nedap_ax8008_spi_uart_info));

	kirkwood_spi_init();
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
