#ifndef LINUX_SPI_74HC4094_H
#define LINUX_SPI_74HC4094_H

struct gen_74hc4094_chip_platform_data {
	/* number assigned to the first GPIO */
	unsigned	base;
	long            mask;
	unsigned        ngpio;
	int gpio_pin_data;
	int gpio_pin_clk;
	int gpio_pin_strobe;
};

#endif
