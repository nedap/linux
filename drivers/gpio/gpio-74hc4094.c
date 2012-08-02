/*
 *  74HC4094 - output expander  GPIO driver
 *  Copyright (C) 2010 Gabor Juhos <juhosg@openwrt.org> 
 *  Copyright (C) 2010 Miguel Gaio <miguel.gaio@efixo.com> 
 *  Copyright (C) 2012 Dirkjan Bussink <dirkjan.bussink@nedap.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  Based on nxp_74hc164.c from the OpenWRT project
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/74hc4094.h>

struct gen_74hc4094_chip {
	struct device		*parent;
	struct gpio_chip	gpio_chip;
	struct mutex		lock;
	long			mask;
};

static void gen_74hc4094_set_value(struct gpio_chip *, unsigned, int);

static struct gen_74hc4094_chip *gpio_to_74hc4094(struct gpio_chip *gc)
{
	return container_of(gc, struct gen_74hc4094_chip, gpio_chip);
}

static int gen_74hc4094_direction_output(struct gpio_chip *gc,
					unsigned offset, int val)
{
	gen_74hc4094_set_value(gc, offset, val);
	return 0;
}

static int gen_74hc4094_get_value(struct gpio_chip *gc, unsigned offset)
{

	struct gen_74hc4094_chip *chip = gpio_to_74hc4094(gc);
	int ret;

	mutex_lock(&chip->lock);
	ret = test_bit(offset, &chip->mask);
	mutex_unlock(&chip->lock);
	return ret;
}

static void gen_74hc4094_set_value(struct gpio_chip *gc,
				  unsigned offset, int val)
{
	struct gen_74hc4094_chip *chip;
	struct gen_74hc4094_chip_platform_data *pdata;
	long mask;
	int refresh;
	int i;

	chip = gpio_to_74hc4094(gc);
	pdata = chip->parent->platform_data;

	mutex_lock(&chip->lock);
	if (val)
		refresh = (test_and_set_bit(offset, &chip->mask) != val);
	else
		refresh = (test_and_clear_bit(offset, &chip->mask) != val);

	if (refresh) {
		mask = chip->mask;
		for (i = gc->ngpio; i > 0; --i, mask <<= 1) {
			gpio_set_value(pdata->gpio_pin_data, mask & 0x80);
			gpio_set_value(pdata->gpio_pin_clk, 1);
			gpio_set_value(pdata->gpio_pin_clk, 0);
		}
		gpio_set_value(pdata->gpio_pin_strobe, 1);
		gpio_set_value(pdata->gpio_pin_strobe, 0);
	}
	mutex_unlock(&chip->lock);
}

static int __devinit gen_74hc4094_probe(struct platform_device *pdev)
{
	struct gen_74hc4094_chip_platform_data *pdata;
	struct gen_74hc4094_chip *chip;
	struct gpio_chip *gc;
	int err;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_dbg(&pdev->dev, "no platform data specified\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct gen_74hc4094_chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&pdev->dev, "no memory for private data\n");
		return -ENOMEM;
	}

	err = gpio_request(pdata->gpio_pin_clk, dev_name(&pdev->dev));
	if (err) {
		dev_err(&pdev->dev, "unable to claim gpio %u, err=%d\n",
			pdata->gpio_pin_clk, err);
		goto err_free_chip;
	}

	err = gpio_request(pdata->gpio_pin_data, dev_name(&pdev->dev));
	if (err) {
		dev_err(&pdev->dev, "unable to claim gpio %u, err=%d\n",
			pdata->gpio_pin_data, err);
		goto err_free_clk;
	}

	err = gpio_request(pdata->gpio_pin_strobe, dev_name(&pdev->dev));
	if (err) {
		dev_err(&pdev->dev, "unable to claim gpio %u, err=%d\n",
			pdata->gpio_pin_strobe, err);
		goto err_free_data;
	}

	err = gpio_direction_output(pdata->gpio_pin_clk, 0);
	if (err) {
		dev_err(&pdev->dev,
			"unable to set direction of gpio %u, err=%d\n",
			pdata->gpio_pin_clk, err);
		goto err_free_strobe;
	}

	err = gpio_direction_output(pdata->gpio_pin_data, 0);
	if (err) {
		dev_err(&pdev->dev,
			"unable to set direction of gpio %u, err=%d\n",
			pdata->gpio_pin_data, err);
		goto err_free_strobe;
	}

	err = gpio_direction_output(pdata->gpio_pin_strobe, 0);
	if (err) {
		dev_err(&pdev->dev,
			"unable to set direction of gpio %u, err=%d\n",
			pdata->gpio_pin_data, err);
		goto err_free_strobe;
	}


	chip->parent = &pdev->dev;
	chip->mask   = pdata->mask;
	mutex_init(&chip->lock);

	gc = &chip->gpio_chip;

	gc->direction_input  = NULL;
	gc->direction_output = gen_74hc4094_direction_output;
	gc->get = gen_74hc4094_get_value;
	gc->set = gen_74hc4094_set_value;
	gc->can_sleep = 1;
	gc->exported = 1;

	gc->base = pdata->base;
	gc->ngpio = pdata->ngpio;
	gc->label = dev_name(chip->parent);
	gc->dev = chip->parent;
	gc->owner = THIS_MODULE;

	err = gpiochip_add(&chip->gpio_chip);
	if (err) {
		dev_err(&pdev->dev, "unable to add gpio chip, err=%d\n", err);
		goto err_free_data;
	}

	platform_set_drvdata(pdev, chip);
	return 0;
 err_free_strobe:
	gpio_free(pdata->gpio_pin_strobe);
 err_free_data:
	gpio_free(pdata->gpio_pin_data);
 err_free_clk:
	gpio_free(pdata->gpio_pin_clk);
 err_free_chip:
	kfree(chip);
	return err;
}

static int gen_74hc4094_remove(struct platform_device *pdev)
{
	struct gen_74hc4094_chip *chip = platform_get_drvdata(pdev);
	struct gen_74hc4094_chip_platform_data *pdata = pdev->dev.platform_data;

	if (chip) {
		int err;

		err = gpiochip_remove(&chip->gpio_chip);
		if (err) {
			dev_err(&pdev->dev,
				"unable to remove gpio chip, err=%d\n",
				err);
			return err;
		}

		gpio_free(pdata->gpio_pin_strobe);
		gpio_free(pdata->gpio_pin_clk);
		gpio_free(pdata->gpio_pin_data);

		kfree(chip);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static struct platform_driver gen_74hc4094_driver = {
	.probe		= gen_74hc4094_probe,
	.remove		= __devexit_p(gen_74hc4094_remove),
	.driver = {
		.name	= "74hc4094",
		.owner	= THIS_MODULE,
	},
};

static int __init gen_74hc4094_init(void)
{
	return platform_driver_register(&gen_74hc4094_driver);
}
subsys_initcall(gen_74hc4094_init);

static void __exit gen_74hc4094_exit(void)
{
	platform_driver_unregister(&gen_74hc4094_driver);
}
module_exit(gen_74hc4094_exit);

MODULE_AUTHOR("Gabor Juhos <juhosg@openwrt.org>");
MODULE_AUTHOR("Miguel Gaio <miguel.gaio@efixo.com>");
MODULE_AUTHOR("Dirkjan Bussink <dirkjan.bussink@nedap.com>");
MODULE_DESCRIPTION("GPIO based GPIO expander driver for 74HC4094");
MODULE_LICENSE("GPL v2");
