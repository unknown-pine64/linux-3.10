/*
 * reg-virtual-consumer.c
 *
 * Copyright 2008 X-POWERS Ltd.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/module.h>
#include "../virtual.h"

static int regulator_virtual_consumer_probe(struct platform_device *pdev)
{
	char *reg_id = pdev->dev.platform_data;
	struct virtual_consumer_data *drvdata;
	int ret, i;
	char *pmu_name = get_pmu_cur_name();

	if (strncmp("axp22", pmu_name, 5))
		return 0;

	drvdata = kzalloc(sizeof(struct virtual_consumer_data), GFP_KERNEL);
	if (drvdata == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_init(&drvdata->lock);
	sprintf(drvdata->regu_name, reg_id);

	for (i = 0; i < ARRAY_SIZE(attributes_virtual); i++) {
		ret = device_create_file(&pdev->dev, attributes_virtual[i]);
		if (ret != 0)
			goto err;
	}

	platform_set_drvdata(pdev, drvdata);

	return 0;

err:
	for (i = 0; i < ARRAY_SIZE(attributes_virtual); i++)
		device_remove_file(&pdev->dev, attributes_virtual[i]);
	kfree(drvdata);
	return ret;
}

static int regulator_virtual_consumer_remove(struct platform_device *pdev)
{
	struct virtual_consumer_data *drvdata = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes_virtual); i++)
		device_remove_file(&pdev->dev, attributes_virtual[i]);

	kfree(drvdata);

	return 0;
}

static struct platform_driver regulator_virtual_consumer_driver[] = {
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-rtc",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-aldo1",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-aldo2",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-aldo3",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dldo1",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dldo2",
		},
	},
#ifdef CONFIG_AXP809
#else
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dldo3",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dldo4",
		},
	},
#endif
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-eldo1",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-eldo2",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-eldo3",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dcdc1",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dcdc2",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dcdc3",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dcdc4",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dcdc5",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-gpio0ldo",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-gpio1ldo",
		},
	},
	{
		.probe      = regulator_virtual_consumer_probe,
		.remove     = regulator_virtual_consumer_remove,
		.driver     = {
			.name   = "reg-22-cs-dc5ldo",
		},
	},
};

static int __init regulator_virtual_consumer_init(void)
{
	int j, ret;

	for (j = 0; j < ARRAY_SIZE(regulator_virtual_consumer_driver); j++) {
		ret = platform_driver_register(
				&regulator_virtual_consumer_driver[j]);
		if (ret)
			goto creat_drivers_failed;
	}

	return ret;

creat_drivers_failed:
	while (j--)
		platform_driver_unregister(
				&regulator_virtual_consumer_driver[j]);
	return ret;
}
module_init(regulator_virtual_consumer_init);

static void __exit regulator_virtual_consumer_exit(void)
{
	int j;

	for (j = ARRAY_SIZE(regulator_virtual_consumer_driver) - 1; j >= 0; j--)
		platform_driver_unregister(
			&regulator_virtual_consumer_driver[j]);
}
module_exit(regulator_virtual_consumer_exit);

MODULE_AUTHOR("Weijin Zhong X-POWERS");
MODULE_DESCRIPTION("Virtual regulator consumer");
MODULE_LICENSE("GPL");
