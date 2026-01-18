// SPDX-License-Identifier: GPL-2.0-only
/*
 * RPMh regulator driver
 *
 * Copyright (c) 2018-2024, Qualcomm Innovation Center, Inc.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <soc/qcom/cmd-db.h>
#include <soc/qcom/rpmh.h>

#define RPMH_ARC_MAX_LEN		16
#define RPMH_VREG_TYPE_MAX_LEN		16

struct rpmh_vreg {
	struct device			*dev;
	struct regulator_dev		*rdev;
	struct rpmh_resource		*res;
	const char			*name;
	u32				addr;
	u32				current_uV;
	u32				enabled;
	int				uv_override; /* -1 = disabled */
};

struct rpmh_regulator_data {
	const char			*name;
	const char			*type;
};

static const struct rpmh_regulator_data rpmh_regulators[] = {
	{ .name = "ldo", .type = "ldo" },
	{ .name = "smps", .type = "smps" },
	{ .name = "bob", .type = "bob" },
};

static int rpmh_regulator_enable(struct regulator_dev *rdev)
{
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);

	if (vreg->enabled)
		return 0;

	rpmh_write_async(vreg->res, RPMH_ACTIVE_ONLY_STATE,
			 vreg->addr, 1);
	vreg->enabled = 1;

	return 0;
}

static int rpmh_regulator_disable(struct regulator_dev *rdev)
{
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);

	if (!vreg->enabled)
		return 0;

	rpmh_write_async(vreg->res, RPMH_ACTIVE_ONLY_STATE,
			 vreg->addr, 0);
	vreg->enabled = 0;

	return 0;
}

static int rpmh_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);

	return vreg->enabled;
}

static int rpmh_regulator_set_voltage(struct regulator_dev *rdev,
				      int min_uV, int max_uV,
				      unsigned int *selector)
{
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);
	int uV;

	/* uv_override handling */
	if (vreg->uv_override >= 0) {
		min_uV = vreg->uv_override;
		max_uV = vreg->uv_override;
	}

	uV = min_uV;

	rpmh_write_async(vreg->res, RPMH_ACTIVE_ONLY_STATE,
			 vreg->addr, uV);
	vreg->current_uV = uV;

	return 0;
}

static int rpmh_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);

	if (vreg->uv_override >= 0)
		return vreg->uv_override;

	return vreg->current_uV;
}

/* ================= uv_override sysfs ================= */

static ssize_t uv_override_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 vreg->uv_override);
}

static ssize_t uv_override_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if (val < 0)
		vreg->uv_override = -1;
	else
		vreg->uv_override = val;

	return count;
}

static DEVICE_ATTR_RW(uv_override);

/* ==================================================== */

static const struct regulator_ops rpmh_regulator_ops = {
	.enable		= rpmh_regulator_enable,
	.disable	= rpmh_regulator_disable,
	.is_enabled	= rpmh_regulator_is_enabled,
	.set_voltage	= rpmh_regulator_set_voltage,
	.get_voltage	= rpmh_regulator_get_voltage,
};

static int rpmh_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpmh_vreg *vreg;
	struct regulator_config config = {};
	struct regulator_desc *rdesc;
	const char *name;
	int ret;

	ret = of_property_read_string(dev->of_node, "regulator-name",
				      &name);
	if (ret)
		return ret;

	vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg)
		return -ENOMEM;

	rdesc = devm_kzalloc(dev, sizeof(*rdesc), GFP_KERNEL);
	if (!rdesc)
		return -ENOMEM;

	vreg->dev = dev;
	vreg->name = name;
	vreg->uv_override = -1;

	vreg->res = rpmh_get_byname(dev, name);
	if (IS_ERR(vreg->res))
		return PTR_ERR(vreg->res);

	rdesc->name = name;
	rdesc->ops = &rpmh_regulator_ops;
	rdesc->type = REGULATOR_VOLTAGE;
	rdesc->owner = THIS_MODULE;

	config.dev = dev;
	config.driver_data = vreg;
	config.of_node = dev->of_node;

	vreg->rdev = devm_regulator_register(dev, rdesc, &config);
	if (IS_ERR(vreg->rdev))
		return PTR_ERR(vreg->rdev);

	device_create_file(&vreg->rdev->dev, &dev_attr_uv_override);

	dev_info(dev, "Registered RPMh regulator %s (uv_override enabled)\n",
		 name);

	return 0;
}

static const struct of_device_id rpmh_regulator_match[] = {
	{ .compatible = "qcom,rpmh-regulator" },
	{}
};
MODULE_DEVICE_TABLE(of, rpmh_regulator_match);

static struct platform_driver rpmh_regulator_driver = {
	.probe = rpmh_regulator_probe,
	.driver = {
		.name = "qcom-rpmh-regulator",
		.of_match_table = rpmh_regulator_match,
	},
};

module_platform_driver(rpmh_regulator_driver);

MODULE_DESCRIPTION("Qualcomm RPMh regulator driver with uv_override");
MODULE_LICENSE("GPL v2");
