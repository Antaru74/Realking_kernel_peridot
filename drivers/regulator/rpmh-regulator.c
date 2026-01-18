// SPDX-License-Identifier: GPL-2.0-only
/*
 * RPMh regulator driver with uv_override support
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/device.h>

#include <soc/qcom/rpmh.h>

struct rpmh_vreg {
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	struct rpmh_client *client;

	int voltage_uv;
	int uv_override;
	bool use_uv_override;

	u32 addr;
	u32 resource_id;
	struct mutex lock;
};

/* ========================================================= */
/* Voltage handling                                           */
/* ========================================================= */

static int rpmh_vreg_send_voltage(struct rpmh_vreg *vreg, int uv)
{
	u32 val;

	/* uV → RPMh encoding（既存処理） */
	val = DIV_ROUND_UP(uv, 1000);

	return rpmh_write_async(vreg->client, RPMH_ACTIVE_ONLY,
				vreg->addr, val);
}

static int rpmh_regulator_set_voltage(struct regulator_dev *rdev,
				      int min_uv, int max_uv,
				      unsigned int *selector)
{
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);
	int uv;
	int ret;

	mutex_lock(&vreg->lock);

	if (vreg->use_uv_override)
		uv = vreg->uv_override;
	else
		uv = min_uv;

	ret = rpmh_vreg_send_voltage(vreg, uv);
	if (!ret)
		vreg->voltage_uv = uv;

	mutex_unlock(&vreg->lock);
	return ret;
}

static int rpmh_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);

	return vreg->voltage_uv;
}

/* ========================================================= */
/* sysfs: uv_override                                         */
/* ========================================================= */

static ssize_t uv_override_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 vreg->use_uv_override ? vreg->uv_override : 0);
}

static ssize_t uv_override_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	struct rpmh_vreg *vreg = rdev_get_drvdata(rdev);
	int uv;
	int ret;

	if (kstrtoint(buf, 10, &uv))
		return -EINVAL;

	mutex_lock(&vreg->lock);

	if (uv <= 0) {
		vreg->use_uv_override = false;
	} else {
		vreg->uv_override = uv;
		vreg->use_uv_override = true;
		ret = rpmh_vreg_send_voltage(vreg, uv);
		if (ret) {
			mutex_unlock(&vreg->lock);
			return ret;
		}
		vreg->voltage_uv = uv;
	}

	mutex_unlock(&vreg->lock);
	return count;
}

static DEVICE_ATTR_RW(uv_override);

/* ========================================================= */
/* regulator ops                                              */
/* ========================================================= */

static const struct regulator_ops rpmh_regulator_ops = {
	.set_voltage = rpmh_regulator_set_voltage,
	.get_voltage = rpmh_regulator_get_voltage,
};

/* ========================================================= */
/* probe                                                      */
/* ========================================================= */

static int rpmh_regulator_probe(struct platform_device *pdev)
{
	struct rpmh_vreg *vreg;
	struct regulator_config config = {};
	int ret;

	vreg = devm_kzalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg)
		return -ENOMEM;

	mutex_init(&vreg->lock);

	vreg->client = rpmh_get_byname(pdev->dev.of_node, "regulator");
	if (IS_ERR(vreg->client))
		return PTR_ERR(vreg->client);

	/* 既存 DT パース処理（省略せず実機に合わせて） */
	of_property_read_u32(pdev->dev.of_node, "qcom,resource-id",
			     &vreg->resource_id);
	of_property_read_u32(pdev->dev.of_node, "qcom,addr",
			     &vreg->addr);

	vreg->desc.name = dev_name(&pdev->dev);
	vreg->desc.type = REGULATOR_VOLTAGE;
	vreg->desc.owner = THIS_MODULE;
	vreg->desc.ops = &rpmh_regulator_ops;

	config.dev = &pdev->dev;
	config.driver_data = vreg;
	config.of_node = pdev->dev.of_node;

	vreg->rdev = devm_regulator_register(&pdev->dev,
					     &vreg->desc, &config);
	if (IS_ERR(vreg->rdev))
		return PTR_ERR(vreg->rdev);

	ret = device_create_file(&vreg->rdev->dev, &dev_attr_uv_override);
	if (ret)
		return ret;

	return 0;
}

static int rpmh_regulator_remove(struct platform_device *pdev)
{
	struct rpmh_vreg *vreg = platform_get_drvdata(pdev);

	device_remove_file(&vreg->rdev->dev, &dev_attr_uv_override);
	return 0;
}

static const struct of_device_id rpmh_regulator_match[] = {
	{ .compatible = "qcom,rpmh-regulator" },
	{}
};
MODULE_DEVICE_TABLE(of, rpmh_regulator_match);

static struct platform_driver rpmh_regulator_driver = {
	.probe  = rpmh_regulator_probe,
	.remove = rpmh_regulator_remove,
	.driver = {
		.name = "rpmh-regulator",
		.of_match_table = rpmh_regulator_match,
	},
};

module_platform_driver(rpmh_regulator_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RPMh regulator driver with uv_override");
