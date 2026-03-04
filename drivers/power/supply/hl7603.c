// SPDX-License-Identifier: GPL-2.0
/*
 * hl7603.c
 *
 * boost bypass ic driver
 *
 * Copyright (c) 2023-2023 Xiaomi Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/printk.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include "hl7603.h"

/* ------------------------------------------------------------------ */
/* Internal register helpers                                            */
/* ------------------------------------------------------------------ */

static int hl7603_read_reg(struct boost_bypass_dev *bq, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		dev_err(bq->dev, "read reg 0x%02x failed: %d\n", reg, ret);
		return ret;
	}
	*val = (u8)ret;
	return 0;
}

static int hl7603_write_reg(struct boost_bypass_dev *bq, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0)
		dev_err(bq->dev, "write reg 0x%02x failed: %d\n", reg, ret);
	return ret;
}

static int hl7603_update_bits(struct boost_bypass_dev *bq,
			      u8 reg, u8 mask, u8 val)
{
	u8 cur;
	int ret;

	ret = hl7603_read_reg(bq, reg, &cur);
	if (ret)
		return ret;

	cur = (cur & ~mask) | (val & mask);
	return hl7603_write_reg(bq, reg, cur);
}

/* ------------------------------------------------------------------ */
/* Voltage threshold (defined early; called by bypass helpers below)   */
/* ------------------------------------------------------------------ */

int hl7603_set_voltage_threshold(struct boost_bypass_dev *bq,
				 u32 vout_threshold)
{
	u8 val = 0;
	int ret;

	if ((vout_threshold > VOUT_REG_MAX) ||
	    vout_threshold < VOUT_REG_BASE)
		return -EINVAL;

	val = (vout_threshold - VOUT_REG_BASE) / VOUT_REG_STEP;

	ret = hl7603_write_reg(bq, VOUT_REG, val);
	if (ret < 0)
		return ret;

	hl7603_read_reg(bq, VOUT_REG, &val);	/* readback */
	return 0;
}

/* ------------------------------------------------------------------ */
/* Bypass mode control                                                  */
/* ------------------------------------------------------------------ */

/*
 * hl7603_set_bypass_mode - enable or disable Forced Bypass Mode
 *
 * enable=true  -> Forced Bypass: VIN connected to VOUT via bypass switch.
 *                 Adapter drives the load directly; no boost conversion loss.
 * enable=false -> Auto mode: IC transitions between boost and bypass
 *                 automatically based on VIN vs VOUT.
 *
 * The caller must hold bq->lock.
 */
static int hl7603_set_bypass_mode(struct boost_bypass_dev *bq, bool enable)
{
	u8 val = enable ? CONFIG1_FORCED_BYPASS_BIT : 0;
	int ret;

	ret = hl7603_update_bits(bq, CONFIG1, CONFIG1_FORCED_BYPASS_BIT, val);
	if (ret) {
		dev_err(bq->dev, "failed to %s bypass mode: %d\n",
			enable ? "enable" : "disable", ret);
		return ret;
	}

	bq->bypass_mode_enabled = enable;
	dev_info(bq->dev, "bypass mode %s\n", enable ? "enabled" : "disabled");
	return 0;
}

/* Read STATUS register to verify actual HW bypass state */
static bool hl7603_is_in_bypass(struct boost_bypass_dev *bq)
{
	u8 status = 0;

	hl7603_read_reg(bq, STATUS, &status);
	return !!(status & STATUS_IN_BYPASS_BIT);
}

/* ------------------------------------------------------------------ */
/* Battery state helpers                                                */
/* ------------------------------------------------------------------ */

static int hl7603_get_battery_capacity(struct boost_bypass_dev *bq)
{
	union power_supply_propval pval = {0};
	int ret;

	if (!bq->batt_psy) {
		bq->batt_psy = power_supply_get_by_name("battery");
		if (!bq->batt_psy)
			return -ENODEV;
	}

	ret = power_supply_get_property(bq->batt_psy,
					POWER_SUPPLY_PROP_CAPACITY, &pval);
	return ret ? ret : pval.intval;
}

static int hl7603_get_charge_status(struct boost_bypass_dev *bq)
{
	union power_supply_propval pval = {0};
	int ret;

	if (!bq->batt_psy)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	ret = power_supply_get_property(bq->batt_psy,
					POWER_SUPPLY_PROP_STATUS, &pval);
	return ret ? POWER_SUPPLY_STATUS_UNKNOWN : pval.intval;
}

/*
 * hl7603_get_usb_online - returns true if USB charger is connected
 *
 * Reads POWER_SUPPLY_PROP_ONLINE from the "usb" power supply.
 * This is the authoritative signal for charger plug/unplug.
 */
static bool hl7603_get_usb_online(struct boost_bypass_dev *bq)
{
	union power_supply_propval pval = {0};
	struct power_supply *usb_psy;
	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy)
		return false;

	ret = power_supply_get_property(usb_psy,
					POWER_SUPPLY_PROP_ONLINE, &pval);
	power_supply_put(usb_psy);
	return ret ? false : !!pval.intval;
}

/*
 * hl7603_get_usb_voltage_uv - returns USB input voltage in uV
 *
 * Used to automatically set VOUT threshold to match the adapter voltage.
 * Returns 0 on error.
 */
static int hl7603_get_usb_voltage_uv(struct boost_bypass_dev *bq)
{
	union power_supply_propval pval = {0};
	struct power_supply *usb_psy;
	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy)
		return 0;

	ret = power_supply_get_property(usb_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	power_supply_put(usb_psy);
	return ret ? 0 : pval.intval;
}

/*
 * hl7603_update_vout_for_adapter - set VOUT threshold to match adapter
 *
 * Snaps the measured USB voltage to the nearest standard PD/QC level
 * (5 V / 9 V / 12 V / 20 V) and programmes VOUT_REG accordingly.
 * A small headroom (VOUT_HEADROOM_MV) is subtracted so the bypass
 * switch closes cleanly before the IC tries to boost.
 *
 * Called whenever the charger is (re-)connected or the voltage changes.
 */
#define VOUT_HEADROOM_MV	100	/* subtract 100 mV as safety margin */

static void hl7603_update_vout_for_adapter(struct boost_bypass_dev *bq)
{
	int uv = hl7603_get_usb_voltage_uv(bq);
	u32 mv, target_mv;

	if (uv <= 0)
		return;

	mv = uv / 1000;

	/* Snap to nearest standard adapter voltage */
	if (mv >= 18000)
		target_mv = 20000;
	else if (mv >= 10500)
		target_mv = 12000;
	else if (mv >= 7500)
		target_mv = 9000;
	else
		target_mv = 5000;

	/* Apply headroom and clamp to register range */
	target_mv -= VOUT_HEADROOM_MV;
	target_mv = clamp(target_mv, (u32)VOUT_REG_BASE, (u32)VOUT_REG_MAX);

	if (target_mv != bq->vout_threshold) {
		dev_info(bq->dev,
			 "adapter %u mV -> VOUT threshold %u mV\n",
			 mv, target_mv);
		hl7603_set_voltage_threshold(bq, target_mv);
		bq->vout_threshold = target_mv;
	}
}

/* ------------------------------------------------------------------ */
/* Bypass decision worker                                               */
/* ------------------------------------------------------------------ */

/*
 * hl7603_bypass_check_work - periodic worker that manages bypass on/off
 *
 * Policy:
 *   Enter Forced Bypass when battery is full (SOC >= BYPASS_SOC_THRESHOLD)
 *   and the charger is still connected.  The adapter then sustains the load
 *   with zero boost-conversion loss (pass-through charging).
 *
 *   Exit Forced Bypass when SOC falls below the threshold or the charger
 *   is removed, reverting to normal boost operation.
 */
static void hl7603_bypass_check_work(struct work_struct *work)
{
	struct boost_bypass_dev *bq =
		container_of(work, struct boost_bypass_dev,
			     bypass_check_work.work);
	int soc, status;
	bool usb_online, want_bypass;

	mutex_lock(&bq->lock);

	usb_online = hl7603_get_usb_online(bq);

	/* If charger is disconnected, exit bypass immediately */
	if (!usb_online) {
		if (bq->bypass_mode_enabled) {
			dev_info(bq->dev, "charger removed, exiting bypass\n");
			hl7603_set_bypass_mode(bq, false);
		}
		mutex_unlock(&bq->lock);
		schedule_delayed_work(&bq->bypass_check_work,
				      msecs_to_jiffies(BYPASS_CHECK_INTERVAL_MS));
		return;
	}

	/* Charger is connected: update VOUT to match adapter voltage */
	hl7603_update_vout_for_adapter(bq);

	soc    = hl7603_get_battery_capacity(bq);
	status = hl7603_get_charge_status(bq);

	want_bypass = (soc >= BYPASS_SOC_THRESHOLD) &&
		      (status == POWER_SUPPLY_STATUS_FULL ||
		       status == POWER_SUPPLY_STATUS_CHARGING);

	if (want_bypass != bq->bypass_mode_enabled) {
		dev_dbg(bq->dev,
			"bypass: %s -> %s (soc=%d status=%d)\n",
			bq->bypass_mode_enabled ? "on" : "off",
			want_bypass ? "on" : "off", soc, status);
		hl7603_set_bypass_mode(bq, want_bypass);
	}

	mutex_unlock(&bq->lock);

	schedule_delayed_work(&bq->bypass_check_work,
			      msecs_to_jiffies(BYPASS_CHECK_INTERVAL_MS));
}

/* ------------------------------------------------------------------ */
/* Power-supply event notifier                                          */
/* ------------------------------------------------------------------ */

static int hl7603_psy_notifier_call(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct boost_bypass_dev *bq =
		container_of(nb, struct boost_bypass_dev, psy_nb);

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_DONE;

	if (!strcmp(psy->desc->name, "usb")) {
		/*
		 * USB state changed (plug/unplug or voltage step).
		 * If charger was just removed, exit bypass immediately
		 * without waiting for the next periodic check.
		 */
		if (!hl7603_get_usb_online(bq)) {
			mutex_lock(&bq->lock);
			if (bq->bypass_mode_enabled) {
				dev_info(bq->dev,
					 "USB removed (notifier), exiting bypass immediately\n");
				hl7603_set_bypass_mode(bq, false);
			}
			mutex_unlock(&bq->lock);
		} else {
			/* Voltage may have changed (e.g. PPS step-up); update VOUT */
			mutex_lock(&bq->lock);
			hl7603_update_vout_for_adapter(bq);
			mutex_unlock(&bq->lock);
		}
		/* Also kick the worker for a full re-evaluation */
		mod_delayed_work(system_wq, &bq->bypass_check_work, 0);

	} else if (!strcmp(psy->desc->name, "battery")) {
		/* SOC or status change -> re-evaluate bypass */
		mod_delayed_work(system_wq, &bq->bypass_check_work, 0);
	}

	return NOTIFY_OK;
}

/* ------------------------------------------------------------------ */
/* sysfs interface                                                      */
/* ------------------------------------------------------------------ */

/*
 * /sys/bus/i2c/devices/<addr>/bypass_mode
 *   read : 1 if HW is currently in bypass, 0 otherwise
 *   write: 1 to force bypass, 0 to restore auto mode
 */
static ssize_t bypass_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct boost_bypass_dev *bq = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", hl7603_is_in_bypass(bq) ? 1 : 0);
}

static ssize_t bypass_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct boost_bypass_dev *bq = dev_get_drvdata(dev);
	int val, ret;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&bq->lock);
	ret = hl7603_set_bypass_mode(bq, !!val);
	mutex_unlock(&bq->lock);

	return ret ? ret : count;
}

/*
 * /sys/bus/i2c/devices/<addr>/vout_threshold
 *   read/write output voltage threshold in mV (2850 - 5500)
 */
static ssize_t vout_threshold_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct boost_bypass_dev *bq = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", bq->vout_threshold);
}

static ssize_t vout_threshold_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct boost_bypass_dev *bq = dev_get_drvdata(dev);
	u32 val;
	int ret;

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&bq->lock);
	ret = hl7603_set_voltage_threshold(bq, val);
	if (!ret)
		bq->vout_threshold = val;
	mutex_unlock(&bq->lock);

	return ret ? ret : count;
}

/*
 * /sys/bus/i2c/devices/<addr>/status
 *   read-only: raw register dump for debugging
 */
static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct boost_bypass_dev *bq = dev_get_drvdata(dev);
	u8 cfg1 = 0, status = 0;

	hl7603_read_reg(bq, CONFIG1, &cfg1);
	hl7603_read_reg(bq, STATUS,  &status);

	return sysfs_emit(buf,
			  "CONFIG1=0x%02x STATUS=0x%02x bypass_sw=%d hw_bypass=%d\n",
			  cfg1, status,
			  bq->bypass_mode_enabled,
			  hl7603_is_in_bypass(bq));
}

static DEVICE_ATTR_RW(bypass_mode);
static DEVICE_ATTR_RW(vout_threshold);
static DEVICE_ATTR_RO(status);

static struct attribute *hl7603_attrs[] = {
	&dev_attr_bypass_mode.attr,
	&dev_attr_vout_threshold.attr,
	&dev_attr_status.attr,
	NULL,
};

static const struct attribute_group hl7603_attr_group = {
	.attrs = hl7603_attrs,
};

/* ------------------------------------------------------------------ */
/* probe / remove / power management                                    */
/* ------------------------------------------------------------------ */

static int hl7603_parse_dt(struct boost_bypass_dev *bq)
{
	struct device_node *np = bq->dev->of_node;

	if (!np)
		return -1;

	of_property_read_u32(np, "vout_threshold", &bq->vout_threshold);
	return 0;
}

static int hl7603_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	msleep(2000); 
	int ret;
	struct boost_bypass_dev *bq;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev    = &client->dev;
	mutex_init(&bq->lock);
	i2c_set_clientdata(client, bq);

	ret = hl7603_parse_dt(bq);
	if (ret) {
		dev_err(bq->dev, "DT parse failed: %d\n", ret);
		return ret;
	}

	ret = hl7603_set_voltage_threshold(bq, bq->vout_threshold);
	if (ret) {
		dev_err(bq->dev, "vout threshold init failed: %d\n", ret);
		return ret;
	}

	/* Start in auto mode; worker will switch to bypass when appropriate */
	mutex_lock(&bq->lock);
	hl7603_set_bypass_mode(bq, false);
	mutex_unlock(&bq->lock);

	/* sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &hl7603_attr_group);
	if (ret)
		dev_warn(bq->dev, "sysfs creation failed: %d\n", ret);

	/* Power-supply change notifier */
	bq->psy_nb.notifier_call = hl7603_psy_notifier_call;
	ret = power_supply_reg_notifier(&bq->psy_nb);
	if (ret)
		dev_warn(bq->dev, "psy notifier register failed: %d\n", ret);

	/* Periodic bypass monitor */
	INIT_DELAYED_WORK(&bq->bypass_check_work, hl7603_bypass_check_work);
	schedule_delayed_work(&bq->bypass_check_work,
			      msecs_to_jiffies(BYPASS_CHECK_INTERVAL_MS));

	dev_info(bq->dev,
		 "hl7603 probed (vout_thr=%u mV, bypass charging supported)\n",
		 bq->vout_threshold);
	return 0;
}

static void hl7603_remove(struct i2c_client *client)
{
	struct boost_bypass_dev *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->bypass_check_work);
	power_supply_unreg_notifier(&bq->psy_nb);
	sysfs_remove_group(&client->dev.kobj, &hl7603_attr_group);

	mutex_lock(&bq->lock);
	hl7603_set_bypass_mode(bq, false);	/* restore auto on unload */
	mutex_unlock(&bq->lock);

	if (bq->batt_psy)
		power_supply_put(bq->batt_psy);
}

static void hl7603_shutdown(struct i2c_client *client)
{
	struct boost_bypass_dev *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->bypass_check_work);
	/* Exit bypass before power-off to ensure safe shutdown */
	hl7603_set_bypass_mode(bq, false);
}

static int hl7603_suspend(struct device *dev)
{
	struct boost_bypass_dev *bq = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&bq->bypass_check_work);
	return 0;
}

static int hl7603_resume(struct device *dev)
{
	struct boost_bypass_dev *bq = dev_get_drvdata(dev);

	schedule_delayed_work(&bq->bypass_check_work, 0);
	return 0;
}

static const struct dev_pm_ops hl7603_pm_ops = {
	.suspend = hl7603_suspend,
	.resume  = hl7603_resume,
};

static const struct of_device_id hl7603_of_match[] = {
	{ .compatible = "hl7603" },
	{},
};
MODULE_DEVICE_TABLE(of, hl7603_of_match);

static struct i2c_driver hl7603_driver = {
	.driver = {
		.name           = "hl7603_boost_bypass",
		.of_match_table = hl7603_of_match,
		.pm             = &hl7603_pm_ops,
	},
	.probe    = hl7603_probe,
	.remove   = hl7603_remove,
	.shutdown = hl7603_shutdown,
};
module_i2c_driver(hl7603_driver);

MODULE_AUTHOR("jinkai <jinkai1@xiaomi.com>");
MODULE_DESCRIPTION("hl7603 boost_bypass driver with bypass charging support");
MODULE_LICENSE("GPL v2");
