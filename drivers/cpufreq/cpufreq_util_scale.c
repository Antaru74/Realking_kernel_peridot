// SPDX-License-Identifier: GPL-2.0-only
/*
 * cpufreq_util_scale.c
 *
 * Vendor hook based util scaler for ALL cpufreq governors including
 * closed-source UAG (cpufreq_uag.ko).
 *
 * Hooks into android_vh_map_util_freq which is called by:
 *   - schedutil (cpufreq_schedutil.c)
 *   - walt     (cpufreq_walt.c)
 *   - UAG      (cpufreq_uag.ko) -- confirmed via strings analysis
 *
 * Usage:
 *   # Enable scaling
 *   echo 85 > /sys/kernel/cpufreq_util_scale/util_scale_pct
 *
 *   # Read current value
 *   cat /sys/kernel/cpufreq_util_scale/util_scale_pct
 *
 *   # Disable (restore to 100%)
 *   echo 100 > /sys/kernel/cpufreq_util_scale/util_scale_pct
 *
 * Effect:
 *   util_scale_pct=85 means governor sees util*0.85
 *   -> lower frequency requested -> CPR selects lower voltage
 *   -> reduced power consumption with minor performance impact
 */

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/atomic.h>
#include <trace/hooks/sched.h>

/* util_scale_pct: 1-100. 100 = no scaling (default) */
static atomic_t util_scale_pct = ATOMIC_INIT(100);

static struct kobject *scale_kobj;

/* ── sysfs ─────────────────────────────────────────────── */
static ssize_t util_scale_pct_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&util_scale_pct));
}

static ssize_t util_scale_pct_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	atomic_set(&util_scale_pct, val);
	return count;
}

static struct kobj_attribute util_scale_attr =
	__ATTR(util_scale_pct, 0644,
	       util_scale_pct_show, util_scale_pct_store);

static struct attribute *scale_attrs[] = {
	&util_scale_attr.attr,
	NULL,
};
static const struct attribute_group scale_attr_grp = {
	.attrs = scale_attrs,
};

/* ── vendor hook ────────────────────────────────────────── */
/*
 * android_vh_map_util_freq is called by all governors that use
 * the Android vendor hook infrastructure (schedutil, walt, UAG).
 *
 * Prototype (from trace/hooks/sched.h):
 *   void android_vh_map_util_freq(unsigned long util,
 *                                  unsigned long freq,
 *                                  unsigned long cap,
 *                                  unsigned long *next_freq,
 *                                  struct cpufreq_policy *policy,
 *                                  bool *need_freq_update);
 *
 * We intercept *next_freq before it is returned to the caller.
 * If next_freq was not set by a previous hook (== 0), we compute
 * the default value ourselves first (same formula as schedutil),
 * then apply the scale.
 */
static void util_scale_hook(void *data,
			    unsigned long util,
			    unsigned long freq,
			    unsigned long cap,
			    unsigned long *next_freq,
			    struct cpufreq_policy *policy,
			    bool *need_freq_update)
{
	unsigned int scale = atomic_read(&util_scale_pct);

	if (scale >= 100)
		return;		/* no-op when at 100% */

	if (!(*next_freq)) {
		/*
		 * No previous hook set next_freq.
		 * Compute the same way schedutil does:
		 *   next_freq = freq * util / cap
		 * (map_util_freq() equivalent, simplified)
		 */
		if (cap == 0)
			return;
		*next_freq = (freq * util) / cap;
	}

	/* Apply scale */
	*next_freq = (*next_freq) * scale / 100;
}

/* ── module init / exit ─────────────────────────────────── */
static int __init util_scale_init(void)
{
	int ret;

	/* /sys/kernel/cpufreq_util_scale/ */
	scale_kobj = kobject_create_and_add("cpufreq_util_scale", kernel_kobj);
	if (!scale_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(scale_kobj, &scale_attr_grp);
	if (ret) {
		kobject_put(scale_kobj);
		return ret;
	}

	/* Register vendor hook */
	ret = register_trace_android_vh_map_util_freq(util_scale_hook, NULL);
	if (ret) {
		pr_err("cpufreq_util_scale: failed to register hook: %d\n", ret);
		sysfs_remove_group(scale_kobj, &scale_attr_grp);
		kobject_put(scale_kobj);
		return ret;
	}

	pr_info("cpufreq_util_scale: loaded. "
		"Write 1-100 to /sys/kernel/cpufreq_util_scale/util_scale_pct\n");
	return 0;
}

static void __exit util_scale_exit(void)
{
	unregister_trace_android_vh_map_util_freq(util_scale_hook, NULL);

	if (scale_kobj) {
		sysfs_remove_group(scale_kobj, &scale_attr_grp);
		kobject_put(scale_kobj);
	}

	pr_info("cpufreq_util_scale: unloaded\n");
}

module_init(util_scale_init);
module_exit(util_scale_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("util_scale vendor hook for all cpufreq governors (incl. UAG)");
MODULE_AUTHOR("custom");
