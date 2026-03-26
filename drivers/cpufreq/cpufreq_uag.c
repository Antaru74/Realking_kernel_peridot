// SPDX-License-Identifier: GPL-2.0
/*
 * cpufreq_uag.c - UAG (Util-Aware Governor) for Qualcomm/POCO devices
 *
 * Reverse-engineered from OnePlus cpufreq_uag.ko, rebuilt on top of
 * the exact same cpufreq-governor framework that schedutil uses.
 *
 * Features:
 *   - target_loads based util→freq mapping (like interactive governor)
 *   - multi_target_loads: per-cluster independent load tables
 *   - hispeed_freq / hispeed_load: boost threshold
 *   - up_rate_limit_us / down_rate_limit_us: asymmetric rate limiting
 *   - soft_limit_freq: soft frequency ceiling with break_freq_margin hysteresis
 *   - stall_aware: AMU counter based stall detection and util reduction
 *   - util_scale_pct: scale util before freq calculation (UV/power effect)
 *
 * UALT — Built-in WALT-equivalent util estimator (zero sched-walt.ko dependency):
 *   - 8ms fixed-window utilization  ≈ WALT prev_runnable_sum
 *   - demand (max of 5-window history) ≈ WALT demand_scaled
 *   - nl (new-load delta on task wakeup) ≈ walt_cpu_load.nl
 *   - pl (bucket-histogram prediction)  ≈ walt_cpu_load.pl
 *   - ed (early detection boost)        ≈ walt_rq.ed_task
 *
 * Compatible: GKI android14-6.1, Snapdragon 8 Gen2/3 (Pineapple/Kalama)
 */

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <linux/kthread.h>
#include <linux/irq_work.h>
#include <linux/sched.h>
#include <linux/sched/cpufreq.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cpumask.h>
#include <linux/topology.h>
#include <linux/atomic.h>
#include <linux/rcupdate.h>
#include <linux/mutex.h>
#include <trace/hooks/sched.h>
#ifdef CONFIG_ARM64_AMU_EXTN
#include <asm/cpufeature.h>
#include <asm/sysreg.h>
#endif

/*
 * sched_cpu_util() is defined in kernel/sched/fair.c (built-in, exported).
 * Not declared in any public header for modules.
 * This is the closest util signal available to loadable modules.
 */
extern unsigned long sched_cpu_util(int cpu);

/*
 * SCHED_FLAG_SUGOV enables SCHED_DEADLINE without real bandwidth admission.
 * Defined in include/linux/sched.h for built-in code; may not be visible
 * to modules on all builds, so guard it.
 */
#ifndef SCHED_FLAG_SUGOV
#define SCHED_FLAG_SUGOV	0x100
#endif

/* Forward declarations */
static struct cpufreq_governor cpufreq_gov_uag;

/* ── Constants ─────────────────────────────────────────── */
#define DEFAULT_TARGET_LOAD      90
#define DEFAULT_HISPEED_LOAD     90
#define DEFAULT_UP_RATE_LIMIT    500    /* µs */
#define DEFAULT_DOWN_RATE_LIMIT  2000   /* µs */
#define DEFAULT_BREAK_FREQ_MARGIN 5     /* % */
#define MAX_TARGET_LOADS         16
#define IOWAIT_BOOST_MIN         (SCHED_CAPACITY_SCALE / 8)

/* AMU counter indices */
#define SYS_AMU_INST_RET         0
#define SYS_AMU_STALL_MEM        1
#define SYS_AMU_CONST_CYC        2
#define SYS_AMU_CORE_CYC         3
#define SYS_AMU_MAX              4

/* report_policy */
#define REPORT_NONE              0
#define REPORT_MIN_UTIL          1
#define REPORT_MAX_UTIL          2
#define REPORT_REDUCE_STALL      3
#define REPORT_NORMAL_REDUCE_STALL 4
#define REPORT_DIRECT            5
#define REPORT_MAX_TYPES         6

/* ── AMU per-cpu data ───────────────────────────────────── */
struct uag_amu_data {
	u64 prev[SYS_AMU_MAX];
	u64 delta[SYS_AMU_MAX];
	u64 last_update_time;
	unsigned long normal_util;
	unsigned long stall_util;
	unsigned long amu_result;
};

static DEFINE_PER_CPU(struct uag_amu_data, amu_cpu_data);

/* ── Tunables ───────────────────────────────────────────── */
struct uag_gov_tunables {
	struct gov_attr_set attr_set;

	/* rate limiting */
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;

	/* target loads: "load0 [freq0 load1 [freq1 ...]]" */
	spinlock_t target_loads_lock;
	unsigned int *target_loads;
	int ntarget_loads;

	/* multi target loads (per cluster sys util) */
	spinlock_t multi_tl_lock;
	unsigned int *multi_target_loads;
	int multi_ntarget_loads;
	bool multi_tl_enable;

	/* hispeed */
	unsigned int hispeed_freq;
	unsigned int hispeed_load;

	/* soft limit */
	bool soft_limit_enable;
	unsigned int soft_limit_freq;
	unsigned int break_freq_margin;  /* % hysteresis above soft_limit */

	/* stall aware (AMU based) */
	bool stall_aware;
	unsigned int reduce_pct_of_stall;       /* 0-100 */
	unsigned int max_stall_reduce_of_util;  /* 0-100 */
	unsigned int report_policy;

	/* cobuck (co-performance buck) */
	bool cobuck_enable;

	/* util_scale_pct: UV/power reduction feature */
	unsigned int util_scale_pct;    /* 1-100, default 100 */
};

/* ── Policy state ───────────────────────────────────────── */
struct uag_gov_policy {
	struct cpufreq_policy *policy;
	struct uag_gov_tunables *tunables;
	struct list_head tunables_hook;

	raw_spinlock_t update_lock;
	u64 last_freq_update_time;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;
	s64 min_rate_delay_ns;
	unsigned int next_freq;
	unsigned int cached_raw_freq;

	/* kthread for slow-path (deferred) updates */
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool limits_changed;
	bool need_freq_update;
};

/* ── Per-cpu state ──────────────────────────────────────── */
struct uag_gov_cpu {
	struct update_util_data update_util;
	struct uag_gov_policy *sg_policy;
	unsigned int cpu;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;

	unsigned long util;
	unsigned long bw_dl;
	unsigned long max;

#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct uag_gov_cpu, uag_gov_cpu);

/*
 * Global tunables & lock — identical to schedutil's pattern.
 * Required for correct lifecycle when have_governor_per_policy() is false.
 */
static struct uag_gov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

/* proc entry root */
static struct proc_dir_entry *uag_proc_root;

/* ── Default target loads (static, never freed) ────────── */
static unsigned int default_target_loads[] = { DEFAULT_TARGET_LOAD };
static unsigned int default_target_loads_sys[] = { DEFAULT_TARGET_LOAD };


/* ═══════════════════════════════════════════════════════════════════════
 * UALT — UAG Built-in WALT-equivalent Util Estimator
 *
 * Provides WALT-like utilization quality using sched_cpu_util() (PELT)
 * as the underlying signal. Zero dependency on sched-walt.ko.
 * ═══════════════════════════════════════════════════════════════════════ */

#define UALT_WINDOW_NS         8000000ULL  /* 8 ms  — identical to WALT */
#define UALT_HIST_SIZE         5           /* windows in demand history  */
#define UALT_NUM_BUCKETS       16          /* prediction histogram bins  */
#define UALT_ED_THRESH_PCT     80          /* ED: util% to arm trigger   */
#define UALT_ED_SUSTAIN_NS     1000000ULL  /* ED: must hold 1 ms         */
#define UALT_NL_JUMP_PCT       15          /* NL: jump% that means wakeup*/
#define UALT_PRED_DECAY_SH     3           /* bucket decay: *7/8 per win */

struct ualt_cpu {
	u64           window_start;
	unsigned long curr_max;
	unsigned long prev_util;

	unsigned long hist[UALT_HIST_SIZE];
	int           hist_idx;
	unsigned long demand;

	u32           buckets[UALT_NUM_BUCKETS];
	unsigned long pl;

	unsigned long prev_sample;
	unsigned long nl;

	u64           ed_arm_ns;
	bool          ed_armed;
	bool          ed_active;
	unsigned long ed_util;

	raw_spinlock_t lock;
};

static DEFINE_PER_CPU(struct ualt_cpu, ualt_cpu_data);

static void ualt_init_cpu(int cpu)
{
	struct ualt_cpu *uc = &per_cpu(ualt_cpu_data, cpu);

	memset(uc, 0, sizeof(*uc));
	raw_spin_lock_init(&uc->lock);
	uc->window_start = ktime_get_ns();
}

static unsigned long ualt_update(int cpu, unsigned long raw,
				  unsigned long cap, u64 now)
{
	struct ualt_cpu *uc = &per_cpu(ualt_cpu_data, cpu);
	unsigned long flags, out;
	u64 elapsed;
	int i, bucket;

	raw_spin_lock_irqsave(&uc->lock, flags);

	elapsed = now - uc->window_start;

	if (elapsed >= UALT_WINDOW_NS) {
		/* Commit current window to history */
		uc->hist[uc->hist_idx] = uc->curr_max;
		uc->hist_idx = (uc->hist_idx + 1) % UALT_HIST_SIZE;

		/* demand = max across history */
		uc->demand = 0;
		for (i = 0; i < UALT_HIST_SIZE; i++)
			if (uc->hist[i] > uc->demand)
				uc->demand = uc->hist[i];

		/* Predicted load bucket update */
		bucket = (int)((raw * UALT_NUM_BUCKETS) / (cap + 1));
		if (bucket >= UALT_NUM_BUCKETS)
			bucket = UALT_NUM_BUCKETS - 1;

		for (i = 0; i < UALT_NUM_BUCKETS; i++)
			uc->buckets[i] -= uc->buckets[i] >> UALT_PRED_DECAY_SH;
		uc->buckets[bucket] += 1 << UALT_PRED_DECAY_SH;

		{
			u32 total = 0, weighted = 0;
			for (i = 0; i < UALT_NUM_BUCKETS; i++) {
				total    += uc->buckets[i];
				weighted += uc->buckets[i] * i;
			}
			uc->pl = total ? (weighted * cap) /
					  (total * UALT_NUM_BUCKETS) : 0;
		}

		uc->prev_util    = uc->curr_max;
		uc->curr_max     = raw;
		uc->window_start = now;
		uc->nl           = 0;
		uc->ed_active    = false;
		uc->ed_armed     = false;
	} else {
		if (raw > uc->curr_max)
			uc->curr_max = raw;

		/* New load detection */
		if (raw > uc->prev_sample) {
			unsigned long jump = raw - uc->prev_sample;
			if (jump > cap * UALT_NL_JUMP_PCT / 100)
				uc->nl = jump;
			else if (uc->nl && raw <= uc->prev_sample)
				uc->nl = 0;
		} else {
			uc->nl = 0;
		}

		/* Early detection */
		{
			unsigned long ed_thresh = cap * UALT_ED_THRESH_PCT / 100;
			if (raw >= ed_thresh) {
				if (!uc->ed_armed) {
					uc->ed_armed  = true;
					uc->ed_arm_ns = now;
				} else if (!uc->ed_active &&
					   (now - uc->ed_arm_ns) >=
						UALT_ED_SUSTAIN_NS) {
					uc->ed_active = true;
					uc->ed_util   = max(raw, ed_thresh);
				}
			} else {
				uc->ed_armed  = false;
				uc->ed_active = false;
			}
		}
	}

	uc->prev_sample = raw;

	/* Compose final util */
	out = max(uc->prev_util, uc->demand);
	out = max(out, uc->pl);
	out = max(out, raw);

	if (uc->nl)
		out = min(out + uc->nl, cap);

	if (uc->ed_active)
		out = max(out, uc->ed_util);

	raw_spin_unlock_irqrestore(&uc->lock, flags);

	return min(out, cap);
}


/* ═══════════════════════════════════════════════════════════════════════
 * AMU-based stall detection
 * ═══════════════════════════════════════════════════════════════════════ */

static bool uag_amu_supported(void)
{
#ifdef CONFIG_ARM64_AMU_EXTN
	return cpus_have_const_cap(ARM64_HAS_AMU_EXTN);
#else
	return false;
#endif
}

static u64 uag_read_amu(int idx)
{
#ifdef CONFIG_ARM64_AMU_EXTN
	switch (idx) {
	case SYS_AMU_INST_RET:  return read_sysreg_s(SYS_AMEVCNTR0_EL0(0));
	case SYS_AMU_STALL_MEM: return read_sysreg_s(SYS_AMEVCNTR0_EL0(1));
	case SYS_AMU_CONST_CYC: return read_sysreg_s(SYS_AMEVCNTR0_EL0(2));
	case SYS_AMU_CORE_CYC:  return read_sysreg_s(SYS_AMEVCNTR0_EL0(3));
	}
#endif
	return 0;
}

static void uag_update_amu_counter(int cpu)
{
	struct uag_amu_data *ad = &per_cpu(amu_cpu_data, cpu);
	u64 now = ktime_get_ns();
	u64 delta_time;
	int i;

	if (!uag_amu_supported())
		return;

	delta_time = now - ad->last_update_time;
	if (delta_time < 1000000ULL) /* < 1ms, skip */
		return;

	for (i = 0; i < SYS_AMU_MAX; i++) {
		u64 cur = uag_read_amu(i);
		ad->delta[i] = cur - ad->prev[i];
		ad->prev[i] = cur;
	}
	ad->last_update_time = now;
}

static unsigned long uag_amu_adjust_util(struct uag_gov_cpu *sg_cpu,
					  unsigned long util)
{
	struct uag_gov_policy *sg_policy = sg_cpu->sg_policy;
	struct uag_gov_tunables *tunables = sg_policy->tunables;
	struct uag_amu_data *ad;
	u64 stall, cyc;
	unsigned long stall_pct, reduce, max_reduce;

	if (!tunables->stall_aware || !uag_amu_supported())
		return util;

	ad = &per_cpu(amu_cpu_data, sg_cpu->cpu);
	stall = ad->delta[SYS_AMU_STALL_MEM];
	cyc   = ad->delta[SYS_AMU_CORE_CYC];

	if (!cyc)
		return util;

	stall_pct = (unsigned long)div64_u64(stall * 100, cyc);
	if (stall_pct > 100)
		stall_pct = 100;

	reduce = util * stall_pct * tunables->reduce_pct_of_stall / 10000;

	max_reduce = util * tunables->max_stall_reduce_of_util / 100;
	if (reduce > max_reduce)
		reduce = max_reduce;

	ad->amu_result  = util - reduce;
	ad->stall_util  = reduce;
	ad->normal_util = util - reduce;

	return util - reduce;
}


/* ═══════════════════════════════════════════════════════════════════════
 * Target loads lookup
 * Format: load0 [freq0 load1 [freq1 ...]]
 * ═══════════════════════════════════════════════════════════════════════ */

static unsigned int freq_to_targetload(struct uag_gov_tunables *tunables,
				       unsigned int freq)
{
	unsigned int ret;
	unsigned int i;
	unsigned long flags;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	ret = tunables->target_loads[0];
	for (i = 1; i < tunables->ntarget_loads - 1; i += 2) {
		if (freq < tunables->target_loads[i])
			break;
		ret = tunables->target_loads[i + 1];
	}
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return ret;
}

static unsigned int choose_freq_from_tl(struct uag_gov_policy *sg_policy,
					 unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct uag_gov_tunables *tunables = sg_policy->tunables;
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	unsigned int freq, tl;
	int i;

	freq = 0;

	/* Walk freq table: find lowest freq where util/max <= tl/100 */
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		unsigned int f = freq_table[i].frequency;
		if (f == CPUFREQ_ENTRY_INVALID)
			continue;
		tl = freq_to_targetload(tunables, f);
		if (util * 100 <= (unsigned long)tl * max) {
			freq = f;
			break;
		}
	}
	if (!freq) {
		/* util exceeds all target_loads; use max freq */
		for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
			if (freq_table[i].frequency != CPUFREQ_ENTRY_INVALID)
				freq = freq_table[i].frequency;
		}
	}
	return freq;
}


/* ═══════════════════════════════════════════════════════════════════════
 * IO-wait boost — ported directly from schedutil
 * ═══════════════════════════════════════════════════════════════════════ */

static bool uag_iowait_reset(struct uag_gov_cpu *sg_cpu, u64 time,
			      bool set_iowait_boost)
{
	s64 delta_ns = time - sg_cpu->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	sg_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	sg_cpu->iowait_boost_pending = set_iowait_boost;
	return true;
}

static void uag_iowait_boost(struct uag_gov_cpu *sg_cpu, u64 time,
			      unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	if (sg_cpu->iowait_boost &&
	    uag_iowait_reset(sg_cpu, time, set_iowait_boost))
		return;

	if (!set_iowait_boost)
		return;

	if (sg_cpu->iowait_boost_pending)
		return;
	sg_cpu->iowait_boost_pending = true;

	if (sg_cpu->iowait_boost) {
		sg_cpu->iowait_boost =
			min_t(unsigned int, sg_cpu->iowait_boost << 1,
			      SCHED_CAPACITY_SCALE);
		return;
	}

	sg_cpu->iowait_boost = IOWAIT_BOOST_MIN;
}

static void uag_iowait_apply(struct uag_gov_cpu *sg_cpu, u64 time)
{
	unsigned long boost;

	if (!sg_cpu->iowait_boost)
		return;

	if (uag_iowait_reset(sg_cpu, time, false))
		return;

	if (!sg_cpu->iowait_boost_pending) {
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
			sg_cpu->iowait_boost = 0;
			return;
		}
	}

	sg_cpu->iowait_boost_pending = false;

	boost = (sg_cpu->iowait_boost * sg_cpu->max) >> SCHED_CAPACITY_SHIFT;
	if (sg_cpu->util < boost)
		sg_cpu->util = boost;
}


/* ═══════════════════════════════════════════════════════════════════════
 * Util collection
 * ═══════════════════════════════════════════════════════════════════════ */

static void uag_gov_get_util(struct uag_gov_cpu *sg_cpu)
{
	unsigned long raw, util;
	u64 now = ktime_get_ns();

	sg_cpu->max   = arch_scale_cpu_capacity(sg_cpu->cpu);
	sg_cpu->bw_dl = 0;  /* cpu_bw_dl(rq) not exported to modules */

	/* Raw PELT util — base signal */
	raw  = sched_cpu_util(sg_cpu->cpu);

	/* UALT enhancement */
	util = ualt_update(sg_cpu->cpu, raw, sg_cpu->max, now);

	sg_cpu->util = util;
}

#ifdef CONFIG_NO_HZ_COMMON
static bool uag_cpu_is_busy(struct uag_gov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool uag_cpu_is_busy(struct uag_gov_cpu *sg_cpu) { return false; }
#endif


/* ═══════════════════════════════════════════════════════════════════════
 * Soft limit with hysteresis
 * ═══════════════════════════════════════════════════════════════════════ */

static unsigned int uag_apply_soft_limit(struct uag_gov_policy *sg_policy,
					  unsigned int freq,
					  unsigned long util,
					  unsigned long max)
{
	struct uag_gov_tunables *tunables = sg_policy->tunables;
	unsigned int soft_freq, break_freq;

	if (!tunables->soft_limit_enable || !tunables->soft_limit_freq)
		return freq;

	soft_freq  = tunables->soft_limit_freq;
	break_freq = soft_freq + soft_freq * tunables->break_freq_margin / 100;

	if (freq <= soft_freq)
		return freq;

	if (freq <= break_freq) {
		unsigned int tl = freq_to_targetload(tunables, soft_freq);
		if (util * 100 <= (unsigned long)tl * max)
			return soft_freq;
		return freq;
	}

	return break_freq;
}


/* ═══════════════════════════════════════════════════════════════════════
 * Rate limiting — UAG style (separate up/down)
 * ═══════════════════════════════════════════════════════════════════════ */

static void update_min_rate_limit_ns(struct uag_gov_policy *sg_policy)
{
	sg_policy->min_rate_delay_ns =
		min(sg_policy->up_rate_delay_ns, sg_policy->down_rate_delay_ns);
}

/*
 * Should we update at all?  Mirrors schedutil's check but keeps UAG's
 * min_rate_delay_ns as a pre-filter (cheaper than computing freq).
 */
static bool uag_should_update_freq(struct uag_gov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	/*
	 * Critical: reject cross-CPU updates the hardware can't handle.
	 * Missing this check was a deadlock source.
	 */
	if (!cpufreq_this_cpu_can_update(sg_policy->policy))
		return false;

	if (sg_policy->work_in_progress)
		return false;

	if (unlikely(sg_policy->limits_changed)) {
		sg_policy->limits_changed = false;
		sg_policy->need_freq_update = true;
		return true;
	}

	delta_ns = (s64)(time - sg_policy->last_freq_update_time);
	return delta_ns >= sg_policy->min_rate_delay_ns;
}

/*
 * Directional rate limit: separate delays for up vs down.
 */
static bool uag_up_down_rate_limit(struct uag_gov_policy *sg_policy,
				    u64 time, unsigned int next_freq)
{
	s64 delta_ns;

	delta_ns = (s64)(time - sg_policy->last_freq_update_time);

	if (next_freq > sg_policy->next_freq &&
	    delta_ns < sg_policy->up_rate_delay_ns)
		return true;

	if (next_freq < sg_policy->next_freq &&
	    delta_ns < sg_policy->down_rate_delay_ns)
		return true;

	return false;
}

static bool uag_update_next_freq(struct uag_gov_policy *sg_policy,
				  u64 time, unsigned int next_freq)
{
	if (sg_policy->need_freq_update)
		sg_policy->need_freq_update = false;
	else if (next_freq == sg_policy->next_freq)
		return false;

	if (uag_up_down_rate_limit(sg_policy, time, next_freq))
		return false;

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;
	return true;
}


/* ═══════════════════════════════════════════════════════════════════════
 * Main frequency calculation
 * ═══════════════════════════════════════════════════════════════════════ */

static unsigned int uag_get_final_freq(struct uag_gov_policy *sg_policy,
					unsigned long util, unsigned long max)
{
	struct uag_gov_tunables *tunables = sg_policy->tunables;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq;
	unsigned long scaled_util = util;

	/* util_scale_pct: UV/power reduction */
	if (tunables->util_scale_pct > 0 && tunables->util_scale_pct < 100)
		scaled_util = util * tunables->util_scale_pct / 100;

	freq = choose_freq_from_tl(sg_policy, scaled_util, max);

	/* hispeed boost */
	if (tunables->hispeed_freq) {
		if (util * 100 >= (unsigned long)tunables->hispeed_load * max) {
			if (freq < tunables->hispeed_freq)
				freq = tunables->hispeed_freq;
		}
	}

	freq = uag_apply_soft_limit(sg_policy, freq, scaled_util, max);

	return clamp(freq, policy->min, policy->max);
}


/* ═══════════════════════════════════════════════════════════════════════
 * Deferred update — kthread path (identical to schedutil)
 * ═══════════════════════════════════════════════════════════════════════ */

static void uag_deferred_update(struct uag_gov_policy *sg_policy)
{
	if (!sg_policy->work_in_progress) {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

static void uag_irq_work(struct irq_work *irq_work)
{
	struct uag_gov_policy *sg_policy =
		container_of(irq_work, struct uag_gov_policy, irq_work);
	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

static void uag_work(struct kthread_work *work)
{
	struct uag_gov_policy *sg_policy =
		container_of(work, struct uag_gov_policy, work);
	unsigned int freq;
	unsigned long flags;

	/*
	 * Hold update_lock briefly: read next_freq and clear work_in_progress
	 * atomically to avoid missed updates.  Identical to sugov_work().
	 */
	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	freq = sg_policy->next_freq;
	sg_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);
}


/* ═══════════════════════════════════════════════════════════════════════
 * update_util callback — single CPU policy
 *
 * Follows the schedutil sugov_update_single_freq() lock discipline:
 * - Main logic runs under rq->lock (implicit serialisation for this CPU)
 * - update_lock is only acquired for the slow-path deferred_update
 * - fast_switch path needs no extra lock
 * ═══════════════════════════════════════════════════════════════════════ */

static void uag_update_single(struct update_util_data *hook,
			       u64 time, unsigned int flags)
{
	struct uag_gov_cpu *sg_cpu =
		container_of(hook, struct uag_gov_cpu, update_util);
	struct uag_gov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int next_f;

	uag_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (!uag_should_update_freq(sg_policy, time))
		return;

	uag_gov_get_util(sg_cpu);
	uag_iowait_apply(sg_cpu, time);

	/* AMU stall adjustment */
	uag_update_amu_counter(sg_cpu->cpu);
	sg_cpu->util = uag_amu_adjust_util(sg_cpu, sg_cpu->util);

	next_f = uag_get_final_freq(sg_policy, sg_cpu->util, sg_cpu->max);

	/*
	 * Do not reduce freq if the CPU hasn't been idle recently
	 * (prevents premature freq drops). Identical to schedutil.
	 */
	if (uag_cpu_is_busy(sg_cpu) && next_f < sg_policy->next_freq &&
	    sg_policy->next_freq != UINT_MAX &&
	    !sg_policy->need_freq_update)
		next_f = sg_policy->next_freq;

	if (!uag_update_next_freq(sg_policy, time, next_f))
		return;

	/*
	 * Lock discipline matches schedutil exactly:
	 * fast_switch → no lock needed (runs under rq->lock)
	 * slow path   → acquire update_lock for deferred_update
	 */
	if (sg_policy->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(sg_policy->policy, next_f);
	} else {
		raw_spin_lock(&sg_policy->update_lock);
		uag_deferred_update(sg_policy);
		raw_spin_unlock(&sg_policy->update_lock);
	}
}


/* ═══════════════════════════════════════════════════════════════════════
 * update_util callback — shared policy (multi-CPU per policy)
 *
 * update_lock held for the entire duration (same as sugov_update_shared).
 * ═══════════════════════════════════════════════════════════════════════ */

static unsigned int uag_next_freq_shared(struct uag_gov_cpu *sg_cpu, u64 time)
{
	struct uag_gov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct uag_gov_cpu *j_sg_cpu = &per_cpu(uag_gov_cpu, j);
		unsigned long j_util, j_max;

		uag_gov_get_util(j_sg_cpu);
		uag_iowait_apply(j_sg_cpu, time);

		/* AMU stall adjustment */
		uag_update_amu_counter(j);
		j_util = uag_amu_adjust_util(j_sg_cpu, j_sg_cpu->util);
		j_max  = j_sg_cpu->max;

		if (j_util * max > j_max * util) {
			util = j_util;
			max  = j_max;
		}
	}

	return uag_get_final_freq(sg_policy, util, max);
}

static void uag_update_shared(struct update_util_data *hook,
			       u64 time, unsigned int flags)
{
	struct uag_gov_cpu *sg_cpu =
		container_of(hook, struct uag_gov_cpu, update_util);
	struct uag_gov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int next_f;

	raw_spin_lock(&sg_policy->update_lock);

	uag_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (uag_should_update_freq(sg_policy, time)) {
		next_f = uag_next_freq_shared(sg_cpu, time);

		if (!uag_update_next_freq(sg_policy, time, next_f))
			goto unlock;

		if (sg_policy->policy->fast_switch_enabled)
			cpufreq_driver_fast_switch(sg_policy->policy, next_f);
		else
			uag_deferred_update(sg_policy);
	}
unlock:
	raw_spin_unlock(&sg_policy->update_lock);
}


/* ═══════════════════════════════════════════════════════════════════════
 * sysfs tunables
 * ═══════════════════════════════════════════════════════════════════════ */

static inline struct uag_gov_tunables *to_uag_tunables(struct gov_attr_set *s)
{
	return container_of(s, struct uag_gov_tunables, attr_set);
}

/* ── target_loads ── */
static ssize_t target_loads_show(struct gov_attr_set *attr_set, char *buf)
{
	struct uag_gov_tunables *t = to_uag_tunables(attr_set);
	unsigned long flags;
	ssize_t ret = 0;
	int i;

	spin_lock_irqsave(&t->target_loads_lock, flags);
	for (i = 0; i < t->ntarget_loads; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s",
				 t->target_loads[i],
				 i < t->ntarget_loads - 1 ? " " : "\n");
	spin_unlock_irqrestore(&t->target_loads_lock, flags);
	return ret;
}

static ssize_t target_loads_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct uag_gov_tunables *t = to_uag_tunables(attr_set);
	unsigned int *new_loads;
	int ntokens = 0;
	char *p, *tmp;
	unsigned long flags;

	tmp = kstrdup(buf, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	new_loads = kcalloc(MAX_TARGET_LOADS, sizeof(*new_loads), GFP_KERNEL);
	if (!new_loads) {
		kfree(tmp);
		return -ENOMEM;
	}

	p = tmp;
	while (p && ntokens < MAX_TARGET_LOADS) {
		unsigned int val;
		if (!kstrtouint(strsep(&p, " "), 10, &val))
			new_loads[ntokens++] = val;
	}
	kfree(tmp);

	if (!ntokens) {
		kfree(new_loads);
		return -EINVAL;
	}

	spin_lock_irqsave(&t->target_loads_lock, flags);
	if (t->target_loads != default_target_loads)
		kfree(t->target_loads);
	t->target_loads  = new_loads;
	t->ntarget_loads = ntokens;
	spin_unlock_irqrestore(&t->target_loads_lock, flags);

	return count;
}
static struct governor_attr target_loads_attr = __ATTR_RW(target_loads);

/* ── up_rate_limit_us ── */
static ssize_t up_rate_limit_us_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->up_rate_limit_us);
}
static ssize_t up_rate_limit_us_store(struct gov_attr_set *s,
				       const char *buf, size_t count)
{
	struct uag_gov_tunables *t = to_uag_tunables(s);
	struct uag_gov_policy *sg_policy;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->up_rate_limit_us = val;
	list_for_each_entry(sg_policy, &s->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = val * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}
	return count;
}
static struct governor_attr up_rate_limit_us_attr = __ATTR_RW(up_rate_limit_us);

/* ── down_rate_limit_us ── */
static ssize_t down_rate_limit_us_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->down_rate_limit_us);
}
static ssize_t down_rate_limit_us_store(struct gov_attr_set *s,
					 const char *buf, size_t count)
{
	struct uag_gov_tunables *t = to_uag_tunables(s);
	struct uag_gov_policy *sg_policy;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->down_rate_limit_us = val;
	list_for_each_entry(sg_policy, &s->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = val * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}
	return count;
}
static struct governor_attr down_rate_limit_us_attr = __ATTR_RW(down_rate_limit_us);

/* ── hispeed_freq ── */
static ssize_t hispeed_freq_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->hispeed_freq);
}
static ssize_t hispeed_freq_store(struct gov_attr_set *s,
				   const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	to_uag_tunables(s)->hispeed_freq = val;
	return count;
}
static struct governor_attr hispeed_freq_attr = __ATTR_RW(hispeed_freq);

/* ── hispeed_load ── */
static ssize_t hispeed_load_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->hispeed_load);
}
static ssize_t hispeed_load_store(struct gov_attr_set *s,
				   const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	to_uag_tunables(s)->hispeed_load = val;
	return count;
}
static struct governor_attr hispeed_load_attr = __ATTR_RW(hispeed_load);

/* ── soft_limit_enable ── */
static ssize_t soft_limit_enable_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->soft_limit_enable);
}
static ssize_t soft_limit_enable_store(struct gov_attr_set *s,
					const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	to_uag_tunables(s)->soft_limit_enable = !!val;
	return count;
}
static struct governor_attr soft_limit_enable_attr = __ATTR_RW(soft_limit_enable);

/* ── soft_limit_freq ── */
static ssize_t soft_limit_freq_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->soft_limit_freq);
}
static ssize_t soft_limit_freq_store(struct gov_attr_set *s,
				      const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	to_uag_tunables(s)->soft_limit_freq = val;
	return count;
}
static struct governor_attr soft_limit_freq_attr = __ATTR_RW(soft_limit_freq);

/* ── break_freq_margin ── */
static ssize_t break_freq_margin_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->break_freq_margin);
}
static ssize_t break_freq_margin_store(struct gov_attr_set *s,
					const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	to_uag_tunables(s)->break_freq_margin = val;
	return count;
}
static struct governor_attr break_freq_margin_attr = __ATTR_RW(break_freq_margin);

/* ── stall_aware ── */
static ssize_t stall_aware_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->stall_aware);
}
static ssize_t stall_aware_store(struct gov_attr_set *s,
				  const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	to_uag_tunables(s)->stall_aware = !!val;
	return count;
}
static struct governor_attr stall_aware_attr = __ATTR_RW(stall_aware);

/* ── reduce_pct_of_stall ── */
static ssize_t reduce_pct_of_stall_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->reduce_pct_of_stall);
}
static ssize_t reduce_pct_of_stall_store(struct gov_attr_set *s,
					  const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	to_uag_tunables(s)->reduce_pct_of_stall = val;
	return count;
}
static struct governor_attr reduce_pct_of_stall_attr = __ATTR_RW(reduce_pct_of_stall);

/* ── max_stall_reduce_of_util ── */
static ssize_t max_stall_reduce_of_util_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->max_stall_reduce_of_util);
}
static ssize_t max_stall_reduce_of_util_store(struct gov_attr_set *s,
					       const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	to_uag_tunables(s)->max_stall_reduce_of_util = val;
	return count;
}
static struct governor_attr max_stall_reduce_of_util_attr =
	__ATTR_RW(max_stall_reduce_of_util);

/* ── report_policy ── */
static ssize_t report_policy_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->report_policy);
}
static ssize_t report_policy_store(struct gov_attr_set *s,
				    const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val) || val >= REPORT_MAX_TYPES)
		return -EINVAL;
	to_uag_tunables(s)->report_policy = val;
	return count;
}
static struct governor_attr report_policy_attr = __ATTR_RW(report_policy);

/* ── multi_tl_enable ── */
static ssize_t multi_tl_enable_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->multi_tl_enable);
}
static ssize_t multi_tl_enable_store(struct gov_attr_set *s,
				      const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	to_uag_tunables(s)->multi_tl_enable = !!val;
	return count;
}
static struct governor_attr multi_tl_enable_attr = __ATTR_RW(multi_tl_enable);

/* ── cobuck_enable ── */
static ssize_t cobuck_enable_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->cobuck_enable);
}
static ssize_t cobuck_enable_store(struct gov_attr_set *s,
				    const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	to_uag_tunables(s)->cobuck_enable = !!val;
	return count;
}
static struct governor_attr cobuck_enable_attr = __ATTR_RW(cobuck_enable);

/* ── util_scale_pct ── */
static ssize_t util_scale_pct_show(struct gov_attr_set *s, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_uag_tunables(s)->util_scale_pct);
}
static ssize_t util_scale_pct_store(struct gov_attr_set *s,
				     const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	to_uag_tunables(s)->util_scale_pct = val;
	return count;
}
static struct governor_attr util_scale_pct_attr = __ATTR_RW(util_scale_pct);

static struct attribute *uag_gov_attrs[] = {
	&target_loads_attr.attr,
	&up_rate_limit_us_attr.attr,
	&down_rate_limit_us_attr.attr,
	&hispeed_freq_attr.attr,
	&hispeed_load_attr.attr,
	&soft_limit_enable_attr.attr,
	&soft_limit_freq_attr.attr,
	&break_freq_margin_attr.attr,
	&stall_aware_attr.attr,
	&reduce_pct_of_stall_attr.attr,
	&max_stall_reduce_of_util_attr.attr,
	&report_policy_attr.attr,
	&multi_tl_enable_attr.attr,
	&cobuck_enable_attr.attr,
	&util_scale_pct_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(uag_gov);

/*
 * Tunables kobject type.
 * Uses governor_sysfs_ops (NOT kobj_sysfs_ops!) — this dispatches through
 * gov_attr_set → governor_attr show/store, which matches our function
 * signatures.  kobj_sysfs_ops expects kobj_attribute signatures (3 args)
 * and using it here was a crash cause.
 */
static void uag_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);
	struct uag_gov_tunables *tunables = to_uag_tunables(attr_set);

	if (tunables->target_loads != default_target_loads)
		kfree(tunables->target_loads);
	if (tunables->multi_target_loads != default_target_loads_sys)
		kfree(tunables->multi_target_loads);
	kfree(tunables);
}

static struct kobj_type uag_tunables_ktype = {
	.default_groups	= uag_gov_groups,
	.sysfs_ops	= &governor_sysfs_ops,   /* CRITICAL: not kobj_sysfs_ops */
	.release	= &uag_tunables_free,
};


/* ═══════════════════════════════════════════════════════════════════════
 * Policy / tunables alloc / free — schedutil-identical lifecycle
 * ═══════════════════════════════════════════════════════════════════════ */

static struct uag_gov_policy *uag_policy_alloc(struct cpufreq_policy *policy)
{
	struct uag_gov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void uag_policy_free(struct uag_gov_policy *sg_policy)
{
	kfree(sg_policy);
}

/*
 * kthread create — follows schedutil exactly:
 *   SCHED_DEADLINE with SCHED_FLAG_SUGOV, bound to policy CPUs.
 *   Only created when fast_switch is NOT available (slow path).
 */
static int uag_kthread_create(struct uag_gov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAG_SUGOV,
		.sched_nice	= 0,
		.sched_priority	= 0,
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&sg_policy->work, uag_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"uag:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("cpufreq_uag: failed to create kthread: %ld\n",
		       PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("cpufreq_uag: failed to set SCHED_DEADLINE, "
			"falling back to FIFO\n");
		/*
		 * Fallback: re-create with FIFO if DEADLINE fails.
		 * Some GKI builds don't allow SCHED_FLAG_SUGOV from modules.
		 */
		thread = kthread_create(kthread_worker_fn,
					&sg_policy->worker,
					"uag:%d",
					cpumask_first(policy->related_cpus));
		if (IS_ERR(thread))
			return PTR_ERR(thread);
		sched_set_fifo_low(thread);
	}

	sg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, uag_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);
	return 0;
}

static void uag_kthread_stop(struct uag_gov_policy *sg_policy)
{
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct uag_gov_tunables *uag_tunables_alloc(struct uag_gov_policy *sg_policy)
{
	struct uag_gov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (!tunables)
		return NULL;

	/*
	 * CRITICAL: Initialise attr_set including policy_list.
	 * Without this, list_add() in governor init dereferences garbage
	 * pointers and crashes.
	 */
	gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);

	if (!have_governor_per_policy())
		global_tunables = tunables;

	spin_lock_init(&tunables->target_loads_lock);
	spin_lock_init(&tunables->multi_tl_lock);

	tunables->target_loads       = default_target_loads;
	tunables->ntarget_loads      = ARRAY_SIZE(default_target_loads);
	tunables->multi_target_loads = default_target_loads_sys;
	tunables->multi_ntarget_loads = ARRAY_SIZE(default_target_loads_sys);

	tunables->up_rate_limit_us   = DEFAULT_UP_RATE_LIMIT;
	tunables->down_rate_limit_us = DEFAULT_DOWN_RATE_LIMIT;
	tunables->hispeed_load       = DEFAULT_HISPEED_LOAD;
	tunables->hispeed_freq       = 0;
	tunables->soft_limit_enable  = false;
	tunables->soft_limit_freq    = 0;
	tunables->break_freq_margin  = DEFAULT_BREAK_FREQ_MARGIN;
	tunables->stall_aware        = false;
	tunables->reduce_pct_of_stall = 50;
	tunables->max_stall_reduce_of_util = 30;
	tunables->report_policy      = REPORT_NONE;
	tunables->multi_tl_enable    = false;
	tunables->cobuck_enable      = false;
	tunables->util_scale_pct     = 100;

	return tunables;
}

static void uag_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
}


/* ═══════════════════════════════════════════════════════════════════════
 * Governor callbacks — init / exit / start / stop / limits
 * Lifecycle management matches schedutil exactly.
 * ═══════════════════════════════════════════════════════════════════════ */

static int uag_gov_init(struct cpufreq_policy *policy)
{
	struct uag_gov_policy *sg_policy;
	struct uag_gov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = uag_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = uag_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set,
				 &sg_policy->tunables_hook);
		goto out;
	}

	tunables = uag_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &uag_tunables_ktype,
				   get_governor_parent_kobj(policy),
				   "%s", cpufreq_gov_uag.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	uag_clear_global_tunables();

stop_kthread:
	uag_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	uag_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("cpufreq_uag: initialization failed (error %d)\n", ret);
	return ret;
}

static void uag_gov_exit(struct cpufreq_policy *policy)
{
	struct uag_gov_policy *sg_policy = policy->governor_data;
	struct uag_gov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	/*
	 * gov_attr_set_put() decrements the refcount and, when it drops
	 * to zero, calls kobject_put() which triggers uag_tunables_free().
	 * This is the correct way to free tunables — never kfree() them
	 * directly, as the kobject may still be referenced.
	 */
	count = gov_attr_set_put(&tunables->attr_set,
				 &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		uag_clear_global_tunables();

	mutex_unlock(&global_tunables_lock);

	uag_kthread_stop(sg_policy);
	uag_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int uag_gov_start(struct cpufreq_policy *policy)
{
	struct uag_gov_policy *sg_policy = policy->governor_data;
	struct uag_gov_tunables *tunables = sg_policy->tunables;
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned int cpu;

	sg_policy->up_rate_delay_ns =
		tunables->up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns =
		tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_ns(sg_policy);

	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq             = 0;
	sg_policy->work_in_progress      = false;
	sg_policy->limits_changed        = false;
	sg_policy->cached_raw_freq       = 0;
	sg_policy->need_freq_update =
		cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS);

	for_each_cpu(cpu, policy->cpus) {
		struct uag_gov_cpu *sg_cpu = &per_cpu(uag_gov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu        = cpu;
		sg_cpu->sg_policy  = sg_policy;

		/* Initialise UALT estimator */
		ualt_init_cpu(cpu);
	}

	if (policy_is_shared(policy))
		uu = uag_update_shared;
	else
		uu = uag_update_single;

	for_each_cpu(cpu, policy->cpus) {
		struct uag_gov_cpu *sg_cpu = &per_cpu(uag_gov_cpu, cpu);
		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util, uu);
	}

	return 0;
}

static void uag_gov_stop(struct cpufreq_policy *policy)
{
	struct uag_gov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void uag_gov_limits(struct cpufreq_policy *policy)
{
	struct uag_gov_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	sg_policy->limits_changed = true;
}

static struct cpufreq_governor cpufreq_gov_uag = {
	.name           = "uag",
	.owner          = THIS_MODULE,
	.flags          = CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init           = uag_gov_init,
	.exit           = uag_gov_exit,
	.start          = uag_gov_start,
	.stop           = uag_gov_stop,
	.limits         = uag_gov_limits,
};


/* ═══════════════════════════════════════════════════════════════════════
 * Exported symbols (for other modules)
 * ═══════════════════════════════════════════════════════════════════════ */

void set_soft_limit_freq(int cpu, unsigned int freq)
{
	struct uag_gov_cpu *sg_cpu = &per_cpu(uag_gov_cpu, cpu);
	struct uag_gov_policy *sg_policy;

	if (!sg_cpu)
		return;
	sg_policy = sg_cpu->sg_policy;
	if (!sg_policy || !sg_policy->tunables)
		return;

	sg_policy->tunables->soft_limit_freq = freq;
	sg_policy->tunables->soft_limit_enable = !!freq;
}
EXPORT_SYMBOL(set_soft_limit_freq);

void set_sugov_tl_uag(int cpu, unsigned int tl)
{
	struct uag_gov_cpu *sg_cpu = &per_cpu(uag_gov_cpu, cpu);
	struct uag_gov_policy *sg_policy;
	struct uag_gov_tunables *t;
	unsigned long flags;

	if (!sg_cpu)
		return;
	sg_policy = sg_cpu->sg_policy;
	if (!sg_policy)
		return;
	t = sg_policy->tunables;
	if (!t)
		return;

	spin_lock_irqsave(&t->target_loads_lock, flags);
	if (t->target_loads != default_target_loads &&
	    t->ntarget_loads > 0)
		t->target_loads[0] = clamp(tl, 1U, 100U);
	spin_unlock_irqrestore(&t->target_loads_lock, flags);
}
EXPORT_SYMBOL_GPL(set_sugov_tl_uag);


/* ═══════════════════════════════════════════════════════════════════════
 * proc/uag entries
 * ═══════════════════════════════════════════════════════════════════════ */

static int uag_pd_capacity_show(struct seq_file *m, void *v)
{
	int cpu;
	for_each_possible_cpu(cpu) {
		seq_printf(m, "cpu%d capacity=%lu\n",
			   cpu, arch_scale_cpu_capacity(cpu));
	}
	return 0;
}
static int uag_pd_capacity_open(struct inode *i, struct file *f)
{
	return single_open(f, uag_pd_capacity_show, NULL);
}
static const struct proc_ops uag_pd_capacity_ops = {
	.proc_open    = uag_pd_capacity_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};


/* ═══════════════════════════════════════════════════════════════════════
 * Module init / exit
 * ═══════════════════════════════════════════════════════════════════════ */

static int __init uag_gov_module_init(void)
{
	int ret;

	uag_proc_root = proc_mkdir("uag", NULL);
	if (!uag_proc_root) {
		pr_warn("cpufreq_uag: mkdir proc/uag failed\n");
	} else {
		if (!proc_create("pd_capacity_tbl", 0444,
				 uag_proc_root, &uag_pd_capacity_ops))
			pr_warn("cpufreq_uag: pd_capacity_tbl create failed\n");
	}

	ret = cpufreq_register_governor(&cpufreq_gov_uag);
	if (ret) {
		pr_err("cpufreq_uag: registration failed (error %d)\n", ret);
		if (uag_proc_root)
			proc_remove(uag_proc_root);
		return ret;
	}

	pr_info("cpufreq_uag: governor registered\n");
	return 0;
}

static void __exit uag_gov_module_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_uag);

	if (uag_proc_root)
		proc_remove(uag_proc_root);
}

module_init(uag_gov_module_init);
module_exit(uag_gov_module_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("UAG CPUFreq Governor — rebuilt on schedutil framework");
MODULE_AUTHOR("custom");
MODULE_ALIAS("cpufreq:uag");
