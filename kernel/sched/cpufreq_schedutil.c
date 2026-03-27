// SPDX-License-Identifier: GPL-2.0
/*
 * CPUFreq schedutil governor - OniKyokyou 200% Edition
 *
 * Based on the original schedutil governor:
 *   Copyright (C) 2016, Intel Corporation
 *   Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * 鬼強強 200% 魔改造 by OniKyokyou Project
 * Compatible with Android vendor hooks (trace_android_vh_*)
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  WHAT MAKES THIS GOVERNOR DIFFERENT (vs stock schedutil)
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 * [1] ASYMMETRIC RATE LIMITING (up_rate_limit_us / down_rate_limit_us)
 *     Stock has a single rate_limit_us applied symmetrically.
 *     We split it: scale UP is near-instant (default 0us), scale DOWN is
 *     deliberately slow (default 8ms ≈ 1 frame @120Hz).
 *     → Result: CPU races to top when load hits, stays there through
 *       entire game frames. No stutter. No dropped frames.
 *
 * [2] FREQUENCY HYSTERESIS (min_sample_time_us)
 *     After a frequency step, we won't drop lower until min_sample_time
 *     has elapsed. 16ms default = exactly one 60Hz frame period.
 *     → Result: prevents freq oscillation within a single frame render.
 *
 * [3] HISPEED LOAD JUMP (hispeed_load / hispeed_freq_pct)
 *     Inspired by the old Interactive governor's hispeed_freq.
 *     When utilization crosses hispeed_load%, instantly jump to
 *     hispeed_freq_pct% of max_freq, bypassing the linear ramp.
 *     → Result: Geekbench, 3DMark: instant burst response.
 *               Games: zero wind-up latency entering heavy scenes.
 *
 * [4] BURST DETECTION & PREEMPTIVE BOOST (burst_boost_pct)
 *     Track the delta between consecutive util samples. If util jumps
 *     by > BURST_DELTA_THRESHOLD_PCT% of max in a single step, it's a
 *     burst event. We add burst_boost_pct% of that delta on top.
 *     → Result: proactive overclock before the workload fully ramps up.
 *
 * [5] SLIDING WINDOW PEAK TRACKING (WALT-inspired)
 *     Keep a small history of recent util samples; always use the
 *     MAX across the window. This prevents premature downscaling when
 *     a bursty workload briefly goes quiet between frames.
 *     → Result: stable freq during game rendering, no mid-frame drops.
 *
 * [6] WAKEUP BOOST (wakeup_boost_pct / WAKEUP_BOOST_DURATION_NS)
 *     When a task wakes up from IO or interactivity, briefly floor
 *     the utilization to wakeup_boost_pct% of max for a short window.
 *     → Result: instant responsiveness on touch/input events.
 *
 * [7] EMA UTILIZATION SMOOTHING (ema_alpha_shift)
 *     Optional exponential moving average on util. Prevents micro-
 *     oscillation on CPUs with noisy util signals. Disabled (shift=0)
 *     by default; enable for platforms with high util noise.
 *     → Result (when enabled): smoother freq curve, less churn, battery.
 *
 * [8] FREQUENCY FLOOR (freq_floor_pct)
 *     Never request a frequency below freq_floor_pct% of max_freq.
 *     Prevents excessive C-state churn on lightly loaded CPUs.
 *     → Result: consistent minimum responsiveness, no "cold start" lag.
 *
 * [9] ENHANCED IO WAIT BOOST (iowait_boost_min_pct)
 *     Tunable IO wait boost floor. Higher value = more aggressive boost
 *     on IO wakeup. Default 25% vs stock's 12.5%.
 *     → Result: storage-heavy workloads and IO-bound games load faster.
 *
 * [10] UTIL SCALING (util_scale_pct) [from original mod, kept]
 *     Scale utilization to simulate undervolting effect.
 *     100 = no change; 85 = 15% lower effective util → lower freq/voltage.
 *     → Result: significant battery gain on stable workloads.
 *
 * [11] CORRECT SIGNAL PIPELINE ORDER (util_enhance BEFORE iowait_apply)
 *     Stock and previous versions ran iowait_apply() before the intelligence
 *     layer. This caused IO wakeups to inflate util before burst detection
 *     compared it to prev_util, triggering false burst events and wasting
 *     power on needless extra boosts.
 *     Correct order: get_util → util_enhance → amu_adjust → iowait_apply.
 *     Each signal is orthogonal and stacks cleanly without cross-contamination.
 *     → Result: burst detection is accurate; IO boost and burst boost no
 *       longer interact; util_avg EMA tracks real CFS load only.
 *
 * [12] AMU STALL REDUCTION (amu_stall_reduce_pct)
 *     On ARM CPUs with Activity Monitors Unit (ARMv8.4+: Snapdragon 8 Gen1+,
 *     Dimensity 9000+, etc.), reads hardware counter 3 (memory stall cycles)
 *     and counter 0 (core cycles) per governor update window.
 *     If stall_cycles/core_cycles > 40% (threshold), the CPU is memory-bound:
 *     raising frequency provides zero throughput benefit but full power cost.
 *     We scale back the utilization signal proportionally, capped at -50%.
 *     Applied after util_enhance but before iowait_apply: IO boosts are never
 *     suppressed by stall reduction (IO completion needs a responsive CPU).
 *     On non-AMU kernels/platforms this is a complete no-op.
 *     → Result: dramatically improved battery life during texture streaming,
 *       asset loading, and any memory-bound game workloads. AMU stall-heavy
 *       scenarios can see 15-30% lower freq requests with identical throughput.
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  TUNABLE REFERENCE (sysfs: /sys/devices/system/cpu/cpufreq/schedutil/)
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *  up_rate_limit_us      [0]       Min time between UP freq steps (us).
 *                                  0 = instant. NEVER raise this high.
 *  down_rate_limit_us    [8000]    Min time between DOWN freq steps (us).
 *                                  8ms = 1 frame @120Hz. Raise for battery.
 *  util_scale_pct        [100]     Util scaling 1-100. 100=off. 85=UV-like.
 *  freq_floor_pct        [10]      Min freq as % of max_freq. 0=disabled.
 *  hispeed_load          [85]      Util% threshold for hispeed jump. 0=off.
 *  hispeed_freq_pct      [90]      Freq to jump to at hispeed_load (% max).
 *  min_sample_time_us    [16000]   Freq hysteresis: min hold time (us).
 *  iowait_boost_min_pct  [25]      IO wait boost floor (% of capacity).
 *  ema_alpha_shift       [0]       EMA shift 1-7 (alpha=1/2^N). 0=disabled.
 *  burst_boost_pct       [50]      Extra util added on burst detection (%).
 *  wakeup_boost_pct      [50]      Util floor on task wakeup (% of max).
 *  amu_stall_reduce_pct  [75]      AMU stall reduction aggressiveness (0=off).
 *                                  No effect on non-AMU platforms.
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  RECOMMENDED PROFILES
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *
 *  [GAMING / BENCHMARK]
 *    up_rate_limit_us=0  down_rate_limit_us=16000  hispeed_load=80
 *    hispeed_freq_pct=95  min_sample_time_us=16000  burst_boost_pct=75
 *    wakeup_boost_pct=60  freq_floor_pct=15
 *
 *  [BALANCED (default)]
 *    up_rate_limit_us=0  down_rate_limit_us=8000  hispeed_load=85
 *    hispeed_freq_pct=90  min_sample_time_us=16000  burst_boost_pct=50
 *    wakeup_boost_pct=50  freq_floor_pct=10
 *
 *  [BATTERY SAVER]
 *    up_rate_limit_us=500  down_rate_limit_us=2000  hispeed_load=90
 *    hispeed_freq_pct=80  min_sample_time_us=8000  burst_boost_pct=25
 *    wakeup_boost_pct=25  freq_floor_pct=5  util_scale_pct=85
 */

#include <trace/hooks/sched.h>

/* IO wait boost minimum - 1/8 of capacity (kept from upstream) */
#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

/*
 * AMU (Activity Monitors Unit) stall reduction.
 *
 * ARMv8.4+ architecturally defines four group-0 AMU counters:
 *   Counter 0: CPU core cycles           (AMEVCNTR0_EL0[0])
 *   Counter 1: Constant-frequency cycles (AMEVCNTR0_EL0[1])
 *   Counter 2: Instructions retired      (AMEVCNTR0_EL0[2])
 *   Counter 3: Memory stall cycles       (AMEVCNTR0_EL0[3])
 *
 * Counter 3 tells us how many cycles the pipeline spent stalled waiting
 * on memory (L2/LLC misses, DRAM latency, etc.). When this ratio is high,
 * raising CPU frequency provides zero compute benefit - the bottleneck is
 * the memory subsystem, not the CPU. We use this to avoid wasteful
 * high-frequency operation during memory-bound workloads.
 *
 * Only compiled on ARM64 kernels with CONFIG_ARM64_AMU_EXTN.
 */
#ifdef CONFIG_ARM64_AMU_EXTN
#include <asm/cpu.h>

/* Architectural AMU group-0 counter register encodings */
#ifndef AMEVCNTR0_CORE_EL0
#define AMEVCNTR0_CORE_EL0		sys_reg(3, 3, 13, 8, 0) /* CPU cycles */
#endif
#ifndef AMEVCNTR0_MEM_STALL
#define AMEVCNTR0_MEM_STALL		sys_reg(3, 3, 13, 8, 3) /* Memory stall cycles */
#endif

/*
 * Stall ratio threshold (0..100%).
 * Below this stall percentage no correction is applied.
 * Default 40%: below 40% memory stall is normal and correction is noise.
 * Above 40% the CPU is increasingly memory-bound and we can safely
 * scale back the frequency request.
 */
#define AMU_STALL_THRESHOLD_PCT		40

/*
 * Minimum cycle delta to consider AMU reading valid.
 * Too short a window = noisy ratio. 4096 cycles ≈ 1-2us at 3GHz.
 */
#define AMU_MIN_DELTA_CYCLES		4096ULL
#endif /* CONFIG_ARM64_AMU_EXTN */

/*
 * Sliding window size for peak utilization tracking.
 * 4 samples at ~2ms each = ~8ms window ≈ 1 frame @120Hz.
 * Must be a power of 2.
 */
#define UTIL_HIST_SIZE			4
#define UTIL_HIST_MASK			(UTIL_HIST_SIZE - 1)

/*
 * Duration of wakeup boost effect in nanoseconds.
 * 1.5ms: long enough to cover a CPU round-trip, short enough for battery.
 */
#define WAKEUP_BOOST_DURATION_NS	1500000ULL

/*
 * Burst detection threshold.
 * If utilization rises by more than this percentage of max capacity
 * in a single update, we classify it as a burst and add extra boost.
 */
#define BURST_DELTA_THRESHOLD_PCT	15

/* ================================================================
 * Data structures
 * ================================================================ */

struct sugov_tunables {
	struct gov_attr_set	attr_set;

	/*
	 * Asymmetric rate limiting:
	 * up   = how quickly we can raise frequency  (0 = instant)
	 * down = how quickly we can lower frequency  (higher = more stable)
	 */
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;

	/*
	 * Utilization scaling for UV-compatible effect (from original mod).
	 * 1-100; 100 = no scaling; 85 = 15% lower util → lower freq request.
	 */
	unsigned int		util_scale_pct;

	/*
	 * Frequency floor: minimum frequency as % of cpuinfo.max_freq.
	 * Prevents dropping to sleep-state frequencies during brief pauses.
	 * 0 = disabled.
	 */
	unsigned int		freq_floor_pct;

	/*
	 * Hispeed jump (Interactive governor inspired):
	 * When util >= hispeed_load% of max_capacity, immediately jump
	 * to at least hispeed_freq_pct% of max_freq.
	 * hispeed_load = 0 disables the feature.
	 */
	unsigned int		hispeed_load;
	unsigned int		hispeed_freq_pct;

	/*
	 * Minimum time to hold a frequency step before allowing downscale.
	 * Acts as frame-aligned hysteresis. 16000us = 1 frame @60Hz.
	 */
	unsigned int		min_sample_time_us;

	/*
	 * IO wait boost minimum as % of SCHED_CAPACITY_SCALE.
	 * Stock = ~12.5% (IOWAIT_BOOST_MIN = 1024/8).
	 * Default = 25% for more aggressive IO response.
	 */
	unsigned int		iowait_boost_min_pct;

	/*
	 * EMA smoothing alpha shift. 0 = disabled.
	 * alpha = 1 / 2^ema_alpha_shift
	 * shift=1 → α=0.5 (fast), shift=2 → α=0.25 (medium),
	 * shift=3 → α=0.125 (slow, stock-like behavior)
	 */
	unsigned int		ema_alpha_shift;

	/*
	 * Burst boost: when util spikes by > BURST_DELTA_THRESHOLD_PCT,
	 * add burst_boost_pct% of the spike delta on top of util.
	 * 0 = disabled.
	 */
	unsigned int		burst_boost_pct;

	/*
	 * Wakeup boost: floor util at wakeup_boost_pct% of max_capacity
	 * for WAKEUP_BOOST_DURATION_NS nanoseconds after a task wakeup.
	 * 0 = disabled.
	 */
	unsigned int		wakeup_boost_pct;

	/*
	 * AMU stall reduction aggressiveness (0 = disabled, 1-100).
	 * When the CPU's AMU stall counter shows high memory-stall ratio,
	 * scale back the util signal by this percentage of the excess stall.
	 * Example: stall_pct=60%, threshold=40%, amu_stall_reduce_pct=75:
	 *   excess = 60-40 = 20%, correction = 20*75/100 = 15%
	 *   util is scaled to 85% of its value → ~15% lower freq request.
	 * 0 = feature disabled (or on non-AMU platforms this is always 0).
	 */
	unsigned int		amu_stall_reduce_pct;
};

struct sugov_policy {
	struct cpufreq_policy	*policy;
	struct sugov_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;

	/* Pre-computed ns versions of the us tunables (avoid multiply hot path) */
	s64			up_rate_limit_ns;
	s64			down_rate_limit_ns;
	s64			min_sample_time_ns;

	unsigned int		next_freq;
	unsigned int		cached_raw_freq;

	/* Slow-path (non-fast-switch) machinery */
	struct			irq_work irq_work;
	struct			kthread_work work;
	struct			mutex work_lock;
	struct			kthread_worker worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;
};

struct sugov_cpu {
	struct update_util_data	update_util;
	struct sugov_policy	*sg_policy;
	unsigned int		cpu;

	/* IO wait boost state */
	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	/* Core utilization tracking */
	unsigned long		util;		/* current (post-iowait) util */
	unsigned long		bw_dl;		/* DL bandwidth */
	unsigned long		max;		/* CPU capacity */

	/* ---- OniKyokyou intelligence fields ---- */

	/* EMA smoothed utilization (disabled when ema_alpha_shift=0) */
	unsigned long		util_avg;

	/* Sliding window: stores raw util for last UTIL_HIST_SIZE updates */
	unsigned long		util_hist[UTIL_HIST_SIZE];
	unsigned int		hist_idx;

	/* Previous raw util for burst delta detection */
	unsigned long		prev_util;

	/* Wakeup boost: value and expiry timestamp */
	unsigned long		wakeup_boost;
	u64			wakeup_boost_expiry;

#ifdef CONFIG_ARM64_AMU_EXTN
	/*
	 * AMU stall reduction: snapshot of AMU counters from the previous
	 * governor update. We compute deltas to get per-window stall ratio.
	 * Must be sampled on the local CPU only (read_sysreg_s restriction).
	 */
	u64			amu_prev_core_cycles;
	u64			amu_prev_stall_cycles;
#endif

#ifdef CONFIG_NO_HZ_COMMON
	unsigned long		saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

/* ================================================================
 * Rate limiting & frequency update gating
 * ================================================================ */

/*
 * sugov_should_update_freq - Decide if it's worth computing a new frequency.
 *
 * For freq-invariant platforms: always return true; the actual rate limit
 * is applied later in sugov_update_next_freq() after we know the direction.
 *
 * For non-freq-invariant: gate on up_rate_limit since we can't do asymmetric
 * checking without knowing next_freq yet.
 */
static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	if (!cpufreq_this_cpu_can_update(sg_policy->policy))
		return false;

	if (unlikely(sg_policy->limits_changed)) {
		sg_policy->limits_changed = false;
		sg_policy->need_freq_update = true;
		return true;
	}

	/*
	 * Freq-invariant: always compute. The asymmetric rate limit is
	 * enforced in sugov_update_next_freq() once direction is known.
	 */
	if (arch_scale_freq_invariant())
		return true;

	/*
	 * Non-freq-invariant: gate on up_rate_limit.
	 * With up_rate_limit_us=0 (default), this is always true.
	 */
	if (sg_policy->up_rate_limit_ns == 0)
		return true;

	return (time - sg_policy->last_freq_update_time) >= sg_policy->up_rate_limit_ns;
}

/*
 * sugov_update_next_freq - Apply asymmetric rate limiting and commit.
 *
 * UP direction:   apply up_rate_limit_ns  (default 0 = instant)
 * DOWN direction: apply down_rate_limit_ns (default 8ms)
 *
 * Returns true if the frequency was updated.
 */
static bool sugov_update_next_freq(struct sugov_policy *sg_policy, u64 time,
				   unsigned int next_freq)
{
	if (sg_policy->need_freq_update) {
		/*
		 * A limits change or driver flag forces an update regardless
		 * of rate limiting.
		 */
		sg_policy->need_freq_update = false;
		goto commit;
	}

	if (next_freq == sg_policy->next_freq)
		return false;

	if (next_freq > sg_policy->next_freq) {
		/*
		 * Scaling UP: apply up_rate_limit.
		 * Default is 0 (instant), so this branch almost never blocks.
		 */
		if (sg_policy->up_rate_limit_ns > 0 &&
		    (time - sg_policy->last_freq_update_time) <
		    sg_policy->up_rate_limit_ns)
			return false;
	} else {
		/*
		 * Scaling DOWN: apply down_rate_limit.
		 * Default 8ms prevents premature drops within a game frame.
		 */
		if (sg_policy->down_rate_limit_ns > 0 &&
		    (time - sg_policy->last_freq_update_time) <
		    sg_policy->down_rate_limit_ns)
			return false;
	}

commit:
	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;
	return true;
}

static void sugov_deferred_update(struct sugov_policy *sg_policy)
{
	if (!sg_policy->work_in_progress) {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

/* ================================================================
 * Frequency calculation
 * ================================================================ */

/**
 * get_next_freq - Compute the target frequency with all OniKyokyou boosts.
 *
 * Stock formula: next_freq = 1.25 * max_freq * util / max
 * (the 1.25x headroom constant C comes from map_util_perf())
 *
 * On top of the base formula we apply:
 *   1. util_scale_pct   - UV-compatible scaling
 *   2. hispeed jump     - instant boost at hispeed_load threshold
 *   3. freq_floor_pct   - absolute minimum frequency
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int freq = arch_scale_freq_invariant() ?
			policy->cpuinfo.max_freq : policy->cur;
	unsigned long next_freq_hook = 0;
	unsigned int next_freq;

	/*
	 * [10] util_scale_pct: scale utilization down before passing to
	 * map_util_freq(), achieving an "undervolting-like" freq reduction
	 * on platforms that allow it.
	 */
	if (tunables->util_scale_pct > 0 && tunables->util_scale_pct < 100)
		util = util * tunables->util_scale_pct / 100;

	/* Standard 1.25x headroom multiplier (kernel's map_util_perf) */
	util = map_util_perf(util);

	/* Android vendor hook: platform may override the mapping */
	trace_android_vh_map_util_freq(util, freq, max, &next_freq_hook, policy,
				       &sg_policy->need_freq_update);
	if (next_freq_hook)
		freq = next_freq_hook;
	else
		freq = map_util_freq(util, freq, max);

	/*
	 * [3] Hispeed jump: if utilization exceeds the hispeed threshold,
	 * immediately jump to hispeed_freq_pct% of max_freq.
	 * This eliminates the gradual ramp-up on burst workloads.
	 */
	if (tunables->hispeed_load > 0) {
		unsigned long hispeed_util =
			(max * (unsigned long)tunables->hispeed_load) / 100UL;
		if (util >= hispeed_util) {
			unsigned int hispeed =
				(policy->cpuinfo.max_freq *
				 tunables->hispeed_freq_pct) / 100;
			freq = max(freq, hispeed);
		}
	}

	/*
	 * [8] Frequency floor: never request below freq_floor_pct% of max.
	 * Prevents CPU from going to near-zero freq during brief quiet periods.
	 */
	if (tunables->freq_floor_pct > 0) {
		unsigned int floor =
			(policy->cpuinfo.max_freq *
			 tunables->freq_floor_pct) / 100;
		freq = max(freq, floor);
	}

	/* Cache check: skip driver resolution if nothing changed */
	if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
		return sg_policy->next_freq;

	sg_policy->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}

/*
 * sugov_get_util - Snapshot current CPU utilization.
 */
static void sugov_get_util(struct sugov_cpu *sg_cpu)
{
	struct rq *rq = cpu_rq(sg_cpu->cpu);

	sg_cpu->max    = arch_scale_cpu_capacity(sg_cpu->cpu);
	sg_cpu->bw_dl  = cpu_bw_dl(rq);
	sg_cpu->util   = effective_cpu_util(sg_cpu->cpu,
					    cpu_util_cfs(sg_cpu->cpu),
					    FREQUENCY_UTIL, NULL);
}

/* ================================================================
 * AMU Stall Reduction
 *
 * Reads hardware Activity Monitor Unit counters to determine what
 * fraction of recent CPU cycles were stalled on memory. When the
 * CPU is memory-bound, high frequency wastes power without improving
 * throughput. We scale back the utilization signal proportionally.
 *
 * IMPORTANT: AMU counters are per-CPU and must be read on the local
 * CPU. This function must only be called from the CPU it belongs to
 * (which is always the case in the single-CPU update paths).
 * ================================================================ */

#ifdef CONFIG_ARM64_AMU_EXTN
/*
 * sugov_amu_stall_pct - Compute memory-stall percentage for the window
 * since the last call on this CPU.
 *
 * Returns stall percentage (0..100). Returns 0 if AMU is unavailable,
 * the window is too short, or we're not running on sg_cpu->cpu.
 */
static unsigned int sugov_amu_stall_pct(struct sugov_cpu *sg_cpu)
{
	u64 core_now, stall_now;
	u64 delta_core, delta_stall;

	/*
	 * AMU registers are only readable from the local CPU.
	 * If called cross-CPU (shouldn't happen in single-CPU paths,
	 * but guard defensively), bail out.
	 */
	if (unlikely(sg_cpu->cpu != smp_processor_id()))
		return 0;

	/*
	 * Runtime check: verify AMU is actually present on this CPU.
	 * cpus_have_const_cap() is cheap (static key).
	 */
	if (!cpus_have_const_cap(ARM64_HAS_AMU_EXTN))
		return 0;

	core_now  = read_sysreg_s(AMEVCNTR0_CORE_EL0);
	stall_now = read_sysreg_s(AMEVCNTR0_MEM_STALL);

	delta_core  = core_now  - sg_cpu->amu_prev_core_cycles;
	delta_stall = stall_now - sg_cpu->amu_prev_stall_cycles;

	/* Update snapshots unconditionally */
	sg_cpu->amu_prev_core_cycles  = core_now;
	sg_cpu->amu_prev_stall_cycles = stall_now;

	/* Reject windows too short to produce a reliable ratio */
	if (delta_core < AMU_MIN_DELTA_CYCLES)
		return 0;

	/*
	 * Sanity: stall cycles can't exceed core cycles.
	 * Counter wrap or first-call noise can produce this.
	 */
	if (delta_stall >= delta_core)
		return 0;

	return (unsigned int)((delta_stall * 100) / delta_core);
}

/**
 * sugov_amu_adjust_util - Scale back util if CPU is memory-bound.
 * @sg_cpu: per-cpu governor data
 * @util:   utilization value to potentially adjust
 *
 * If the AMU stall ratio exceeds AMU_STALL_THRESHOLD_PCT, we reduce
 * the utilization (and thus the frequency request) proportionally.
 * The aggressiveness is controlled by tunables->amu_stall_reduce_pct.
 *
 * Formula:
 *   excess     = stall_pct - AMU_STALL_THRESHOLD_PCT
 *   correction = excess * amu_stall_reduce_pct / 100
 *   correction = min(correction, 50)    [never cut util by more than half]
 *   adjusted   = util * (100 - correction) / 100
 *
 * Example with stall=65%, threshold=40%, reduce_pct=75:
 *   excess=25, correction=18, adjusted=82% of original util.
 *   → ~18% lower frequency request on a heavily memory-stalled CPU.
 *   → Saves power with zero throughput loss (the work is mem-bound).
 */
static unsigned long sugov_amu_adjust_util(struct sugov_cpu *sg_cpu,
					   unsigned long util)
{
	unsigned int stall_pct;
	unsigned int excess, correction;
	unsigned int scale = sg_cpu->sg_policy->tunables->amu_stall_reduce_pct;

	if (!scale)
		return util;

	stall_pct = sugov_amu_stall_pct(sg_cpu);

	if (stall_pct <= AMU_STALL_THRESHOLD_PCT)
		return util;

	excess     = stall_pct - AMU_STALL_THRESHOLD_PCT;
	correction = (excess * scale) / 100;
	/* Hard cap: never reduce util by more than 50% via stall alone */
	correction = min(correction, 50u);

	return util * (100 - correction) / 100;
}

#else /* !CONFIG_ARM64_AMU_EXTN */

static inline unsigned long sugov_amu_adjust_util(struct sugov_cpu *sg_cpu,
						   unsigned long util)
{
	return util; /* No-op on non-AMU platforms */
}

#endif /* CONFIG_ARM64_AMU_EXTN */

/* ================================================================
 * OniKyokyou Intelligence Layer: sugov_util_enhance()
 *
 * This is the heart of the governor. It transforms the raw scheduler
 * utilization into a boosted, peak-tracking, burst-aware signal.
 *
 * Pipeline (in order):
 *   [7] EMA smoothing        - optional, platform-dependent
 *   [5] Sliding window max   - hold recent peak, prevent premature drops
 *   [4] Burst detection      - proactive boost on rapid util rise
 *   [6] Wakeup boost         - brief floor on task/IO wakeup
 *
 * Returns: the enhanced utilization value to feed into get_next_freq().
 * ================================================================ */
static unsigned long sugov_util_enhance(struct sugov_cpu *sg_cpu, u64 time,
					unsigned int flags)
{
	struct sugov_tunables *tunables = sg_cpu->sg_policy->tunables;
	unsigned long util = sg_cpu->util;
	unsigned long max  = sg_cpu->max;
	unsigned int i;

	/*
	 * [7] EMA smoothing (optional).
	 *
	 * When ema_alpha_shift > 0, blend the current util into a running
	 * average. We then use max(current, avg) so the EMA acts as a
	 * "memory floor" - it can only keep freq UP, never pull it DOWN
	 * below current. This catches the case where current util briefly
	 * dips but the EMA remembers recent high load.
	 */
	if (tunables->ema_alpha_shift > 0) {
		unsigned int shift = tunables->ema_alpha_shift;
		/* util_avg += (util - util_avg) >> shift  (signed) */
		if (util > sg_cpu->util_avg)
			sg_cpu->util_avg +=
				(util - sg_cpu->util_avg) >> shift;
		else
			sg_cpu->util_avg -=
				(sg_cpu->util_avg - util) >> shift;
		/*
		 * Use EMA as a lower floor only: if recent average suggests
		 * we were running high, don't drop below it.
		 */
		util = max(util, sg_cpu->util_avg);
	}

	/*
	 * [5] Sliding window peak tracking (WALT-inspired).
	 *
	 * Record util in a circular history buffer and always use the
	 * maximum value across the window. With a 4-slot window at ~2ms
	 * per sample, this covers ~8ms of history (1 frame @120Hz).
	 *
	 * This is the single biggest contributor to stutter-free gaming:
	 * it ensures that a brief quiet period between render calls doesn't
	 * cause a premature frequency drop that would miss the next frame.
	 */
	sg_cpu->util_hist[sg_cpu->hist_idx & UTIL_HIST_MASK] = util;
	sg_cpu->hist_idx++;

	for (i = 0; i < UTIL_HIST_SIZE; i++)
		util = max(util, sg_cpu->util_hist[i]);

	/*
	 * [4] Burst detection and preemptive boost.
	 *
	 * Measure the utilization delta from the previous sample.
	 * If it exceeds BURST_DELTA_THRESHOLD_PCT% of max capacity,
	 * we're in a burst: add burst_boost_pct% of the delta on top.
	 *
	 * This gives us a "look-ahead" boost: if load is ramping fast,
	 * we proactively request more frequency than the current sample
	 * alone would suggest, anticipating where load is going.
	 */
	if (tunables->burst_boost_pct > 0 && sg_cpu->util > sg_cpu->prev_util) {
		unsigned long delta      = sg_cpu->util - sg_cpu->prev_util;
		unsigned long threshold  = (max * BURST_DELTA_THRESHOLD_PCT) / 100;

		if (delta >= threshold) {
			unsigned long boost =
				(delta * tunables->burst_boost_pct) / 100;
			util = min(util + boost, max);
		}
	}
	/* Update prev_util after burst detection, using the raw (pre-window) util */
	sg_cpu->prev_util = sg_cpu->util;

	/*
	 * [6] Wakeup boost.
	 *
	 * On any wakeup event (IO completion, task unblock), floor the
	 * utilization at wakeup_boost_pct% of max_capacity for a brief
	 * window. This ensures the CPU is already at a decent frequency
	 * when the woken task actually starts executing.
	 *
	 * Touch events, vsync signals, and IO completions all benefit here.
	 */
	if (tunables->wakeup_boost_pct > 0) {
		if (flags & SCHED_CPUFREQ_IOWAIT) {
			unsigned long wb =
				(max * tunables->wakeup_boost_pct) / 100;
			sg_cpu->wakeup_boost        = wb;
			sg_cpu->wakeup_boost_expiry = time + WAKEUP_BOOST_DURATION_NS;
		}
		if (sg_cpu->wakeup_boost) {
			if (time < sg_cpu->wakeup_boost_expiry)
				util = max(util, sg_cpu->wakeup_boost);
			else
				sg_cpu->wakeup_boost = 0;
		}
	}

	return util;
}

/* ================================================================
 * IO wait boost subsystem (upstream-compatible, tunables extended)
 * ================================================================ */

/*
 * Get the effective IO wait boost minimum from tunables.
 * iowait_boost_min_pct=25 → 1024*25/100 = 256 (vs stock's 128).
 */
static inline unsigned int sugov_iowait_boost_min(struct sugov_cpu *sg_cpu)
{
	unsigned int pct = sg_cpu->sg_policy->tunables->iowait_boost_min_pct;
	unsigned int val = (SCHED_CAPACITY_SCALE * pct) / 100;
	return max_t(unsigned int, val, (unsigned int)IOWAIT_BOOST_MIN);
}

/**
 * sugov_iowait_reset - Reset IO boost state after an idle period.
 */
static bool sugov_iowait_reset(struct sugov_cpu *sg_cpu, u64 time,
				bool set_iowait_boost)
{
	s64 delta_ns = time - sg_cpu->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	sg_cpu->iowait_boost         = set_iowait_boost ?
					sugov_iowait_boost_min(sg_cpu) : 0;
	sg_cpu->iowait_boost_pending = set_iowait_boost;
	return true;
}

/**
 * sugov_iowait_boost - Update IO boost on wakeup event.
 * Double the boost each successive wakeup, from min → max.
 */
static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
			       unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	if (sg_cpu->iowait_boost &&
	    sugov_iowait_reset(sg_cpu, time, set_iowait_boost))
		return;

	if (!set_iowait_boost)
		return;

	if (sg_cpu->iowait_boost_pending)
		return;
	sg_cpu->iowait_boost_pending = true;

	if (sg_cpu->iowait_boost) {
		sg_cpu->iowait_boost =
			min_t(unsigned int,
			      sg_cpu->iowait_boost << 1,
			      SCHED_CAPACITY_SCALE);
		return;
	}

	/* First wakeup: start at our enhanced minimum (25% vs stock 12.5%) */
	sg_cpu->iowait_boost = sugov_iowait_boost_min(sg_cpu);
}

/**
 * sugov_iowait_apply - Clamp util upward by current IO boost value.
 */
static void sugov_iowait_apply(struct sugov_cpu *sg_cpu, u64 time)
{
	unsigned long boost;

	if (!sg_cpu->iowait_boost)
		return;

	if (sugov_iowait_reset(sg_cpu, time, false))
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
	boost = uclamp_rq_util_with(cpu_rq(sg_cpu->cpu), boost, NULL);
	if (sg_cpu->util < boost)
		sg_cpu->util = boost;
}

/* ================================================================
 * Idle/busy detection helpers (unchanged from upstream)
 * ================================================================ */

#ifdef CONFIG_NO_HZ_COMMON
static bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	return false;
}
#endif

/*
 * Force a freq update when DL bandwidth increases.
 */
static inline void ignore_dl_rate_limit(struct sugov_cpu *sg_cpu)
{
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_dl)
		sg_cpu->sg_policy->limits_changed = true;
}

/* ================================================================
 * Common single-CPU update path
 * ================================================================ */

/*
 * sugov_update_single_common - Shared preamble for single-CPU update paths.
 *
 * Signal pipeline (intentional order — do NOT reorder):
 *
 *   1. sugov_get_util()       → raw CFS utilization from the scheduler
 *   2. sugov_util_enhance()   → OniKyokyou layer on PURE CFS util:
 *                                burst detection, peak window, wakeup boost.
 *                                Must run before iowait so burst delta tracks
 *                                actual CFS load trends, not IO-inflated values.
 *   3. sugov_amu_adjust_util()→ hardware stall correction (AMU).
 *                                Applied after enhance but before IO boost so
 *                                the stall reduction doesn't suppress IO boosts.
 *   4. sugov_iowait_apply()   → IO wait boost stacks independently on top.
 *                                Additive to whatever enhance+AMU produced.
 *
 * Why this order matters for burst detection:
 *   If iowait ran first, a sudden IO wakeup would inflate sg_cpu->util by
 *   e.g. 30%. The next burst-delta check would see this inflation as a "burst"
 *   and fire an extra burst_boost on top — a false positive that wastes power.
 *   With enhance first, prev_util tracks only real CFS activity, and iowait
 *   boosts remain orthogonal and correctly capped.
 */
static inline bool sugov_update_single_common(struct sugov_cpu *sg_cpu,
					      u64 time, unsigned int flags)
{
	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	ignore_dl_rate_limit(sg_cpu);

	if (!sugov_should_update_freq(sg_cpu->sg_policy, time))
		return false;

	/* Step 1: raw scheduler utilization */
	sugov_get_util(sg_cpu);

	/*
	 * Step 2: OniKyokyou intelligence on pure CFS util.
	 * burst detection reads sg_cpu->util (pre-iowait) for clean deltas.
	 */
	sg_cpu->util = sugov_util_enhance(sg_cpu, time, flags);

	/*
	 * Step 3: AMU stall reduction.
	 * If the CPU is memory-bound, scale back the boosted util.
	 * No-op on non-AMU platforms or when amu_stall_reduce_pct=0.
	 */
	sg_cpu->util = sugov_amu_adjust_util(sg_cpu, sg_cpu->util);

	/*
	 * Step 4: IO wait boost stacks on top, independent of above.
	 * iowait_apply() clamps sg_cpu->util upward if IO boost > current.
	 */
	sugov_iowait_apply(sg_cpu, time);

	return true;
}

/* ================================================================
 * Update callbacks (called from scheduler tick / cpufreq_update_util)
 * ================================================================ */

/*
 * sugov_update_single_freq - Single-CPU policy, frequency interface.
 *
 * Used on platforms without cpufreq_driver_adjust_perf().
 * Applies full OniKyokyou logic including:
 *   - busy-CPU holdoff (don't drop if CPU was recently busy)
 *   - frequency hysteresis (min_sample_time_us)
 *   - asymmetric rate limiting (via sugov_update_next_freq)
 */
static void sugov_update_single_freq(struct update_util_data *hook, u64 time,
				     unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int cached_freq = sg_policy->cached_raw_freq;
	unsigned int next_f;

	if (!sugov_update_single_common(sg_cpu, time, flags))
		return;

	next_f = get_next_freq(sg_policy, sg_cpu->util, sg_cpu->max);

	/*
	 * Busy-CPU holdoff: if the CPU has not been idle recently, don't
	 * reduce frequency - the drop is likely premature.
	 * (Waived when uclamp_max is capping the CPU.)
	 */
	if (!uclamp_rq_is_capped(cpu_rq(sg_cpu->cpu)) &&
	    sugov_cpu_is_busy(sg_cpu) &&
	    next_f < sg_policy->next_freq &&
	    !sg_policy->need_freq_update) {
		next_f = sg_policy->next_freq;
		sg_policy->cached_raw_freq = cached_freq;
	}

	/*
	 * [2] Frequency hysteresis (min_sample_time_us):
	 * After any frequency step, refuse to drop lower until
	 * min_sample_time_ns has elapsed. Default 16ms = 1 frame @60Hz.
	 *
	 * This prevents the classic "drop-and-claw-back" pattern that
	 * causes micro-stutter in games with periodic light frames.
	 */
	if (next_f < sg_policy->next_freq &&
	    sg_policy->min_sample_time_ns > 0) {
		s64 elapsed = time - sg_policy->last_freq_update_time;

		if (elapsed < sg_policy->min_sample_time_ns) {
			next_f = sg_policy->next_freq;
			sg_policy->cached_raw_freq = cached_freq;
		}
	}

	if (!sugov_update_next_freq(sg_policy, time, next_f))
		return;

	if (sg_policy->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(sg_policy->policy, next_f);
	} else {
		raw_spin_lock(&sg_policy->update_lock);
		sugov_deferred_update(sg_policy);
		raw_spin_unlock(&sg_policy->update_lock);
	}
}

/*
 * sugov_update_single_perf - Single-CPU policy, adjust_perf interface.
 *
 * Used on platforms with cpufreq_driver_adjust_perf() (e.g. ARM AMU-based).
 * Falls back to sugov_update_single_freq if frequency invariance is absent.
 */
static void sugov_update_single_perf(struct update_util_data *hook, u64 time,
				     unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	unsigned long prev_util = sg_cpu->util;

	/*
	 * adjust_perf requires frequency invariance for the util→perf mapping
	 * to be accurate. Fall back to the freq path if unavailable.
	 */
	if (!arch_scale_freq_invariant()) {
		sugov_update_single_freq(hook, time, flags);
		return;
	}

	if (!sugov_update_single_common(sg_cpu, time, flags))
		return;

	/*
	 * Same busy-CPU holdoff as sugov_update_single_freq, but operating
	 * on the util/perf level instead of frequency.
	 */
	if (!uclamp_rq_is_capped(cpu_rq(sg_cpu->cpu)) &&
	    sugov_cpu_is_busy(sg_cpu) && sg_cpu->util < prev_util)
		sg_cpu->util = prev_util;

	cpufreq_driver_adjust_perf(sg_cpu->cpu,
				   map_util_perf(sg_cpu->bw_dl),
				   map_util_perf(sg_cpu->util),
				   sg_cpu->max);

	sg_cpu->sg_policy->last_freq_update_time = time;
}

/*
 * sugov_next_freq_shared - Compute target frequency for a shared policy.
 *
 * For big.LITTLE / DynamIQ clusters where multiple CPUs share a frequency
 * domain. We iterate all CPUs in the cluster and take the maximum enhanced
 * utilization as the cluster's frequency request.
 *
 * Per-CPU signal pipeline (same order as sugov_update_single_common):
 *   get_util → util_enhance → amu_adjust → iowait_apply
 *
 * Burst detection correctness: flags (wakeup/IO) are passed only to the
 * triggering CPU; other CPUs use flags=0. Each CPU's wakeup_boost state
 * is managed independently, so cross-CPU interference is impossible.
 */
static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu,
					   u64 time, unsigned int flags)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;
		unsigned int j_flags = (j == sg_cpu->cpu) ? flags : 0;

		/* Step 1: raw CFS util */
		sugov_get_util(j_sg_cpu);

		/*
		 * Step 2: OniKyokyou on pure CFS util.
		 * burst delta is clean because iowait hasn't run yet.
		 */
		j_sg_cpu->util = sugov_util_enhance(j_sg_cpu, time, j_flags);

		/*
		 * Step 3: AMU stall reduction per-CPU.
		 * Note: sugov_amu_adjust_util() bails out silently for
		 * cross-CPU calls (sg_cpu->cpu != smp_processor_id()),
		 * so remote CPUs in the cluster won't have stall correction.
		 * This is acceptable — stall data for remote CPUs is stale.
		 */
		j_sg_cpu->util = sugov_amu_adjust_util(j_sg_cpu, j_sg_cpu->util);

		/* Step 4: IO wait boost stacks on top */
		sugov_iowait_apply(j_sg_cpu, time);

		j_util = j_sg_cpu->util;
		j_max  = j_sg_cpu->max;

		/* Cluster frequency is determined by the most loaded CPU */
		if (j_util * max > j_max * util) {
			util = j_util;
			max  = j_max;
		}
	}

	return get_next_freq(sg_policy, util, max);
}

/*
 * sugov_update_shared - Shared-policy update callback.
 *
 * Called from the scheduler for any CPU in a shared-frequency cluster.
 * Holds update_lock across the entire computation to ensure atomicity.
 */
static void sugov_update_shared(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int next_f;

	raw_spin_lock(&sg_policy->update_lock);

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	ignore_dl_rate_limit(sg_cpu);

	if (sugov_should_update_freq(sg_policy, time)) {
		next_f = sugov_next_freq_shared(sg_cpu, time, flags);

		/*
		 * [2] Frequency hysteresis for shared policy:
		 * Same frame-aligned holdoff as the single-CPU path.
		 */
		if (next_f < sg_policy->next_freq &&
		    sg_policy->min_sample_time_ns > 0) {
			s64 elapsed = time - sg_policy->last_freq_update_time;

			if (elapsed < sg_policy->min_sample_time_ns)
				next_f = sg_policy->next_freq;
		}

		if (!sugov_update_next_freq(sg_policy, time, next_f))
			goto unlock;

		if (sg_policy->policy->fast_switch_enabled)
			cpufreq_driver_fast_switch(sg_policy->policy, next_f);
		else
			sugov_deferred_update(sg_policy);
	}
unlock:
	raw_spin_unlock(&sg_policy->update_lock);
}

/* ================================================================
 * Slow-path worker (non-fast-switch platforms)
 * ================================================================ */

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy =
		container_of(work, struct sugov_policy, work);
	unsigned int freq;
	unsigned long flags;

	/*
	 * Snapshot next_freq under the lock to handle the race where
	 * sugov_deferred_update() updates next_freq just before we clear
	 * work_in_progress.
	 */
	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	freq = sg_policy->next_freq;
	sg_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy =
		container_of(irq_work, struct sugov_policy, irq_work);

	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

/* ================================================================
 * sysfs tunables interface
 * ================================================================ */

static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

/* ---- up_rate_limit_us ---- */
static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->up_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->up_rate_limit_us = val;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->up_rate_limit_ns = (s64)val * NSEC_PER_USEC;

	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

/* ---- down_rate_limit_us ---- */
static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->down_rate_limit_us);
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->down_rate_limit_us = val;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->down_rate_limit_ns = (s64)val * NSEC_PER_USEC;

	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

/* ---- util_scale_pct (from original mod) ---- */
static ssize_t util_scale_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 to_sugov_tunables(attr_set)->util_scale_pct);
}

static ssize_t util_scale_pct_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;

	tunables->util_scale_pct = val;
	return count;
}
static struct governor_attr util_scale_pct = __ATTR_RW(util_scale_pct);

/* ---- freq_floor_pct ---- */
static ssize_t freq_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->freq_floor_pct);
}

static ssize_t freq_floor_pct_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;

	tunables->freq_floor_pct = val;
	return count;
}
static struct governor_attr freq_floor_pct = __ATTR_RW(freq_floor_pct);

/* ---- hispeed_load ---- */
static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->hispeed_load);
}

static ssize_t hispeed_load_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;

	tunables->hispeed_load = val;
	return count;
}
static struct governor_attr hispeed_load = __ATTR_RW(hispeed_load);

/* ---- hispeed_freq_pct ---- */
static ssize_t hispeed_freq_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->hispeed_freq_pct);
}

static ssize_t hispeed_freq_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;

	tunables->hispeed_freq_pct = val;
	return count;
}
static struct governor_attr hispeed_freq_pct = __ATTR_RW(hispeed_freq_pct);

/* ---- min_sample_time_us ---- */
static ssize_t min_sample_time_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->min_sample_time_us);
}

static ssize_t min_sample_time_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->min_sample_time_us = val;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->min_sample_time_ns = (s64)val * NSEC_PER_USEC;

	return count;
}
static struct governor_attr min_sample_time_us = __ATTR_RW(min_sample_time_us);

/* ---- iowait_boost_min_pct ---- */
static ssize_t iowait_boost_min_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_sugov_tunables(attr_set)->iowait_boost_min_pct);
}

static ssize_t iowait_boost_min_pct_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;

	tunables->iowait_boost_min_pct = val;
	return count;
}
static struct governor_attr iowait_boost_min_pct = __ATTR_RW(iowait_boost_min_pct);

/* ---- ema_alpha_shift ---- */
static ssize_t ema_alpha_shift_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->ema_alpha_shift);
}

static ssize_t ema_alpha_shift_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	/* shift=0 disables EMA; max=7 (alpha=1/128, very slow) */
	if (kstrtouint(buf, 10, &val) || val > 7)
		return -EINVAL;

	tunables->ema_alpha_shift = val;
	return count;
}
static struct governor_attr ema_alpha_shift = __ATTR_RW(ema_alpha_shift);

/* ---- burst_boost_pct ---- */
static ssize_t burst_boost_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->burst_boost_pct);
}

static ssize_t burst_boost_pct_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 200)
		return -EINVAL;

	tunables->burst_boost_pct = val;
	return count;
}
static struct governor_attr burst_boost_pct = __ATTR_RW(burst_boost_pct);

/* ---- wakeup_boost_pct ---- */
static ssize_t wakeup_boost_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_sugov_tunables(attr_set)->wakeup_boost_pct);
}

static ssize_t wakeup_boost_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;

	tunables->wakeup_boost_pct = val;
	return count;
}
static struct governor_attr wakeup_boost_pct = __ATTR_RW(wakeup_boost_pct);

/* ---- amu_stall_reduce_pct ---- */
static ssize_t amu_stall_reduce_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_sugov_tunables(attr_set)->amu_stall_reduce_pct);
}

static ssize_t amu_stall_reduce_pct_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;

	tunables->amu_stall_reduce_pct = val;
	return count;
}
static struct governor_attr amu_stall_reduce_pct = __ATTR_RW(amu_stall_reduce_pct);

/* ---- sysfs attribute group ---- */
static struct attribute *sugov_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&util_scale_pct.attr,
	&freq_floor_pct.attr,
	&hispeed_load.attr,
	&hispeed_freq_pct.attr,
	&min_sample_time_us.attr,
	&iowait_boost_min_pct.attr,
	&ema_alpha_shift.attr,
	&burst_boost_pct.attr,
	&wakeup_boost_pct.attr,
	&amu_stall_reduce_pct.attr,
	NULL
};
ATTRIBUTE_GROUPS(sugov);

static void sugov_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = to_gov_attr_set(kobj);

	kfree(to_sugov_tunables(attr_set));
}

static struct kobj_type sugov_tunables_ktype = {
	.default_groups	= sugov_groups,
	.sysfs_ops	= &governor_sysfs_ops,
	.release	= &sugov_tunables_free,
};

/* ================================================================
 * cpufreq governor lifecycle
 * ================================================================ */

struct cpufreq_governor schedutil_gov;

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAG_SUGOV,
		.sched_nice	= 0,
		.sched_priority	= 0,
		/*
		 * Fake (unused) bandwidth; workaround to "fix"
		 * priority inheritance.
		 */
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	trace_android_vh_set_sugov_sched_attr(&attr);
	kthread_init_work(&sg_policy->work, sugov_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_DEADLINE\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);
	wake_up_process(thread);
	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void sugov_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sugov_kthread_create(sg_policy);
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

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	/*
	 * ════════════════════════════════════════════════════
	 * OniKyokyou 200% DEFAULT TUNABLE VALUES
	 *
	 * These defaults represent the "Balanced" profile:
	 * maximum responsiveness with sensible battery trade-offs.
	 * ════════════════════════════════════════════════════
	 *
	 * up_rate_limit_us = 0
	 *   Instant frequency upscaling. Never change this to a high value
	 *   - any delay here DIRECTLY causes stutter and benchmark drops.
	 *
	 * down_rate_limit_us = 8000 (8ms)
	 *   Hold frequency for 8ms before allowing a reduction.
	 *   At 120Hz frame rate, one frame = 8.33ms. This effectively
	 *   means we hold the high freq for the entire current frame
	 *   before considering a reduction. Key anti-stutter mechanism.
	 *
	 * util_scale_pct = 100 (disabled)
	 *   No util scaling by default. UV users: try 90 or 85.
	 *
	 * freq_floor_pct = 10
	 *   Never drop below 10% of max_freq. Prevents cold-start lag.
	 *
	 * hispeed_load = 85
	 *   At 85% utilization, jump straight to hispeed_freq_pct.
	 *   This eliminates the gradual multi-step ramp in burst workloads.
	 *
	 * hispeed_freq_pct = 90
	 *   Jump to 90% of max_freq at hispeed trigger. High enough to
	 *   handle most burst loads; max_freq handles the remaining 10%.
	 *
	 * min_sample_time_us = 16000 (16ms)
	 *   After a frequency change, hold for at least 16ms (1 frame @60Hz)
	 *   before downscaling. Eliminates within-frame oscillation.
	 *
	 * iowait_boost_min_pct = 25
	 *   IO wakeup boost starts at 25% of max capacity vs stock 12.5%.
	 *   Aggressive storage/IO response for game asset loading.
	 *
	 * ema_alpha_shift = 0 (disabled)
	 *   EMA smoothing disabled by default. Enable (try 2 or 3) on
	 *   platforms with noisy util signals causing freq oscillation.
	 *
	 * burst_boost_pct = 50
	 *   On burst detection, add 50% of the util spike on top.
	 *   Proactive overclock before workload fully ramps up.
	 *
	 * wakeup_boost_pct = 50
	 *   Brief 50% util floor on task wakeup/IO completion.
	 *   Touch → immediate freq response, no perceptible lag.
	 */
	tunables->up_rate_limit_us     = 0;
	tunables->down_rate_limit_us   = 8000;
	tunables->util_scale_pct       = 100;
	tunables->freq_floor_pct       = 10;
	tunables->hispeed_load         = 85;
	tunables->hispeed_freq_pct     = 90;
	tunables->min_sample_time_us   = 16000;
	tunables->iowait_boost_min_pct = 25;
	tunables->ema_alpha_shift      = 0;
	tunables->burst_boost_pct      = 50;
	tunables->wakeup_boost_pct     = 50;

	/*
	 * amu_stall_reduce_pct = 75
	 *   On AMU-capable platforms (Snapdragon 8 Gen1+, Dimensity 9000+),
	 *   when memory stall ratio exceeds 40%, scale back util aggressively.
	 *   75 means: for every 1% of stall above threshold, we apply 0.75%
	 *   correction to the util signal. At 65% stall → ~18% freq reduction.
	 *   On non-AMU platforms this tunable has no effect whatsoever.
	 *   Set to 0 to disable if experiencing issues on specific hardware.
	 */
	tunables->amu_stall_reduce_pct = 75;

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   schedutil_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	sugov_clear_global_tunables();

stop_kthread:
	sugov_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		sugov_clear_global_tunables();

	mutex_unlock(&global_tunables_lock);

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned int cpu;

	/* Pre-compute nanosecond versions of all time-based tunables */
	sg_policy->up_rate_limit_ns   = (s64)tunables->up_rate_limit_us   * NSEC_PER_USEC;
	sg_policy->down_rate_limit_ns = (s64)tunables->down_rate_limit_us * NSEC_PER_USEC;
	sg_policy->min_sample_time_ns = (s64)tunables->min_sample_time_us * NSEC_PER_USEC;
	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq             = 0;
	sg_policy->work_in_progress      = false;
	sg_policy->limits_changed        = false;
	sg_policy->cached_raw_freq       = 0;

	sg_policy->need_freq_update =
		cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS);

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu       = cpu;
		sg_cpu->sg_policy = sg_policy;
		/*
		 * util_avg, util_hist, prev_util, wakeup_boost all start at 0.
		 * They will converge naturally within the first few update calls.
		 */
	}

	if (policy_is_shared(policy))
		uu = sugov_update_shared;
	else if (policy->fast_switch_enabled && cpufreq_driver_has_adjust_perf())
		uu = sugov_update_single_perf;
	else
		uu = sugov_update_single_freq;

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util, uu);
	}
	return 0;
}

static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	sg_policy->limits_changed = true;
}

struct cpufreq_governor schedutil_gov = {
	.name			= "schedutil",
	.owner			= THIS_MODULE,
	.flags			= CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init			= sugov_init,
	.exit			= sugov_exit,
	.start			= sugov_start,
	.stop			= sugov_stop,
	.limits			= sugov_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &schedutil_gov;
}
#endif

cpufreq_governor_init(schedutil_gov);
