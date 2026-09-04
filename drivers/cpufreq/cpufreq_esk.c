// SPDX-License-Identifier: GPL-2.0
/*
 * ESK CPUFreq Governor v2.2 — schedutil-derived, tri-cluster.
 *
 * Two profiles: gaming (high band + frame pacing) and daily (ceiling-relative
 * caps/floors). Policy-wide directional EMA util, load-proportional headroom,
 * frame-risk boost, latched thermal net. Every floor, cap and frame percent is
 * a percentage of the effective ceiling (fceil), never of hardware fmax.
 *
 * v2.2 port of Templar's Vorpal (upstream Linux6-Staging end state: v2.2
 * upgrade + the Linux4 portability/daily fold, Linux5 time-domain fixes,
 * Linux6 sustained-cap fix). Key v2.2 properties kept intact:
 *   • single lock, zero early-returns inside it in the util hook
 *   • EMA/ramp bookkeeping advances by consumed periods (sub-period hold)
 *   • elapsed-time helpers clamp rq_clock skew across a shared policy
 *   • fceil reads policy->max against a max_seen high-water baseline (MTK HAL)
 * ESK-specific adaptations (from our v2.1 fork, kept deliberately):
 *   • Prime tier = policy owning the topmost CPU (MT6895: CPU7 is a separate
 *     DVFS domain whose capacity equals Big's, so capacity-based tier tests
 *     cannot distinguish them), not Templar's distinct-capacity count
 *   • prime_gaming_floor_pct sysfs knob on the prime policy
 *   • GKI 5.10 util interface (esk_*_gki510 helpers in schedutil)
 *
 * Author: Templar Dev (Steambot12)
 * Renamed/adapted to ESK for AKALIorg ESK-Reborn (MT6895)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/topology.h>
#include <linux/rcupdate.h>
#include <linux/sched/rt.h>
#include <linux/sched/cpufreq.h>
#include <uapi/linux/sched/types.h>
#include <linux/tick.h>
#include <linux/timekeeping.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/irq_work.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/list.h>
#ifdef CONFIG_THERMAL
#include <linux/thermal.h>
#endif

#define CPUFREQ_ESK_NAME     "esk"
#define CPUFREQ_ESK_VERSION  "2.2"
#define CPUFREQ_ESK_AUTHOR   "Templar Dev"

/* Core-sched helpers (owned by core sched): util getter, DL-bandwidth check,
 * and the SUGOV DL class setter for the slow-path worker. */
extern void esk_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern bool esk_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);
extern int esk_setattr_sugov_gki510(struct task_struct *t);

/* ===================================================================== */
/* Tunable defaults (KMI-safe: plain #defines).                          */
/* ===================================================================== */

/* Cluster identification by arch capacity. */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* DAILY eval rate limits (us), at rest only: gaming, interaction and the DL
 * bypass all override. 3ms matches PELT's ~32ms half-life; finer is overhead.
 * up=0 = commit on the first eval that sees the rise. */
#define RFX_LITTLE_RATE_US		3000
#define RFX_LITTLE_UP_US		200
#define RFX_LITTLE_DOWN_US		3000

#define RFX_BIG_RATE_US			3000
#define RFX_BIG_UP_US			0
#define RFX_BIG_DOWN_US			2500

/* Gaming eval rate. Measured-stable; do not raise without an FPS measurement. */
#define RFX_FAST_RATE_US		250

/* Daily interaction rate: ~5 evals per 120Hz frame, PELT moves ~2% across it,
 * so the rise is not missed. */
#define RFX_UI_RATE_US			1500

/* Gaming down-rate gate. Sets descent GRANULARITY only: it also caps the slew
 * window, so the descent rate stays RFX_GAMING_DOWN_PCT_PER_MS either way and
 * a longer gate just means fewer, larger steps. One 120Hz frame -- at 4ms an
 * unsaturated cluster took two downward OPP transitions per frame, each one a
 * hardware DVFS transition (plus a kthread hop where fast-switch is absent) a
 * frame can land inside. Rises are never gated (up-rate is 0). */
#define RFX_GAMING_DOWN_US		8300

/* Gaming floors, percent of the effective ceiling. NO cluster is capped: every
 * cluster tracks demand up to fceil, so a heavy frame is never throttled into a
 * util pile-up. Floors only cover a cold landing -- and they are the gaming
 * resting-power dial.
 *
 * Do not lower a *_FLOOR_PCT on a tier that may render (58/80 on the top tier
 * measured as an FPS drop), and do not raise one either: the extra heat lowers
 * fceil and the render cluster leaves fmax. Frame floors are frame-miss
 * recovery; recovery from a lower OPP misses the deadline. */
#define RFX_G_PRIME_FLOOR_PCT		64
#define RFX_G_PRIME_FRAME_PCT		92
#define RFX_G_BIG_FLOOR_PCT		58
#define RFX_G_BIG_FRAME_PCT		90
/* Little never renders (compositor/input/audio, demand-tracked at ~30-40%), so
 * both its floors are pure resting power and sit just above the V/f knee.
 * Demand and up-rate-0 still take it as high as a frame needs. */
#define RFX_G_LITTLE_FLOOR_PCT		45
#define RFX_G_LITTLE_FLOOR_BOOST_PCT	66

/* Max downward slew, percent of ceiling per ms elapsed. Only stops one eval
 * stepping off a cliff; the EMA owns descent SHAPE, so this stays LOOSER than
 * the filter -- the pair is tuned together. Rises untouched. */
#define RFX_GAMING_DOWN_PCT_PER_MS	1

/* ---- Daily frequency shaping, percent of the effective ceiling ---- */
/* Little daily cap: just above the V/f knee, so light scrolling stays on one
 * voltage step instead of toggling across it. */
#define RFX_D_LITTLE_CAP_PCT		65
#define RFX_D_LITTLE_BOOST_CAP_PCT	80
/* Little interaction floor, window-scoped: holds Little off fmin between scroll
 * frames. One voltage step below the knee. */
#define RFX_D_LITTLE_UI_FLOOR_PCT	32
/* Sustained cap: long background work (media scan, sync) at lower voltage. */
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	80
/* Sustained latch, skewed 1.25x => real demand on at 58%, off at 44%. */
#define RFX_D_LITTLE_LIFT_PCT		72
#define RFX_D_LITTLE_DROP_PCT		55
/*
 * Daily Big/Prime caps keep unperceived background work off the top OPPs. A
 * touch/UI window lifts to *_BOOST_CAP_PCT; the sustained latch lifts to
 * *_SUSTAINED_CAP_PCT for long foreground work.
 *
 * Prime's boost cap matches Big's: a flat one meant the interaction lift did not
 * exist on parts that composite on the top tier. Latch skewed 1.25x => real
 * demand on at 64%, off at 54%; at 52/44 ordinary foreground rendering ratcheted
 * Big to the ceiling and held it. Both edges move together.
 *
 * A sustained cap may not EXCEED the boost cap (Little already obeyed this).
 * Big at 94 outranked its own 80% interaction cap: work with no deadline got the
 * top voltage steps while the frame under the finger was capped lower, and
 * ordinary foreground use sits inside the latch band, so it held for hours.
 * Sustained means "hold the interactive cap with no finger down", not "faster
 * than interactive".
 */
#define RFX_D_BIG_CAP_PCT		70
#define RFX_D_BIG_BOOST_CAP_PCT		80
#define RFX_D_PRIME_CAP_PCT		68
#define RFX_D_PRIME_BOOST_CAP_PCT	80
#define RFX_D_BIG_LIFT_PCT		80
#define RFX_D_BIG_DROP_PCT		68
#define RFX_D_BIG_SUSTAINED_CAP_PCT	80
#define RFX_D_PRIME_SUSTAINED_CAP_PCT	80

/* Cold-start burst floors, clamped to cap: cover an app launch until real
 * demand is visible. */
#define RFX_D_BIG_BURST_FLOOR_PCT	45
#define RFX_D_PRIME_BURST_FLOOR_PCT	42

/* Daily burst delta: catches animation ramps without firing on video/background
 * work. Measured against a FIXED 16ms base (one 60Hz frame), not the variable
 * eval spacing, else the detector goes blind under the finger. */
#define RFX_D_RAMP_DELTA_PCT		12
#define RFX_D_RAMP_SAMPLE_NS		(16 * NSEC_PER_MSEC)
/* Cold-start: 40% demand jump from <=10% base = app launch, not sensor work. */
#define RFX_D_COLDSTART_DELTA_PCT	40
#define RFX_D_COLDSTART_BASE_PCT	10
/* UI boost covers animation (140-160ms). 220ms held the 80% cap through a whole
 * fling, so passive autoplay paid scroll heat. */
#define RFX_D_UI_BOOST_NS		(150 * NSEC_PER_MSEC)
/* Cold-start boost: spawn + initial layout + first render. */
#define RFX_D_COLDSTART_BOOST_NS	(200 * NSEC_PER_MSEC)

/* Touch window: keyboard popup (180-220ms), momentum tail trimmed. */
#define RFX_INPUT_WINDOW_NS		(230 * NSEC_PER_MSEC)

/* ---- Util EMA: rise instant, decay time-normalised, so the time constant is
 * independent of eval rate (250us gaming .. 3ms daily). Period = interval that
 * removes 1/RFX_EMA_GAMING_DIVISOR of the remaining error. ---- */
#define RFX_EMA_DECAY_PERIOD_NS		250000	/* 250us: one gaming eval */
/* Gaming decay: 1/100 per period = ~25ms tau. Must span more than one frame gap
 * or the inter-frame trough collapses the render floor every frame; 50 (12.5ms)
 * tracked intra-frame duty instead of the scene and measured as an FPS drop.
 * The slew bound must stay LOOSER than this filter -- the two move together. */
#define RFX_EMA_GAMING_DIVISOR		100
/* Step cap: 32 periods = 8ms, one frame gap. Bounds util-hook work. */
#define RFX_EMA_MAX_STEPS		32

/* ---- Headroom above demand, percent. Stacks on the 25% DVFS margin already
 * applied by esk_get_util_gki510, so this only lowers the resting OPP; a real
 * frame still reaches fmax via the saturation shortcut. ---- */
#define RFX_HEADROOM_DAILY_HIGH		4
#define RFX_HEADROOM_DAILY_MID		2
/* Gaming headroom: applied every eval, so it is the steady-state power lever.
 * At 5% a sustained 70-80% render load pinned Big flat at fmax while FPS stayed
 * engine-bound. Do not raise without a measurement showing FPS -- not just CPU%
 * -- falling. */
#define RFX_HEADROOM_GAMING		2

/* Util percent at which we stop interpolating and request fmax outright.
 * Gaming 98: at 95 a sustained 70-80% render load (x1.25 = 95) pinned the top
 * OPP for whole sessions while FPS stayed engine-bound; only >=78% real
 * saturates now, so heavy frames are unchanged. Daily 95: the last OPP is a
 * battery cost. */
#define RFX_SAT_TO_MAX_GAMING_PCT	98
#define RFX_SAT_TO_MAX_DAILY_PCT	95

/* ---- Thermal emergency net. HW LMH (thermal_pressure) and the vendor HAL
 * (policy->max) are the real controllers; this is one hard net for when the
 * vendor engine is absent or asleep. One trip, one release, 7C apart, so it
 * cannot oscillate. ---- */
#define RFX_THERMAL_POLL_GAMING_MS	100
/* Idle poll: die time constant is ~seconds, so 5s detects runaway in <2
 * constants. Deferrable, so free in deep sleep. */
#define RFX_THERMAL_POLL_IDLE_MS	5000
/* Warm tier: charging + use can climb 3-5C between 5s polls. Daily only. */
#define RFX_THERMAL_POLL_WARM_MS	2000
#define RFX_TEMP_WARM_MC		70000
#define RFX_TEMP_EMERGENCY_MC		95000	/* junction; LMH acts far below */
#define RFX_TEMP_EMERGENCY_CLEAR_MC	88000
#define RFX_EMERGENCY_CAP_PCT		70

/* ---- Frame pacing ---- */
/* ~4 frames at 120fps: the at-risk frame plus settle. risk_high also clears on
 * window EXPIRY, so this is the re-arm period too -- shortening it raised the
 * excursion RATE on a cluster hovering near SATURATION. */
#define RFX_FRAME_BOOST_NS		(33 * NSEC_PER_MSEC)

/* Frame risk vs the effective ceiling: >90% servable = next frame at risk, must
 * fall under 75% before another window can arm. On the inflated demand scale,
 * so ~72%/60% real -- above Big's ordinary band, which 80/68 sat inside and
 * re-armed continuously. No arm dwell: it is mutually exclusive with the
 * edge-only "nothing to gain" test and delays a rescue inside 8.3ms. */
#define RFX_RISK_SATURATION_PCT		90
#define RFX_RISK_CLEAR_PCT		75

/* Frame boost ramp: instant rise, gentle decay to baseline floors. Also carries
 * the warmup release, so it must outlast a few frames. */
#define RFX_FRAME_BOOST_RAMP_DOWN_MS	60

/* Gaming warmup lifts floors to frame-boost level to cover spawn + asset load.
 * Adaptive: extends while demand stays >EXTEND_PCT up to MAX_NS, releases early
 * below RELEASE_PCT for RELEASE_NS. EXTEND_PCT is on the inflated scale -- 60
 * there (~48% real) extended unconditionally. MAX_NS stays short: the window is
 * anchored to the sysfs write, so a longer one pins every cluster through the
 * hottest phase (reverted twice). It feeds the frame-boost ramp, so its release
 * is a decay rather than a cliff. */
#define RFX_GAMING_WARMUP_NS		(300 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_MAX_NS	(600 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_EXTEND_PCT	90
#define RFX_GAMING_WARMUP_RELEASE_PCT	40
#define RFX_GAMING_WARMUP_RELEASE_NS	(100 * NSEC_PER_MSEC)

/* Gaming demand gate -- the only demand threshold in the gaming band. Below
 * GATE a cluster is idle: floor releases and no lift may arm; it participates
 * again above GATE_EXIT. Every lift reads the p->floor_gated latch, never
 * demand directly -- a bare compare is an undeadbanded second gate, and
 * filtered demand dithers across it every few frames.
 *
 * One gate for every role: a higher PRIME gate (45/55) assumed the top tier
 * only receives spill and dropped the render cluster to idle mid-session. Which
 * tier renders is a per-frame EAS decision the governor cannot see. */
#define RFX_G_FLOOR_GATE_PCT		25
#define RFX_G_FLOOR_GATE_EXIT_PCT	35

/* Floor for a gated (idle) cluster. Not fmin: from there a cluster must climb
 * the whole range when work lands, and the OPP transition plus rate gate turn
 * that into a visible hitch. Sits at the V/f knee. */
#define RFX_G_IDLE_FLOOR_PCT		38

/* Cluster cool-down band, hysteretic. Below ENTER the platform limiter is
 * taking capacity, so floors drop for relief and return at EXIT. The deadband
 * stops toggling as the HAL steps policy->max across an OPP boundary. */
#define RFX_G_COOL_ENTER_PCT		80
#define RFX_G_COOL_EXIT_PCT		85

/* Cooling floors, split by role. Do NOT collapse both to idle: boost_fl at 38%
 * makes the risk detector's `next_freq >= boost_fl` test short-circuit every
 * eval, so frame-boost never arms while throttled -- exactly when frames miss.
 *   STEADY - sustained-load relief floor, below the band but above idle.
 *   BOOST  - reduced but still armable. 72, not 66: at 66 a late frame during
 *            throttle is rescued from too low an OPP to make the deadline. */
#define RFX_G_COOL_STEADY_FLOOR_PCT	52
#define RFX_G_COOL_BOOST_FLOOR_PCT	72

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

/* ===================================================================== */
/* Global state                                                          */
/* ===================================================================== */

/* Master gaming switch, written by gaming_mode sysfs (Prime cluster only). */
static atomic_t esk_gaming = ATOMIC_INIT(0);

static inline bool esk_gaming_enabled(void)
{
	return atomic_read(&esk_gaming) != 0;
}

/* Last input event timestamp (daily touch boost). */
static atomic64_t esk_input_ts_ns = ATOMIC64_INIT(0);

/* Emergency thermal cap percent (100 = inactive). Latched with hysteresis. */
static atomic_t esk_emergency_cap_pct = ATOMIC_INIT(100);
/* Userspace-fed temperature fallback (milli-Celsius); 0 = unavailable. */
static atomic_t esk_temp_mc = ATOMIC_INIT(0);

/*
 * Frame-miss boost deadline (ns since boot), GLOBAL so every cluster reacts to
 * a dropped frame - not just Prime. A missed frame means SOME cluster was too
 * slow; since the render thread's placement is not known in-kernel, all of
 * Prime/Big/Little lift their floor together until this deadline.
 */
static atomic64_t esk_frame_boost_end_ns = ATOMIC64_INIT(0);

/* All live policies, so gaming-off can reset every cluster (not just Prime). */
static LIST_HEAD(esk_policy_list);
static DEFINE_SPINLOCK(esk_policy_list_lock);

/* ===================================================================== */
/* Data structures                                                       */
/* ===================================================================== */

struct esk_tunables {
	struct gov_attr_set attr_set;
	unsigned int rate_limit_us;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
	/*
	 * Prime-selective gaming boost (ESK): while gaming_mode is set, the
	 * prime policy holds this floor while the other tiers rest lower.
	 * Percent of the effective ceiling; 0 disables the override and falls
	 * back to RFX_G_PRIME_FLOOR_PCT.
	 */
	unsigned int prime_gaming_floor_pct;
};

struct esk_policy {
	struct cpufreq_policy *policy;
	struct esk_tunables *tunables;
	struct list_head tunables_hook;
	struct list_head gov_node;	/* on esk_policy_list */

	raw_spinlock_t update_lock;

	u64 last_upfreq_time;
	u64 last_downfreq_time;
	u64 last_eval_time;		/* stamped on every evaluation, not just commits */
	s64 freq_update_delay_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;

	unsigned int next_freq;
	unsigned int cached_raw_freq;	/* raw request behind the last commit */
	unsigned int pending_raw_freq;	/* raw request awaiting the rate gate */
	unsigned int max_seen;		/* high-water policy->max = unthrottled baseline */

	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool limits_changed;
	bool need_freq_update;

	bool is_prime;			/* PRIME band: this policy owns the topmost CPU */
	bool is_little;
	bool gaming_attr;		/* this policy hosts the gaming_mode node */

	unsigned int prev_upct;		/* daily ramp reference demand% */
	u64 prev_upct_ns;		/* when that reference was sampled */
	u64 ui_boost_end_ns;		/* daily: UI render-burst floor hold */
	u64 coldstart_boost_end_ns;	/* daily: cold-start burst boost */

	/* Frame boost ramp (smooth transition, not binary) */
	unsigned int frame_boost_ramp_pct;	/* current ramp level 0-100 */
	u64 frame_boost_ramp_last_ns;		/* previous ramp evaluation */

	u64 gaming_warmup_end_ns;	/* floor lift after gaming_mode=1 */
	u64 gaming_warmup_start_ns;	/* arm time — anchors the absolute cap */

	/*
	 * Cluster-wide smoothed util. Owned by the policy, not by the CPU that
	 * happened to run the hook: a shared policy commits one frequency, so a
	 * per-CPU EMA made the committed value depend on which CPU ticked last
	 * (visible as frequency jitter and micro-stutter).
	 */
	unsigned long filt_util;
	u64 last_ema_ns;			/* timestamp of last EMA update */

	bool risk_high;			/* frame-risk edge state */
	bool floor_gated;		/* gaming: floor released to idle, hysteretic */
	bool little_cap_lifted;		/* daily: sustained-load cap lift latch */
	bool big_cap_lifted;		/* daily: sustained-load cap lift for Big/Prime */
	bool thermal_cooling;		/* gaming: floors dropped to idle, hysteretic */

	/* adaptive warmup — early-release tracking */
	u64 warmup_low_demand_since_ns;	/* when demand first fell below release threshold */

};

struct esk_cpu {
	struct update_util_data update_util;
	struct esk_policy *esk_policy;
	unsigned int cpu;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;

	unsigned long util;
	unsigned long bwmin;
};

static DEFINE_PER_CPU(struct esk_cpu, esk_cpu);

static inline struct esk_tunables *to_esk_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct esk_tunables, attr_set);
}

static inline struct gov_attr_set *esk_to_gov_attr_set(struct kobject *kobj)
{
	return container_of(kobj, struct gov_attr_set, kobj);
}

/*
 * Cluster identification.
 *
 * MT6895 (and similar SoCs) expose THREE cpufreq policies: little (cap ~380),
 * big (cap 1024) and a separate Prime DVFS domain (cap 1024, its own
 * performance-domain in the DTS). Capacity alone cannot tell big from prime,
 * so the prime tier is keyed off policy identity: a policy whose related_cpus
 * contains the highest-numbered CPU in the system is the prime policy. On
 * two-tier SoCs (no separate prime domain) the top policy still classifies as
 * prime and the big tier is simply unused, preserving the two-cluster
 * behavior. This deliberately replaces upstream's esk_ntiers() distinct-
 * capacity count: on MT6895 big and prime share capacity 1024, so counting
 * distinct capacities reads two tiers and would move the top tier into the
 * BIG band, losing the PRIME band entirely (upstream's ntiers is also why
 * the gaming_mode node migrates to esk_hosts_gaming_attr(), which uses the
 * same topmost-CPU rule and is unaffected).
 */
static inline bool esk_cap_is_little(unsigned long cap)
{
	return cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool esk_cap_is_top(unsigned long cap)
{
	return cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
}

/* True when this policy owns the topmost CPU (the prime DVFS domain). */
static inline bool esk_policy_is_prime(const struct cpufreq_policy *policy)
{
	return cpumask_test_cpu(nr_cpu_ids - 1, policy->related_cpus);
}

/* fmax * pct / 100 */
static inline unsigned int esk_pct(unsigned int fmax, unsigned int pct)
{
	return (unsigned int)((u64)fmax * pct / 100);
}

/*
 * Effective ceiling: percent remaining, plus the unthrottled baseline it applies
 * to. Two throttle channels; reading only the first was the portability gap:
 *   thermal_pressure - cpufreq_cooling / LMH. Present on QCOM.
 *   policy->max      - vendor thermal HAL. The only channel on MTK, where
 *                      cpufreq_cooling never registers.
 * Measured against the high-water policy->max, not the live value: a statically
 * low baseline (MTK per-core OPP split) must read as no throttle, and folding
 * the live value in directly is the earlier double-clamped-daily-caps regression.
 */
static unsigned int esk_thermal_headroom_pct(struct esk_policy *p,
					     unsigned long max_cap,
					     unsigned int *baseline)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int press_pct = 100, clamp_pct = 100;
	unsigned int pmax = READ_ONCE(pol->max);
	unsigned long press;

	if (pmax > p->max_seen)
		p->max_seen = pmax;
	*baseline = p->max_seen ? p->max_seen : pol->cpuinfo.max_freq;

	press = arch_scale_thermal_pressure(cpumask_first(pol->related_cpus));
	if (max_cap) {
		if (press >= max_cap)
			press_pct = 0;
		else if (press)
			press_pct = (unsigned int)((u64)(max_cap - press) *
						   100 / max_cap);
	}

	if (pmax && pmax < *baseline)
		clamp_pct = (unsigned int)((u64)pmax * 100 / *baseline);

	return min(press_pct, clamp_pct);
}

/*
 * Elapsed ns between a stored @stamp and the current hook @time. Both are
 * rq_clock, but each CPU snapshots its own, so a sibling's stamp in a shared
 * policy can read microseconds AHEAD of ours -- unsigned subtraction turns that
 * into ~584 years and every `elapsed >= period` test fires early. Clamp the
 * backward case to 0. Direct `time < end_ns` tests need no such care.
 */
static inline u64 esk_elapsed(u64 time, u64 stamp)
{
	s64 delta = (s64)(time - stamp);

	return delta > 0 ? (u64)delta : 0;
}

/*
 * Timestamps compared against the util hook's @time must come from
 * sched_clock(), not ktime_get_ns(): @time is rq_clock, while CLOCK_MONOTONIC has
 * a different epoch and NTP rate correction, so mixing them can wrap (time - ts)
 * to a window that silently never opens. Only the thermal poller keeps ktime (it
 * compares against itself).
 */
static inline bool esk_input_active(u64 time)
{
	u64 ts = (u64)atomic64_read(&esk_input_ts_ns);

	return ts && esk_elapsed(time, ts) < RFX_INPUT_WINDOW_NS;
}

/* ===================================================================== */
/* Helpers                                                               */
/* ===================================================================== */

/*
 * Frame boost ramp: instant rise to 100 when a boost arms (zero-latency miss
 * recovery), then a linear decay over RFX_FRAME_BOOST_RAMP_DOWN_MS so the
 * floor slides back to baseline instead of stepping off a cliff.
 */
static unsigned int esk_update_frame_boost_ramp(struct esk_policy *p, bool boost_active, u64 time)
{
	u64 delta_ns;
	unsigned int step;

	if (boost_active) {
		p->frame_boost_ramp_pct = 100;
		p->frame_boost_ramp_last_ns = time;
		return 100;
	}

	if (p->frame_boost_ramp_pct == 0)
		return 0;

	if (!p->frame_boost_ramp_last_ns)
		p->frame_boost_ramp_last_ns = time;
	delta_ns = esk_elapsed(time, p->frame_boost_ramp_last_ns);

	step = (unsigned int)min_t(u64,
		(delta_ns * 100) / ((u64)RFX_FRAME_BOOST_RAMP_DOWN_MS * NSEC_PER_MSEC), 100);

	/*
	 * Advance by the time the step consumed, not to `time`. One ramp percent
	 * is 0.6ms but gaming updates arrive every ~250us, so the division floors
	 * to zero on most calls; advancing to `time` dropped that remainder and
	 * left a stale ramp level when two boost windows overlap.
	 */
	if (step > 0) {
		u64 consumed_ns = (u64)step *
			((u64)RFX_FRAME_BOOST_RAMP_DOWN_MS * NSEC_PER_MSEC) /
			100;
		p->frame_boost_ramp_last_ns += consumed_ns;
		p->frame_boost_ramp_pct -= min(p->frame_boost_ramp_pct, step);
	}

	return p->frame_boost_ramp_pct;
}

/* ===================================================================== */
/* Util smoothing                                                        */
/* ===================================================================== */

/*
 * Directional EMA: instant rise, time-normalised decay. The slow decay is the
 * anti-yoyo filter but must still reach the bottom, so the step scales with
 * elapsed time -- each period removes 1/RFX_EMA_GAMING_DIVISOR of the error.
 */
static unsigned long esk_ema(unsigned long old, unsigned long val, u64 time,
			     u64 *last_ns, bool gaming)
{
	u64 delta_ns;
	unsigned long diff;
	unsigned int steps;

	/*
	 * Seed, instant rise, or daily instant fall -- nothing pending, so the
	 * reference moves to now. (A daily gentle decay rode the peak of bursty
	 * light loads and pinned a high resting OPP through inter-frame dips.)
	 */
	if (!old || val >= old || !gaming) {
		*last_ns = time;
		return val;
	}

	/* Unseeded reference: worth one period, as an absolute stamp so the
	 * advance below stays in the clock's domain. */
	if (unlikely(!*last_ns))
		*last_ns = time - RFX_EMA_DECAY_PERIOD_NS;
	delta_ns = esk_elapsed(time, *last_ns);

	steps = (unsigned int)min_t(u64, delta_ns / RFX_EMA_DECAY_PERIOD_NS,
				    RFX_EMA_MAX_STEPS);
	if (!steps)
		return old;	/* sub-period: hold, keep the remainder */

	/*
	 * Advance by the periods consumed, not to @time: an off-period eval
	 * dropped its whole remainder, stretching the time constant so every
	 * latch released late. At the step cap the excess is discarded by
	 * design, so the reference goes to @time -- carrying a multi-second
	 * backlog forward would hold the cap engaged.
	 */
	if (steps < RFX_EMA_MAX_STEPS)
		*last_ns += (u64)steps * RFX_EMA_DECAY_PERIOD_NS;
	else
		*last_ns = time;

	while (steps--) {
		diff = old - val;
		if (!diff)
			break;
		old -= max_t(unsigned long, diff / RFX_EMA_GAMING_DIVISOR, 1);
	}

	return old;
}

/*
 * Request slightly more capacity than measured, so we land on an OPP with room
 * to spare instead of running pinned at 100% util. Gaming uses flat headroom (no
 * multiplicative variance); daily uses a tiered curve -- nothing at low util
 * (battery), more as util climbs (responsiveness).
 */
static unsigned long esk_apply_headroom(unsigned long util, unsigned long max_cap,
					bool gaming, bool little)
{
	unsigned int upct;

	if (!max_cap || util >= max_cap)
		return max_cap;

	upct = (unsigned int)(util * 100 / max_cap);
	if (upct >= (gaming ? RFX_SAT_TO_MAX_GAMING_PCT :
			      RFX_SAT_TO_MAX_DAILY_PCT))
		return max_cap;

	if (gaming)
		return min(util + (max_cap * RFX_HEADROOM_GAMING / 100), max_cap);

	if (little) {
		if (upct >= 65)
			return min(util + util * RFX_HEADROOM_DAILY_HIGH / 100, max_cap);
		if (upct >= 40)
			return min(util + util * RFX_HEADROOM_DAILY_MID / 100, max_cap);
		return util;
	}

	if (upct >= 70)
		return min(util + util * RFX_HEADROOM_DAILY_HIGH / 100, max_cap);
	if (upct >= 45)
		return min(util + util * RFX_HEADROOM_DAILY_MID / 100, max_cap);

	/* Below 45%: none. The 25% DVFS margin from the util getter already
	 * covers OPP granularity; the old 2% residual just held a higher
	 * voltage step through every idle evaluation. */
	return util;
}

/* ===================================================================== */
/* Thermal emergency clamp (final clamp)                                 */
/* ===================================================================== */

/*
 * Last clamp before OPP resolution. Flat and latched -- no walking, no
 * proportional target. Normal throttling is the platform's job (LMH via
 * thermal_pressure, thermal HAL via policy->max); this only fires at
 * RFX_TEMP_EMERGENCY_MC, which on a healthy device never happens.
 */
static unsigned int esk_thermal_clamp(unsigned int freq, unsigned int fmax)
{
	int pct = atomic_read(&esk_emergency_cap_pct);

	if (likely(pct >= 100))
		return freq;

	return min(freq, esk_pct(fmax, pct));
}

/* ===================================================================== */
/* Frame pacing                                                          */
/* ===================================================================== */

static inline bool esk_frame_boost_active(u64 time)
{
	u64 end = (u64)atomic64_read(&esk_frame_boost_end_ns);

	return end && time < end;
}

/*
 * Frame-risk detector. Arms one boost window when raw cluster demand crosses
 * RFX_RISK_SATURATION_PCT; demand must fall below RISK_CLEAR_PCT before another
 * can arm. Little excluded. Reads demand before headroom inflation.
 */
static void esk_frame_risk_check(struct esk_policy *p, unsigned int demand_pct,
				 unsigned int boost_fl, u64 time)
{
	if (likely(p->is_little))
		return;

	if (demand_pct < RFX_RISK_SATURATION_PCT) {
		/*
		 * Clear on demand < CLEAR OR window expiry. Without the expiry
		 * check, demand parked in (CLEAR, SATURATION) -- where a busy
		 * scene sits between frames -- latches risk_high forever after
		 * the first boost and blocks re-arm. This is the only re-arm
		 * path, which is what keeps one crossing to one window.
		 */
		if (demand_pct <= RFX_RISK_CLEAR_PCT ||
		    !esk_frame_boost_active(time))
			p->risk_high = false;
		return;
	}

	/*
	 * Re-arming while still saturated made the boost the steady state
	 * wherever sustained demand sits above SATURATION. Such a cluster is
	 * already at fmax via the saturation shortcut, so the boost adds no
	 * clock -- only heat on the clusters it pins.
	 */
	if (p->risk_high)
		return;

	/*
	 * Nothing to gain: already at/above the floor a boost would install, so
	 * arming cannot raise this OPP -- only pin the others. Beats a
	 * `policy->max < fmax` test, which disabled the detector while
	 * throttled, i.e. when frames miss.
	 */
	if (p->next_freq >= boost_fl)
		return;

	/*
	 * Only while the platform is taking capacity. The test above admits only
	 * a cluster that still has ceiling left -- which is never the tier that
	 * missed the frame: a saturated one is already at fceil via the
	 * saturation shortcut, and with up-rate 0 an unsaturated one is at its
	 * demand OPP within one 250us eval. So unthrottled the window adds no
	 * clock where the frame is late and only pins the other clusters, and
	 * that heat walks fceil down until the render tiers leave fmax (measured:
	 * a spill cluster armed ~93ms of lift, 33 window + 60 ramp, back to back
	 * for a whole session). Throttled, boost_fl is min'd to
	 * RFX_G_COOL_BOOST_FLOOR_PCT and the lift is real -- which is the case
	 * this detector was added for.
	 */
	if (!p->thermal_cooling)
		return;

	p->risk_high = true;
	atomic64_set(&esk_frame_boost_end_ns, time + RFX_FRAME_BOOST_NS);
}

/* ===================================================================== */
/* Frequency decision                                                    */
/* ===================================================================== */

/*
 * Pure-ish frequency selection from a (smoothed) util value. Order:
 *   1. headroom -> base freq from util/capacity
 *   2. profile shaping (gaming band + bounded slew OR daily caps/floors)
 *   3. thermal step clamp (final ceiling)
 *   4. resolve to a real OPP (cached to skip redundant table walks)
 */
static unsigned int esk_target_freq(struct esk_policy *p, unsigned long util,
				    unsigned long max_cap, u64 time, bool gaming)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int fmin = pol->cpuinfo.min_freq;
	bool little = p->is_little;
	bool prime = p->is_prime;
	unsigned int freq;
	unsigned long raw_util = util;	/* demand before headroom inflation */
	/* One ceiling for both bands. Every floor/cap/frame percent below is a
	 * percentage of THIS, so the shape slides down under a clamp instead of
	 * colliding with it (against hardware fmax a floor sits above the clamp,
	 * pins the cluster flat and makes the platform clamp harder). fmax is the
	 * unthrottled baseline, not cpuinfo.max_freq -- on MTK it sits lower. */
	unsigned int fmax;
	unsigned int fceil;
	unsigned int fceil_pct;

	if (unlikely(!pol->cpuinfo.max_freq || !max_cap))
		return pol->cur;

	fceil_pct = esk_thermal_headroom_pct(p, max_cap, &fmax);
	if (unlikely(!fmax))
		return pol->cur;
	fceil = esk_pct(fmax, fceil_pct);
	fceil = clamp(fceil, fmin, fmax);

	util = esk_apply_headroom(util, max_cap, gaming, little);

	/* arch capacity 1024 is defined against cpuinfo max, so demand->freq uses
	 * that; only the percentage shape below uses fmax. */
	freq = (unsigned int)((u64)pol->cpuinfo.max_freq * util / max_cap);
	freq = clamp(freq, fmin, fceil);

	if (gaming) {
		bool fboost_active, warmup_active;
		unsigned int fboost_ramp_pct;
		unsigned int fl, boost_fl, demand_pct;
		u64 down_step, slew_ns;

		/*
		 * Demand before esk_apply_headroom's inflation -- what the risk
		 * detector and floor gate judge against.
		 *
		 * CAVEAT, do NOT re-tune thresholds without reading this: raw_util
		 * is filt_util and esk_get_util_gki510 already applied the 25% DVFS
		 * margin, so demand_pct reads ~1.25x measured demand (saturating).
		 * Every threshold below was tuned WITH that skew -- skew and numbers
		 * are a matched pair, fix both together or neither.
		 */
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);

		warmup_active = p->gaming_warmup_end_ns && time < p->gaming_warmup_end_ns;

		/* Adaptive warmup: extend while Big/Prime demand holds above
		 * EXTEND_PCT (absolute cap MAX_NS from arm), release early below
		 * RELEASE_PCT for RELEASE_NS. */
		if (warmup_active && !little) {
			if (demand_pct >= RFX_GAMING_WARMUP_EXTEND_PCT) {
				u64 cap = p->gaming_warmup_start_ns +
					  RFX_GAMING_WARMUP_MAX_NS;
				u64 ext = time + RFX_EMA_DECAY_PERIOD_NS * 4;

				if (ext > cap)
					ext = cap;
				if (ext > p->gaming_warmup_end_ns)
					p->gaming_warmup_end_ns = ext;
				p->warmup_low_demand_since_ns = 0;
			} else if (demand_pct < RFX_GAMING_WARMUP_RELEASE_PCT) {
				if (!p->warmup_low_demand_since_ns)
					p->warmup_low_demand_since_ns = time;
				else if (esk_elapsed(time,
						p->warmup_low_demand_since_ns) >=
					 RFX_GAMING_WARMUP_RELEASE_NS)
					p->gaming_warmup_end_ns = time;
			} else {
				p->warmup_low_demand_since_ns = 0;
			}
			warmup_active = time < p->gaming_warmup_end_ns;
		}

		/* One parameterised band for all three clusters: the three copies
		 * this replaces differed only in their percentages, and each had
		 * to be fixed separately. */
		if (prime) {
			unsigned int pf = p->tunables->prime_gaming_floor_pct;

			/* ESK knob: 0 = fall back to the measured default. */
			if (pf > 100)
				pf = 100;
			fl = esk_pct(fceil, pf ? pf : RFX_G_PRIME_FLOOR_PCT);
			boost_fl = esk_pct(fceil, RFX_G_PRIME_FRAME_PCT);
		} else if (!little) {		/* Big: demand-tracked, uncapped */
			fl = esk_pct(fceil, RFX_G_BIG_FLOOR_PCT);
			boost_fl = esk_pct(fceil, RFX_G_BIG_FRAME_PCT);
		} else {			/* Little: compositor / audio / input */
			fl = esk_pct(fceil, RFX_G_LITTLE_FLOOR_PCT);
			boost_fl = esk_pct(fceil, RFX_G_LITTLE_FLOOR_BOOST_PCT);
		}

		/* Once the platform has taken capacity, holding floors defeats
		 * thermal relief and makes the HW limiter saw-tooth the clock. Drop
		 * floors; demand and up-rate stay intact. */
		if (fceil_pct < RFX_G_COOL_ENTER_PCT)
			p->thermal_cooling = true;
		else if (fceil_pct >= RFX_G_COOL_EXIT_PCT)
			p->thermal_cooling = false;

		if (p->thermal_cooling) {
			unsigned int steady = esk_pct(fceil,
						      RFX_G_COOL_STEADY_FLOOR_PCT);
			unsigned int boost = esk_pct(fceil,
						     RFX_G_COOL_BOOST_FLOOR_PCT);

			fl = min(fl, steady);
			boost_fl = min(boost_fl, boost);
		}

		/* Hand the detector the reachable (clamped) floor, not the nominal
		 * one, or it arms windows resolving to the clamp it already sits on. */
		esk_frame_risk_check(p, demand_pct,
				     esk_thermal_clamp(boost_fl, fceil), time);

		/*
		 * Bounded slew, measured from last commit (not last eval) so budget
		 * accumulates correctly. Window capped at the down-rate period, or
		 * budget grows while the gate blocks commits and the first accepted
		 * one dumps it as a cliff. Order-independent: floors only raise.
		 */
		slew_ns = esk_elapsed(time, max(p->last_upfreq_time,
						p->last_downfreq_time));
		slew_ns = min_t(u64, slew_ns,
				(u64)RFX_GAMING_DOWN_US * NSEC_PER_USEC);
		down_step = (u64)esk_pct(fceil, RFX_GAMING_DOWN_PCT_PER_MS) *
			    slew_ns / NSEC_PER_MSEC;
		if (down_step < fceil && p->next_freq > (unsigned int)down_step &&
		    freq < p->next_freq - (unsigned int)down_step)
			freq = p->next_freq - (unsigned int)down_step;

		fboost_active = esk_frame_boost_active(time);
		/* Warmup feeds the same ramp rather than holding a hard floor: at
		 * expiry the old branch stepped boost_fl -> fl in one eval, and that
		 * cliff landed as the first real frames arrived. */
		fboost_ramp_pct = esk_update_frame_boost_ramp(p,
					fboost_active || warmup_active, time);

		/*
		 * Idle latch: enter below GATE, leave only above GATE_EXIT.
		 * Role-independent -- any cluster may be the one rendering. Every
		 * lift below reads this, never demand_pct directly.
		 */
		if (demand_pct < RFX_G_FLOOR_GATE_PCT)
			p->floor_gated = true;
		else if (demand_pct >= RFX_G_FLOOR_GATE_EXIT_PCT)
			p->floor_gated = false;

		if (p->floor_gated)
			fl = esk_pct(fceil, RFX_G_IDLE_FLOOR_PCT);
		else if (fboost_ramp_pct > 0)
			fl = fl + (boost_fl - fl) * fboost_ramp_pct / 100;

		if (freq < fl)
			freq = fl;
	} else {
		bool ui_active, coldstart_active;
		unsigned int demand_pct;

		/* Raw demand, before headroom: post-headroom util is stepped by
		 * tier, so a crossing jumps the value with no load change. Same
		 * 1.25x skew as the gaming band; read that note before re-tuning. */
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);

		/*
		 * Cold-start: a 0->high demand jump arms aggressive floors.
		 * Touch-gated like the ramp below -- a launch always follows a tap,
		 * while a radio/RX wakeup has the identical signature (idle base,
		 * one big step) and was arming the burst floor on every one.
		 */
		if (esk_input_active(time) &&
		    demand_pct >= RFX_D_COLDSTART_DELTA_PCT &&
		    p->prev_upct <= RFX_D_COLDSTART_BASE_PCT)
			p->coldstart_boost_end_ns = time + RFX_D_COLDSTART_BOOST_NS;

		/* Render burst re-arms the UI window, touch-gated: autoplay decode
		 * spikes look identical to an animation ramp, so ungated it pinned
		 * the boost cap through passive watching. */
		if (esk_input_active(time) &&
		    demand_pct > p->prev_upct &&
		    demand_pct - p->prev_upct >= RFX_D_RAMP_DELTA_PCT)
			p->ui_boost_end_ns = time + RFX_D_UI_BOOST_NS;

		/*
		 * Refresh the reference on a fixed cadence and nothing else.
		 * Snapping it down on a fall made this an unbounded peak-to-trough
		 * comparator: any oscillating load re-armed at its own minimum.
		 * Against a fixed 16ms base the test means what it says.
		 */
		if (!p->prev_upct_ns ||
		    esk_elapsed(time, p->prev_upct_ns) >= RFX_D_RAMP_SAMPLE_NS) {
			p->prev_upct = demand_pct;
			p->prev_upct_ns = time;
		}

		coldstart_active = p->coldstart_boost_end_ns && time < p->coldstart_boost_end_ns;
		ui_active = esk_input_active(time) ||
			    (p->ui_boost_end_ns && time < p->ui_boost_end_ns);

		if (little) {
			unsigned int cap = ui_active ?
				esk_pct(fceil, RFX_D_LITTLE_BOOST_CAP_PCT) :
				esk_pct(fceil, RFX_D_LITTLE_CAP_PCT);

			/* Sustained heavy load relaxes the cap (to 80%, not fmax),
			 * else it strangles long multithread work once the touch
			 * and burst windows expire. Light use never gets here. */
			if (demand_pct >= RFX_D_LITTLE_LIFT_PCT)
				p->little_cap_lifted = true;
			else if (demand_pct <= RFX_D_LITTLE_DROP_PCT)
				p->little_cap_lifted = false;
			if (p->little_cap_lifted)
				cap = max(cap, esk_pct(fceil,
					RFX_D_LITTLE_SUSTAINED_CAP_PCT));

			if (freq > cap)
				freq = cap;

			/* Interaction floor, applied AFTER the cap so a lifted cap
			 * is never undercut and the floor never exceeds it. */
			if (ui_active) {
				unsigned int fl = min(cap,
					esk_pct(fceil, RFX_D_LITTLE_UI_FLOOR_PCT));

				if (freq < fl)
					freq = fl;
			}
		} else {
			/* Big/Prime daily cap, same model as Little: base cap off
			 * the top OPPs, touch/UI lift for burst, sustained latch for
			 * long foreground work, cold-start floor clamped to the cap. */
			unsigned int cap, base_cap_pct, boost_cap_pct;

			if (prime) {
				base_cap_pct = RFX_D_PRIME_CAP_PCT;
				boost_cap_pct = RFX_D_PRIME_BOOST_CAP_PCT;
			} else {
				base_cap_pct = RFX_D_BIG_CAP_PCT;
				boost_cap_pct = RFX_D_BIG_BOOST_CAP_PCT;
			}

			cap = ui_active ?
				esk_pct(fceil, boost_cap_pct) :
				esk_pct(fceil, base_cap_pct);

			/* Sustained-load latch (Big/Prime share one latch). */
			if (demand_pct >= RFX_D_BIG_LIFT_PCT)
				p->big_cap_lifted = true;
			else if (demand_pct <= RFX_D_BIG_DROP_PCT)
				p->big_cap_lifted = false;
			if (p->big_cap_lifted)
				cap = max(cap, esk_pct(fceil, prime ?
					RFX_D_PRIME_SUSTAINED_CAP_PCT :
					RFX_D_BIG_SUSTAINED_CAP_PCT));

			if (freq > cap)
				freq = cap;

			/* Cold-start burst floor, clamped to the cap. */
			if (coldstart_active) {
				unsigned int fl = esk_pct(fceil, prime ?
						RFX_D_PRIME_BURST_FLOOR_PCT :
						RFX_D_BIG_BURST_FLOOR_PCT);

				fl = min(fl, cap);
				if (freq < fl)
					freq = fl;
			}
		}
	}

	freq = esk_thermal_clamp(freq, fceil);
	freq = clamp(freq, fmin, fceil);

	/*
	 * The raw request becomes the cache key only once committed (see
	 * esk_commit_freq). Writing it here poisoned the cache whenever the rate
	 * gate rejected a commit: the next tick hit the cache, returned the stale
	 * next_freq, and lost the pending change until demand moved again.
	 */
	if (freq == p->cached_raw_freq && !p->need_freq_update)
		return p->next_freq;
	p->pending_raw_freq = freq;
	return cpufreq_driver_resolve_freq(pol, freq);
}

/* ===================================================================== */
/* IO-wait boost (unchanged behaviour from schedutil lineage)            */
/* ===================================================================== */

static bool esk_iowait_reset(struct esk_cpu *esk_c, u64 time, bool set)
{
	s64 delta_ns = time - esk_c->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	esk_c->iowait_boost = set ? IOWAIT_BOOST_MIN : 0;
	esk_c->iowait_boost_pending = set;
	return true;
}

static void esk_iowait_boost(struct esk_cpu *esk_c, u64 time, unsigned int flags)
{
	bool set = flags & SCHED_CPUFREQ_IOWAIT;
	unsigned long max_cap;
	unsigned int cap;

	/* Reset boost if the CPU has been idle long enough. */
	if (esk_c->iowait_boost && esk_iowait_reset(esk_c, time, set))
		return;

	/* Boost only tasks waking up after IO. */
	if (!set)
		return;

	/* Double at most once per boost consumption. */
	if (esk_c->iowait_boost_pending)
		return;
	esk_c->iowait_boost_pending = true;

	/*
	 * Per-cluster ceiling: Little modest (completions there are
	 * housekeeping), Big/Prime enough for the CPU side of a completion
	 * (sqlite, log, decompression) at the V/f knee.
	 *
	 * No gaming tier. A 60% one was the largest ungated lift in the gaming
	 * band -- it enters util BEFORE the 25% margin, so it pinned ~75% demand
	 * on any cluster taking streaming completions, above every gaming floor
	 * and blind to both the idle gate and the cooling band. Asset streaming is
	 * async and buffered, so that latency never bounded a frame.
	 */
	if (esk_c->iowait_boost) {
		max_cap = arch_scale_cpu_capacity(esk_c->cpu);
		cap = esk_cap_is_little(max_cap) ? SCHED_CAPACITY_SCALE / 6
						 : SCHED_CAPACITY_SCALE / 4;
		esk_c->iowait_boost = min_t(unsigned int,
					    esk_c->iowait_boost << 1, cap);
		return;
	}
	esk_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long esk_iowait_apply(struct esk_cpu *esk_c, u64 time,
				      unsigned long max_cap)
{
	/* Fast path: no boost active, skip all computation */
	if (likely(!esk_c->iowait_boost))
		return 0;
	if (esk_iowait_reset(esk_c, time, false))
		return 0;
	if (!esk_c->iowait_boost_pending) {
		esk_c->iowait_boost >>= 1;
		if (esk_c->iowait_boost < IOWAIT_BOOST_MIN) {
			esk_c->iowait_boost = 0;
			return 0;
		}
	}
	esk_c->iowait_boost_pending = false;
	return esk_c->iowait_boost * max_cap >> SCHED_CAPACITY_SHIFT;
}

static void esk_get_util(struct esk_cpu *esk_c, unsigned long boost)
{
	esk_get_util_gki510(esk_c->cpu, boost, &esk_c->util, &esk_c->bwmin);
}

static inline void esk_ignore_dl_rate_limit(struct esk_cpu *esk_c)
{
	if (esk_dl_bw_exceeded_gki510(esk_c->cpu, esk_c->bwmin))
		esk_c->esk_policy->need_freq_update = true;
}

/* ===================================================================== */
/* Rate limiting                                                         */
/* ===================================================================== */

/* Set the active down-rate-limit for this update (long while gaming). */
static inline void esk_set_down_delay(struct esk_policy *p, bool gaming)
{
	if (gaming)
		p->down_rate_delay_ns = (s64)RFX_GAMING_DOWN_US * NSEC_PER_USEC;
	else
		p->down_rate_delay_ns =
			(s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
}

/* up-rate-limit: ZERO while gaming, every cluster, no exception -- a nonzero
 * up-rate inside an 8.3ms frame budget is a frame-time tax, i.e. an FPS cap (a
 * 2ms gate on the fastest tier measured out at ~100fps). Tunable otherwise. */
static inline void esk_pol_up_delay(struct esk_policy *p, bool gaming)
{
	if (gaming)
		p->up_rate_delay_ns = 0;
	else
		p->up_rate_delay_ns =
			(s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
}

/*
 * Eval delay for this update. Set BEFORE esk_should_update_freq, so it depends
 * only on state known up front -- fine, since every reason for a sub-ms cadence
 * is known without util. Everything else uses its tunable.
 *
 * Every cluster gets the interaction rate: which tier a UI frame lands on is an
 * EAS decision the governor cannot see, and at the 3ms rest rate the top tier's
 * cap lift arrived a frame late. One lever with RFX_D_PRIME_BOOST_CAP_PCT.
 */
static inline void esk_set_eval_delay(struct esk_policy *p, bool gaming, u64 time)
{
	if (gaming)
		p->freq_update_delay_ns = (s64)RFX_FAST_RATE_US * NSEC_PER_USEC;
	else if (esk_input_active(time) ||
		 (p->ui_boost_end_ns && time < p->ui_boost_end_ns))
		p->freq_update_delay_ns = (s64)RFX_UI_RATE_US * NSEC_PER_USEC;
	else
		p->freq_update_delay_ns =
			(s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
}

/*
 * Evaluation gate. Measures from last_eval_time (stamped on every evaluation),
 * not from last commit — esk_commit_freq() skips stamping when freq is
 * unchanged, so commit-based gating is permanently open when gaming floors
 * pin the clock. That was the 114fps overhead.
 */
static bool esk_should_update_freq(struct esk_policy *p, u64 time)
{
	s64 delta;

	if (unlikely(!p || !p->policy))
		return false;
	if (!cpufreq_this_cpu_can_update(p->policy))
		return false;

	if (unlikely(READ_ONCE(p->limits_changed))) {
		WRITE_ONCE(p->limits_changed, false);
		p->need_freq_update = true;
		smp_mb();
		return true;
	}
	if (p->need_freq_update)
		return true;

	delta = (s64)(time - p->last_eval_time);
	return delta >= p->freq_update_delay_ns;
}

/* Commit next_freq subject to directional up/down rate limits. */
static bool esk_commit_freq(struct esk_policy *p, u64 time, unsigned int next_freq)
{
	s64 delta;

	if (p->need_freq_update) {
		p->need_freq_update = false;
		if (p->next_freq == next_freq)
			return false;
	} else if (p->next_freq == next_freq) {
		return false;
	}

	if (next_freq < p->next_freq) {
		delta = (s64)(time - p->last_downfreq_time);
		if (p->down_rate_delay_ns > 0 && delta < p->down_rate_delay_ns)
			return false;
		p->last_downfreq_time = time;
	} else {
		delta = (s64)(time - p->last_upfreq_time);
		if (p->up_rate_delay_ns > 0 && delta < p->up_rate_delay_ns)
			return false;
		p->last_upfreq_time = time;
	}

	/*
	 * Commit accepted: the raw request behind it is now the valid cache key.
	 * Promoting here (rather than in esk_target_freq) is what keeps a
	 * gate-rejected update from being silently dropped.
	 */
	p->cached_raw_freq = p->pending_raw_freq;
	p->next_freq = next_freq;
	return true;
}

/* ===================================================================== */
/* Update hooks                                                          */
/* ===================================================================== */

static unsigned int esk_next_freq(struct esk_cpu *esk_c, u64 time, bool gaming)
{
	struct esk_policy *p = esk_c->esk_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(esk_c->cpu);
	unsigned long max_util = 0;
	unsigned int j;

	/*
	 * Aggregate max util across the policy's CPUs, then filter once. The EMA
	 * lives on the policy, not the CPU that ran the hook: a shared policy
	 * commits one frequency, so a per-CPU filter let the committed value flip
	 * with whichever CPU ticked last -- jitter with no change in load.
	 */
	for_each_cpu(j, policy->cpus) {
		struct esk_cpu *jc = per_cpu_ptr(&esk_cpu, j);
		unsigned long jb, je;

		jb = esk_iowait_apply(jc, time, max_cap);
		esk_get_util(jc, jb);
		je = max(jc->util, jb);

		if (je > max_util)
			max_util = je;
	}

	p->filt_util = esk_ema(p->filt_util, max_util, time, &p->last_ema_ns,
			       gaming);

	esk_set_down_delay(p, gaming);
	esk_pol_up_delay(p, gaming);

	return esk_target_freq(p, p->filt_util, max_cap, time, gaming);
}

/*
 * One hook for every policy, single-CPU or shared. The separate single-CPU
 * variant this replaces skipped update_lock (upstream can, having no cross-CPU
 * writer) while esk_reset_all_policies() writes filt_util, max_seen and every
 * latch from a sysfs write on another CPU. A 1+3+4 part gives Prime a one-CPU
 * policy, so that race sat on the top tier. The loop below runs once there, so
 * the cost is one uncontended spinlock.
 */
static void esk_update(struct update_util_data *hook, u64 time,
		       unsigned int flags)
{
	struct esk_cpu *esk_c = container_of(hook, struct esk_cpu, update_util);
	struct esk_policy *p = esk_c->esk_policy;
	bool gaming = esk_gaming_enabled();
	unsigned long irqflags;
	unsigned int next_f;
	bool do_deferred = false;
	bool do_fast_switch = false;

	raw_spin_lock_irqsave(&p->update_lock, irqflags);

	esk_iowait_boost(esk_c, time, flags);
	esk_c->last_update = time;
	esk_ignore_dl_rate_limit(esk_c);
	esk_set_eval_delay(p, gaming, time);

	if (esk_should_update_freq(p, time)) {
		p->last_eval_time = time;
		next_f = esk_next_freq(esk_c, time, gaming);
		if (esk_commit_freq(p, time, next_f)) {
			if (p->policy->fast_switch_enabled) {
				do_fast_switch = true;
			} else {
				if (!p->work_in_progress) {
					p->work_in_progress = true;
					do_deferred = true;
				}
			}
		}
	}

	raw_spin_unlock_irqrestore(&p->update_lock, irqflags);

	if (do_fast_switch)
		cpufreq_driver_fast_switch(p->policy, p->next_freq);

	if (do_deferred)
		irq_work_queue(&p->irq_work);
}

static void esk_work(struct kthread_work *work)
{
	struct esk_policy *p = container_of(work, struct esk_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&p->update_lock, flags);
	freq = p->next_freq;
	p->work_in_progress = false;
	raw_spin_unlock_irqrestore(&p->update_lock, flags);

	mutex_lock(&p->work_lock);
	/*
	 * __cpufreq_driver_target, not cpufreq_driver_target: the latter takes
	 * policy->rwsem, and esk_limits() already runs holding that rwsem while it
	 * takes work_lock -- opposite order, so the pair deadlocks. Reachable on
	 * any driver without fast_switch (MTK), which is the live case there.
	 */
	__cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&p->work_lock);
}

static void esk_irq_work(struct irq_work *irq_work)
{
	struct esk_policy *p = container_of(irq_work, struct esk_policy, irq_work);

	kthread_queue_work(&p->worker, &p->work);
}

/* ===================================================================== */
/* Thermal poller (slow path, may sleep -> never in the util hook)       */
/* ===================================================================== */

#ifdef CONFIG_THERMAL
static struct thermal_zone_device *esk_tz;
static char esk_tz_name[THERMAL_NAME_LENGTH];
#endif
static struct delayed_work esk_thermal_work;

static void esk_thermal_fn(struct work_struct *w)
{
	int t_mc = 0;
	bool have = false;
	unsigned int delay_ms;

#ifdef CONFIG_THERMAL
	if (READ_ONCE(esk_tz) && !thermal_zone_get_temp(READ_ONCE(esk_tz), &t_mc))
		have = true;
#endif
	if (!have) {
		t_mc = atomic_read(&esk_temp_mc);
		if (t_mc > 0)
			have = true;
	}

	/* Latched net, 7C hysteresis: trip once, hold until the die cools,
	 * release once. The proportional relay it replaces re-derived a target
	 * every 50ms and fought both HW LMH and its own effect on temperature. */
	if (have) {
		if (atomic_read(&esk_emergency_cap_pct) >= 100) {
			if (t_mc >= RFX_TEMP_EMERGENCY_MC) {
				atomic_set(&esk_emergency_cap_pct,
					   RFX_EMERGENCY_CAP_PCT);
				pr_warn_ratelimited("esk: thermal emergency %d mC, cap %d%%\n",
						    t_mc, RFX_EMERGENCY_CAP_PCT);
			}
		} else if (t_mc <= RFX_TEMP_EMERGENCY_CLEAR_MC) {
			atomic_set(&esk_emergency_cap_pct, 100);
			pr_info("esk: thermal emergency cleared %d mC\n", t_mc);
		}
	} else {
		/* No source configured: the poll can never do anything, so stop
		 * re-arming rather than waking for the life of the boot. Both
		 * sysfs stores re-arm when a source appears. */
		atomic_set(&esk_emergency_cap_pct, 100);
		return;
	}


	if (esk_gaming_enabled())
		delay_ms = RFX_THERMAL_POLL_GAMING_MS;
	else if (t_mc >= RFX_TEMP_WARM_MC)
		delay_ms = RFX_THERMAL_POLL_WARM_MS;
	else
		delay_ms = RFX_THERMAL_POLL_IDLE_MS;
	queue_delayed_work(system_power_efficient_wq, &esk_thermal_work,
			   msecs_to_jiffies(delay_ms));
}

/* ===================================================================== */
/* Input handler (daily touch boost; off during gaming)                  */
/* ===================================================================== */

static void esk_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	if (esk_gaming_enabled())
		return;
	if (type == EV_ABS || type == EV_KEY)
		atomic64_set(&esk_input_ts_ns, sched_clock());
}

static int esk_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int err;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "esk";

	err = input_register_handle(handle);
	if (err)
		goto err_free;
	err = input_open_device(handle);
	if (err)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return err;
}

static void esk_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id esk_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
	},
	{ },
};

static struct input_handler esk_input_handler = {
	.event		= esk_input_event,
	.connect	= esk_input_connect,
	.disconnect	= esk_input_disconnect,
	.name		= "esk",
	.id_table	= esk_input_ids,
};

/* ===================================================================== */
/* sysfs                                                                 */
/* ===================================================================== */

static struct esk_tunables *esk_global_tunables;
static DEFINE_MUTEX(esk_global_tunables_lock);

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_esk_tunables(attr_set)->rate_limit_us);
}
static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct esk_tunables *t = to_esk_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->rate_limit_us = val;
	/*
	 * No need to push the new value into every policy here: every update
	 * calls esk_set_eval_delay() before esk_should_update_freq() reads
	 * freq_update_delay_ns, so the store below was overwritten before it
	 * was ever used.
	 */
	return count;
}
static struct governor_attr rate_limit_us = __ATTR_RW(rate_limit_us);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_esk_tunables(attr_set)->up_rate_limit_us);
}
static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct esk_tunables *t = to_esk_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->up_rate_limit_us = val;
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_esk_tunables(attr_set)->down_rate_limit_us);
}
static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct esk_tunables *t = to_esk_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->down_rate_limit_us = val;
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

/*
 * Clear every transient latch and window on one policy. Called on both profile
 * edges: neither profile's residue may shape the other, and each threshold is
 * hysteretic, so a latch left engaged survives until its far edge is crossed.
 * Caller holds p->update_lock.
 */
static void esk_reset_policy_locked(struct esk_policy *p)
{
	p->frame_boost_ramp_pct = 0;
	p->frame_boost_ramp_last_ns = 0;
	p->gaming_warmup_end_ns = 0;
	p->gaming_warmup_start_ns = 0;
	p->risk_high = false;
	p->thermal_cooling = false;
	p->floor_gated = false;
	p->warmup_low_demand_since_ns = 0;
	p->little_cap_lifted = false;
	p->big_cap_lifted = false;
	p->prev_upct = 0;
	p->prev_upct_ns = 0;
	p->ui_boost_end_ns = 0;
	p->coldstart_boost_end_ns = 0;
	p->need_freq_update = true;
}

/* Reset transient residue on every live policy (all clusters). */
static void esk_reset_all_policies(void)
{
	struct esk_policy *p;
	unsigned long flags, pflags;

	spin_lock_irqsave(&esk_policy_list_lock, flags);

	atomic64_set(&esk_frame_boost_end_ns, 0);

	list_for_each_entry(p, &esk_policy_list, gov_node) {
		raw_spin_lock_irqsave(&p->update_lock, pflags);
		esk_reset_policy_locked(p);
		/* Do not carry saturated gaming demand into the daily profile. */
		p->filt_util = 0;
		p->last_ema_ns = 0;
		raw_spin_unlock_irqrestore(&p->update_lock, pflags);
	}
	spin_unlock_irqrestore(&esk_policy_list_lock, flags);
}

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", esk_gaming_enabled());
}
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;

	atomic_set(&esk_gaming, val);

	if (!val) {
		esk_reset_all_policies();
		/* Drop the 100ms gaming thermal poll back to idle rate. */
		mod_delayed_work(system_power_efficient_wq, &esk_thermal_work,
				 msecs_to_jiffies(RFX_THERMAL_POLL_IDLE_MS));
	} else {
		struct esk_policy *p;
		unsigned long flags, pflags;
		u64 now = sched_clock();

		spin_lock_irqsave(&esk_policy_list_lock, flags);
		atomic64_set(&esk_frame_boost_end_ns, 0);
		list_for_each_entry(p, &esk_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			esk_reset_policy_locked(p);
			/* Warmup floor lift covers process spawn / asset load. */
			p->gaming_warmup_end_ns = now + RFX_GAMING_WARMUP_NS;
			p->gaming_warmup_start_ns = now;
			raw_spin_unlock_irqrestore(&p->update_lock, pflags);
		}
		spin_unlock_irqrestore(&esk_policy_list_lock, flags);

		/* Sample temperature sooner once gaming begins. */
		mod_delayed_work(system_power_efficient_wq, &esk_thermal_work,
				 msecs_to_jiffies(RFX_THERMAL_POLL_GAMING_MS));
	}
	return count;
}
static struct governor_attr gaming_mode = __ATTR_RW(gaming_mode);

/* ESK: prime-selective gaming floor (read-only knob on the prime policy). */
static ssize_t prime_gaming_floor_pct_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	return sprintf(buf, "%u\n",
		       to_esk_tunables(attr_set)->prime_gaming_floor_pct);
}

static ssize_t prime_gaming_floor_pct_store(struct gov_attr_set *attr_set,
					    const char *buf, size_t count)
{
	struct esk_tunables *tunables = to_esk_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	tunables->prime_gaming_floor_pct = val;
	return count;
}
static struct governor_attr prime_gaming_floor_pct =
	__ATTR_RW(prime_gaming_floor_pct);

static ssize_t temp_mc_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&esk_temp_mc));
}
static ssize_t temp_mc_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&esk_temp_mc, val);
	/* Re-arm: the poller stops itself while no source is configured. */
	if (val > 0)
		mod_delayed_work(system_power_efficient_wq, &esk_thermal_work, 0);
	return count;
}
static struct governor_attr temp_mc = __ATTR_RW(temp_mc);

static ssize_t thermal_zone_show(struct gov_attr_set *attr_set, char *buf)
{
#ifdef CONFIG_THERMAL
	return sprintf(buf, "%s\n", esk_tz_name[0] ? esk_tz_name : "(none)");
#else
	return sprintf(buf, "(no CONFIG_THERMAL)\n");
#endif
}
static ssize_t thermal_zone_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
#ifdef CONFIG_THERMAL
	struct thermal_zone_device *tz;
	char name[THERMAL_NAME_LENGTH];

	strscpy(name, buf, sizeof(name));
	strim(name);
	tz = thermal_zone_get_zone_by_name(name);
	if (IS_ERR(tz))
		return -EINVAL;
	WRITE_ONCE(esk_tz, tz);
	strscpy(esk_tz_name, name, sizeof(esk_tz_name));
	/* Re-arm: the poller stops itself while no source is configured. */
	mod_delayed_work(system_power_efficient_wq, &esk_thermal_work, 0);
	return count;
#else
	return -ENODEV;
#endif
}
static struct governor_attr thermal_zone = __ATTR_RW(thermal_zone);

static struct attribute *esk_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&temp_mc.attr,
	&thermal_zone.attr,
	NULL
};
ATTRIBUTE_GROUPS(esk);

static void esk_tunables_free(struct kobject *kobj)
{
	kfree(to_esk_tunables(esk_to_gov_attr_set(kobj)));
}

/*
 * One attr set shape for every cluster. gaming_mode/temp_mc/thermal_zone are
 * global state, so which policy hosts them does not matter -- but whether they
 * exist does: without CPUFREQ_HAVE_GOVERNOR_PER_POLICY there is a single attr
 * set from whichever policy initialised first (policy0 = Little), so a per-tier
 * ktype left the governor's only activation path missing on those devices.
 * gaming_mode is therefore added at start() rather than declared here -- see
 * esk_gaming_attr().
 */
static struct kobj_type esk_ktype = {
	.default_groups = esk_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = esk_tunables_free,
};

/*
 * Place the prime-cluster nodes (gaming_mode + the ESK prime_gaming_floor_pct
 * knob) on the top cluster only. They cannot live in esk_attrs[]: capacities
 * are unnormalized when the first policy's kobject is built, so an is_visible
 * test there would hide them on every cluster.
 *
 * Removal is confined to the per-policy case. With one shared kobject a
 * non-top policy stopping would otherwise delete the governor's only
 * activation path from under a running top cluster; that kobject's own
 * teardown removes the files anyway.
 */
static void esk_gaming_attr(struct esk_policy *p, bool want)
{
	struct kobject *kobj = &p->tunables->attr_set.kobj;

	if (want == p->gaming_attr)
		return;
	if (want) {
		bool ok;

		ok = !sysfs_add_file_to_group(kobj, &gaming_mode.attr, NULL) &&
		     !sysfs_add_file_to_group(kobj,
					      &prime_gaming_floor_pct.attr,
					      NULL);
		if (ok)
			p->gaming_attr = true;
		else
			sysfs_remove_file_from_group(kobj,
						     &prime_gaming_floor_pct.attr,
						     NULL);
	} else if (have_governor_per_policy()) {
		sysfs_remove_file_from_group(kobj, &gaming_mode.attr, NULL);
		sysfs_remove_file_from_group(kobj,
					     &prime_gaming_floor_pct.attr,
					     NULL);
		p->gaming_attr = false;
	}
}

/*
 * Top cluster by highest possible CPU, not by capacity. As the boot default
 * governor, start() for policy0 runs before the last policy exists, so
 * capacities still read 1024 everywhere and a capacity test claims the node on
 * Little too. CPU numbering ascends with capacity on every DynamIQ part,
 * exactly one policy owns the last possible CPU, and that holds from init --
 * no fixup pass once topology settles. related_cpus, not cpus: the latter
 * holds only the online members.
 */
static bool esk_hosts_gaming_attr(struct cpufreq_policy *policy)
{
	return cpumask_test_cpu(cpumask_last(cpu_possible_mask),
				policy->related_cpus);
}

static struct cpufreq_governor esk_gov;

/* ===================================================================== */
/* Allocation / kthread                                                  */
/* ===================================================================== */

static struct esk_policy *esk_policy_alloc(struct cpufreq_policy *policy)
{
	struct esk_policy *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;
	p->policy = policy;
	raw_spin_lock_init(&p->update_lock);
	INIT_LIST_HEAD(&p->gov_node);
	return p;
}

static void esk_policy_free(struct esk_policy *p)
{
	kfree(p);
}

/*
 * DVFS worker for the slow path (any driver without fast_switch -- notably
 * mediatek-cpufreq, which only has .target_index, so every commit on MTK goes
 * through here).
 *
 * SCHED_DEADLINE + SCHED_FLAG_SUGOV, not SCHED_FIFO: a DL task needs the clock
 * this worker is about to raise, so it must not be preemptible by one, and an
 * RT worker shares the rt_rq bandwidth a runaway vendor RT thread can throttle
 * -- a throttled DVFS worker holds the last committed OPP for the whole period.
 * The flag is the upstream escape hatch (dl_entity_is_special): DL class, fake
 * bandwidth, no admission control, still allowed to sleep.
 */
static int esk_kthread_create(struct esk_policy *p)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = p->policy;
	int ret;

	/* Both are only used on the deferred path, but initialise them before the
	 * fast-switch return anyway: three call sites currently remember the
	 * !fast_switch guard, and a zeroed mutex is a crash, not a warning. */
	init_irq_work(&p->irq_work, esk_irq_work);
	mutex_init(&p->work_lock);

	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&p->work, esk_work);
	kthread_init_worker(&p->worker);
	thread = kthread_create(kthread_worker_fn, &p->worker, "esk_gov/%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("esk: kthread create failed %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = esk_setattr_sugov_gki510(thread);
	if (ret) {
		kthread_stop(thread);
		pr_warn("esk: failed to set SCHED_DEADLINE\n");
		return ret;
	}

	p->thread = thread;
	/* Bind to the cluster only when DVFS must run on it. Where it need not,
	 * leave the worker unbound: pinning it to the very CPUs whose frequency
	 * it is about to raise delays the update when they are all busy. Never
	 * set_cpus_allowed_ptr() here -- on a DL task that lands in
	 * set_cpus_allowed_dl()->__dl_sub(), which is not special-cased. */
	if (!policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, policy->related_cpus);

	wake_up_process(thread);
	return 0;
}

static void esk_kthread_stop(struct esk_policy *p)
{
	if (p->policy->fast_switch_enabled)
		return;
	kthread_flush_worker(&p->worker);
	kthread_stop(p->thread);
	mutex_destroy(&p->work_lock);
}

static struct esk_tunables *esk_tunables_alloc(struct esk_policy *p)
{
	struct esk_tunables *t;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (t) {
		gov_attr_set_init(&t->attr_set, &p->tunables_hook);
		if (!have_governor_per_policy())
			esk_global_tunables = t;
	}
	return t;
}

static void esk_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		esk_global_tunables = NULL;
}

/* ===================================================================== */
/* Governor callbacks                                                    */
/* ===================================================================== */

static int esk_init(struct cpufreq_policy *policy)
{
	struct esk_policy *p;
	struct esk_tunables *t;
	unsigned long max_cap;
	int ret = 0;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	p = esk_policy_alloc(policy);
	if (!p) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = esk_kthread_create(p);
	if (ret)
		goto free_p;

	/* Provisional: picks the rate limits below. start() re-derives them
	 * once capacities are normalized (see esk_start()). */
	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	p->is_prime = esk_cap_is_top(max_cap) && esk_policy_is_prime(policy);
	p->is_little = esk_cap_is_little(max_cap);

	mutex_lock(&esk_global_tunables_lock);

	if (esk_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = p;
		p->tunables = esk_global_tunables;
		gov_attr_set_get(&esk_global_tunables->attr_set, &p->tunables_hook);
		goto out;
	}

	t = esk_tunables_alloc(p);
	if (!t) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	/* Default prime gaming floor (ESK knob; see prime_gaming_floor_pct). */
	t->prime_gaming_floor_pct = RFX_G_PRIME_FLOOR_PCT;

	if (p->is_little) {
		t->rate_limit_us = RFX_LITTLE_RATE_US;
		t->up_rate_limit_us = RFX_LITTLE_UP_US;
		t->down_rate_limit_us = RFX_LITTLE_DOWN_US;
	} else {
		t->rate_limit_us = RFX_BIG_RATE_US;
		t->up_rate_limit_us = RFX_BIG_UP_US;
		t->down_rate_limit_us = RFX_BIG_DOWN_US;
	}

	policy->governor_data = p;
	p->tunables = t;

	ret = kobject_init_and_add(&t->attr_set.kobj, &esk_ktype,
				   get_governor_parent_kobj(policy),
				   "%s", esk_gov.name);
	if (ret)
		goto fail;

out:
	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
	mutex_unlock(&esk_global_tunables_lock);
	return 0;

fail:
	kobject_put(&t->attr_set.kobj);
	policy->governor_data = NULL;
	esk_clear_global_tunables();
stop_kthread:
	esk_kthread_stop(p);
	mutex_unlock(&esk_global_tunables_lock);
free_p:
	esk_policy_free(p);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("esk: init failed error %d\n", ret);
	return ret;
}

static void esk_exit(struct cpufreq_policy *policy)
{
	struct esk_policy *p = policy->governor_data;
	struct esk_tunables *t = p->tunables;
	unsigned int count;

	mutex_lock(&esk_global_tunables_lock);
	count = gov_attr_set_put(&t->attr_set, &p->tunables_hook);
	policy->governor_data = NULL;
	if (!count) {
		esk_clear_global_tunables();
		atomic_set(&esk_gaming, 0);
	}
	mutex_unlock(&esk_global_tunables_lock);

	esk_kthread_stop(p);
	esk_policy_free(p);
	cpufreq_disable_fast_switch(policy);
}

static int esk_start(struct cpufreq_policy *policy)
{
	struct esk_policy *p = policy->governor_data;
	unsigned long flags, max_cap;
	unsigned int cpu;
	u64 now = sched_clock();

	/* Re-derive roles here, not just in init(): on the cpufreq-notifier
	 * topology path capacities are normalized only after the last policy is
	 * created, so an early init() sees 1024 everywhere (one tier, is_little
	 * never true). Roles are read per-eval, so refreshing at start suffices.
	 * is_prime additionally requires this policy to own the topmost CPU --
	 * capacity alone cannot separate big from prime on MT6895. */
	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	p->is_prime = esk_cap_is_top(max_cap) && esk_policy_is_prime(policy);
	p->is_little = esk_cap_is_little(max_cap);
	esk_gaming_attr(p, esk_hosts_gaming_attr(policy));

	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;

	p->last_upfreq_time = now;
	p->last_downfreq_time = now;
	p->last_eval_time = now;
	p->next_freq = policy->cur > 0 ? policy->cur : policy->cpuinfo.min_freq;
	p->cached_raw_freq = 0;
	p->pending_raw_freq = 0;
	/* Unthrottled baseline; only ratchets up. */
	p->max_seen = policy->max;
	p->work_in_progress = false;
	p->limits_changed = false;
	p->filt_util = 0;
	p->last_ema_ns = 0;

	/* Not yet on the policy list, so nothing can race the hook here. */
	esk_reset_policy_locked(p);
	p->need_freq_update = false;

	spin_lock_irqsave(&esk_policy_list_lock, flags);
	list_add(&p->gov_node, &esk_policy_list);
	spin_unlock_irqrestore(&esk_policy_list_lock, flags);

	for_each_cpu(cpu, policy->cpus) {
		struct esk_cpu *esk_c = per_cpu_ptr(&esk_cpu, cpu);

		memset(esk_c, 0, sizeof(*esk_c));
		esk_c->cpu = cpu;
		esk_c->esk_policy = p;
	}

	for_each_cpu(cpu, policy->cpus)
		cpufreq_add_update_util_hook(cpu,
			&per_cpu_ptr(&esk_cpu, cpu)->update_util, esk_update);
	return 0;
}

static void esk_stop(struct cpufreq_policy *policy)
{
	struct esk_policy *p = policy->governor_data;
	unsigned long flags;
	unsigned int cpu;

	esk_gaming_attr(p, false);

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	spin_lock_irqsave(&esk_policy_list_lock, flags);
	list_del(&p->gov_node);
	spin_unlock_irqrestore(&esk_policy_list_lock, flags);

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&p->irq_work);
		kthread_cancel_work_sync(&p->work);
	}
}

static void esk_limits(struct cpufreq_policy *policy)
{
	struct esk_policy *p = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&p->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&p->work_lock);
	}
	smp_wmb();
	WRITE_ONCE(p->limits_changed, true);
}

static struct cpufreq_governor esk_gov = {
	.name = CPUFREQ_ESK_NAME,
	.owner = THIS_MODULE,
	.flags = CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init = esk_init,
	.exit = esk_exit,
	.start = esk_start,
	.stop = esk_stop,
	.limits = esk_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ESK
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &esk_gov;
}
#endif

/* ===================================================================== */
/* gaming_mode ownership                                                 */
/* ===================================================================== */

/*
 * gaming_mode is USER-OWNED: nothing in this driver ever writes it, and there is
 * deliberately no PM/suspend auto-clear. Cost: a session left at 1 keeps the
 * gaming eval rate and idle floors across post-suspend wakes.
 */

/*
 * Self-check for the two time-domain helpers. Every recurring bug in this file
 * has been one of three things: an unsigned rq_clock delta (a sibling CPU's
 * stamp reads ahead, the subtraction wraps, and every window fires instantly), a
 * periodic decay advancing its reference to @time instead of by the periods it
 * consumed (the dropped remainder stretches the time constant), and an inverted
 * deadband. The first two are asserted here, the third by BUILD_BUG_ON below.
 */
static void __init esk_selfcheck(void)
{
	const u64 t = 1ULL << 40;
	struct esk_policy p = { };
	u64 ns;

	/* Rise is instant and re-seeds the reference. */
	ns = 0;
	WARN_ON(esk_ema(100, 200, t, &ns, true) != 200 || ns != t);

	/* Daily falls instantly; gaming holds inside one period. */
	ns = t;
	WARN_ON(esk_ema(200, 100, t, &ns, false) != 100);
	WARN_ON(esk_ema(1000, 0, t, &ns, true) != 1000);

	/* One period removes 1/DIVISOR of the error, reference advances by it. */
	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(esk_ema(1000, 0, t, &ns, true) !=
		1000 - 1000 / RFX_EMA_GAMING_DIVISOR || ns != t);

	/* Stamp from a sibling CPU reading ahead must decay nothing, not wrap. */
	ns = t + NSEC_PER_MSEC;
	WARN_ON(esk_ema(1000, 0, t, &ns, true) != 1000);

	/* Ramp: instant to 100, zero at RAMP_DOWN_MS, and a sub-percent step
	 * keeps its remainder instead of stalling at 100 forever. */
	WARN_ON(esk_update_frame_boost_ramp(&p, true, t) != 100);
	WARN_ON(esk_update_frame_boost_ramp(&p, false, t) != 100);
	WARN_ON(esk_update_frame_boost_ramp(&p, false,
			t + (u64)RFX_FRAME_BOOST_RAMP_DOWN_MS * NSEC_PER_MSEC));
	p.frame_boost_ramp_pct = 100;
	p.frame_boost_ramp_last_ns = t;
	WARN_ON(esk_update_frame_boost_ramp(&p, false, t + 250000) != 100 ||
		p.frame_boost_ramp_last_ns != t);
}

static int __init esk_gov_init(void)
{
	int ret;

	/* Deadbands: every hysteretic pair must have its exit above its entry,
	 * every floor at or below the boost it decays from, every daily floor at
	 * or below the cap that clamps it. An inversion here is a latch that can
	 * never release (or never engage) and is invisible at runtime. */
	BUILD_BUG_ON(RFX_G_FLOOR_GATE_PCT >= RFX_G_FLOOR_GATE_EXIT_PCT);
	BUILD_BUG_ON(RFX_G_COOL_ENTER_PCT >= RFX_G_COOL_EXIT_PCT);
	BUILD_BUG_ON(RFX_RISK_CLEAR_PCT >= RFX_RISK_SATURATION_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_DROP_PCT >= RFX_D_LITTLE_LIFT_PCT);
	BUILD_BUG_ON(RFX_D_BIG_DROP_PCT >= RFX_D_BIG_LIFT_PCT);
	BUILD_BUG_ON(RFX_TEMP_EMERGENCY_CLEAR_MC >= RFX_TEMP_EMERGENCY_MC);
	BUILD_BUG_ON(RFX_G_PRIME_FLOOR_PCT > RFX_G_PRIME_FRAME_PCT);
	BUILD_BUG_ON(RFX_G_BIG_FLOOR_PCT > RFX_G_BIG_FRAME_PCT);
	BUILD_BUG_ON(RFX_G_LITTLE_FLOOR_PCT > RFX_G_LITTLE_FLOOR_BOOST_PCT);
	BUILD_BUG_ON(RFX_G_IDLE_FLOOR_PCT > RFX_G_LITTLE_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_COOL_STEADY_FLOOR_PCT > RFX_G_COOL_BOOST_FLOOR_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_UI_FLOOR_PCT > RFX_D_LITTLE_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_CAP_PCT > RFX_D_LITTLE_BOOST_CAP_PCT);
	BUILD_BUG_ON(RFX_D_BIG_CAP_PCT > RFX_D_BIG_BOOST_CAP_PCT);
	BUILD_BUG_ON(RFX_D_BIG_SUSTAINED_CAP_PCT > RFX_D_BIG_BOOST_CAP_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_CAP_PCT > RFX_D_PRIME_BOOST_CAP_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_SUSTAINED_CAP_PCT > RFX_D_PRIME_BOOST_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_SUSTAINED_CAP_PCT > RFX_D_LITTLE_BOOST_CAP_PCT);
	BUILD_BUG_ON(RFX_D_BIG_BURST_FLOOR_PCT > RFX_D_BIG_CAP_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_BURST_FLOOR_PCT > RFX_D_PRIME_CAP_PCT);
	BUILD_BUG_ON(RFX_LITTLE_CAP_THRESHOLD >= RFX_PRIME_CAP_THRESHOLD);
	BUILD_BUG_ON(RFX_EMA_GAMING_DIVISOR < 1 || RFX_EMA_MAX_STEPS < 1);
	BUILD_BUG_ON(RFX_EMERGENCY_CAP_PCT >= 100);

	pr_info("ESK Governor v%s by %s\n", CPUFREQ_ESK_VERSION,
		CPUFREQ_ESK_AUTHOR);

	esk_selfcheck();

	INIT_DEFERRABLE_WORK(&esk_thermal_work, esk_thermal_fn);
	queue_delayed_work(system_power_efficient_wq, &esk_thermal_work,
			   msecs_to_jiffies(RFX_THERMAL_POLL_IDLE_MS));

	if (input_register_handler(&esk_input_handler))
		pr_warn("esk: input handler register failed (touch boost off)\n");

	ret = cpufreq_register_governor(&esk_gov);
	if (ret) {
		input_unregister_handler(&esk_input_handler);
		cancel_delayed_work_sync(&esk_thermal_work);
	}
	return ret;
}

static void __exit esk_gov_exit(void)
{
	cpufreq_unregister_governor(&esk_gov);
	input_unregister_handler(&esk_input_handler);
	cancel_delayed_work_sync(&esk_thermal_work);
}

module_init(esk_gov_init);
module_exit(esk_gov_exit);

MODULE_AUTHOR("Steambot12");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ESK CPUFreq Governor v2.2");
