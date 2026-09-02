// SPDX-License-Identifier: GPL-2.0
/*
 * ESK CPUFreq Governor v2.1 - Perfect Gaming & Thermal Edition
 * Based on schedutil — optimized for 120fps gaming & daily use
 *
 * Features:
 *   • Dual-Profile Operating Modes      	— Gaming (locked high-band) / Daily (power-efficient)
 *   • Tri-Cluster Topology Awareness    	— Independent tuning for Little / Big / Prime
 *   • Directional EMA Util Smoothing    	— Fast-rise / slow-decay anti-yoyo filter
 *   • Dynamic Capacity Headroom         	— Load-proportional OPP headroom allocation
 *   • Latching Thermal Emergency Net    	— Single hard trip, 7C hysteresis, no limit cycle
 *   • Thermal Zone Integration          	— Hardware sensor + userspace fallback
 *   • In-Kernel Frame-Risk Detection    	— Ceiling-relative saturation arming, no userspace feeder
 *   • Global Frame Boost with Ramp      	— All-cluster sync on dropped frames, smooth transition
 *   • Touch Input Responsiveness Boost  	— 280ms window: fast DVFS gate + Little interaction floor
 *   • UI Ramp-Assist / Render Burst     	— Sharp demand-rise detection for animations
 *   • Profile-Gated Evaluation Rate     	— Sub-ms DVFS only while gaming or interacting
 *   • Ceiling-Relative Daily Shaping    	— Daily caps/floors track policy->max, not fmax
 *   • Knee-Limited Sustained Little Cap 	— Long background work stays below top voltage step
 *   • Adaptive Floor (Idle/Busy)        	— Prime & Little dynamic floor with hysteresis
 *   • Directional Rate Limiting         	— Per-cluster up/down rate gates
 *   • Profile-Split IOWait Boost        	— Streaming ceiling while gaming, half scale daily
 *   • Deadline Bandwidth Awareness      	— DL task frequency bypass
 *   • Jank Reporting via Frame-Risk     	— In-kernel jank detection (no userspace feeder needed)
 *   • Deferred IRQ-Work Frequency Commit 	— Async non-fast-switch path
 *   • Global Policy State Reset         	— Clean gaming-off transition
 *   • GKI 5.10 Util Interface           	— esk_get_util_gki510 / esk_dl_bw_exceeded_gki510
 *   • Sustained Load Lock (v2.1)        	— Benchmark/throttle test frequency sustain via headroom band
 *   • Cluster-Wide Util Aggregation     	— Single shared EMA per policy (no per-CPU divergence)
 *   • Bounded Slew Control              	— Symmetric down-step limiting, no cliff bypass
 *   • Demand-Gated Gaming Floors        	— Idle clusters release, loaded clusters hold band
 *   • Raw-Demand Thresholds             	— Every trip point measured before headroom inflation
 *   • Full Util Aggregation (v2.2)      	— uclamp / RT / DL / IRQ visible to the governor
 *   • Time-Base-Stable Detectors (v2.2) 	— Ramp & slew measured in ns, immune to eval rate
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
#define CPUFREQ_ESK_VERSION  "2.1"
#define CPUFREQ_ESK_AUTHOR   "Templar Dev"

/* Core-sched util getter / deadline-bandwidth check (owned by core sched). */
extern void esk_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern bool esk_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);

/* ===================================================================== */
/* Tunable defaults (KMI-safe: plain #defines, no struct-layout changes). */
/* ===================================================================== */

/* Cluster identification by arch capacity. */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* Per-cluster rate limits (microseconds). up=0 means "scale up instantly".
 *
 * rate_limit_us is the EVALUATION gate: how often the governor re-reads util
 * for every CPU in the policy, refilters it and recomputes a target. It is the
 * governor's own CPU cost and it is paid on scheduler events, so it lands on
 * whichever CPU is already busy.
 *
 * These are DAILY values. The fast gate (RFX_FAST_RATE_US) covers gaming and
 * the whole interaction window; outside it every cluster, Little included,
 * uses its tunable. Little used to sit at an unconditional 500us "transition
 * rate" instead, which spent its budget in the one state that cannot benefit
 * from it - an idle cluster with nothing to re-evaluate - while interaction
 * got the SLOWER of the two paths. Deferring to the fast gate is both quicker
 * under the finger and three times cheaper at rest.
 */
#define RFX_LITTLE_RATE_US		2000
#define RFX_LITTLE_UP_US		200
#define RFX_LITTLE_DOWN_US		4000

#define RFX_BIG_RATE_US			1000
#define RFX_BIG_UP_US			0
#define RFX_BIG_DOWN_US			8000

#define RFX_PRIME_RATE_US		1000
#define RFX_PRIME_UP_US			0
#define RFX_PRIME_DOWN_US		8000

/*
 * Evaluation rate while gaming, or while the touch window is open. Frame pacing
 * and gesture tracking are the only two things on the device that genuinely
 * need sub-millisecond DVFS decisions, and both are bounded events rather than
 * a standing cost - the finger lifts, the game exits, and the gate relaxes.
 */
#define RFX_FAST_RATE_US		250

/*
 * Evaluation rate during a DAILY interaction window. Gaming needs 250us
 * because its frame-risk detector, global boost ramp and slew bound are all
 * state machines that must react within a frame. A daily scroll has none of
 * that: it needs the util rise seen promptly, and the signal it is watching is
 * PELT, whose ~32ms half-life moves about half a percent in 250us. Running the
 * governor four times per millisecond to observe that is pure overhead, paid
 * on whichever CPU is already busy, for the entire 280ms window after every
 * touch - which during normal use is most of the time the screen is on.
 *
 * 700us is still more than twice the daily tunable and loses nothing
 * measurable off the ramp, at well under half the sampling cost of 250us.
 */
#define RFX_UI_RATE_US			700

/*
 * Gaming down-rate-limit. This gate blocks EVERY downward commit, however
 * small, so it sets the granularity of the descent: the slew bound below is a
 * rate, and this gate turns that rate into discrete steps of
 * RFX_GAMING_DOWN_PCT_PER_MS * (this gate) percent each.
 *
 * The two MUST be chosen together. At the previous 10ms with a 3%/ms slew, one
 * step was 30% of the ceiling - the exact cliff the slew bound exists to
 * prevent, arriving once per 10ms instead of smoothly. 4ms is half a frame at
 * 120fps and pairs with 1%/ms for a 4% maximum step.
 */
#define RFX_GAMING_DOWN_US		4000

/*
 * Gaming frequency band, percent of the effective ceiling.
 *
 * v2.0 floors (90/80%) gave great initial fps but cooked the die within
 * minutes — thermal throttle collapsed sustain.  v2.1 cut to 62/60% and
 * fixed thermals but dropped ~2fps and doubled jank (118/0.4% → 116/1%)
 * because inter-frame dips pulled the clock down and the ramp back cost a
 * frame.  The previous midpoint (72/68%) restored initial fps but still
 * ran hot enough to throttle during sustained sessions (Test 2 jank 1%).
 *
 * 70/66% — the sweet spot between thermal budget and frame stability.
 * The gap between floor and frame target is what the clock hunts through
 * between frames: too wide (24pt at 66/90) and inter-frame dips pull the
 * OPP down, costing a frame on the ramp back; too narrow and the cluster
 * holds frame-floor voltage through idle gaps, eating thermal budget.
 *
 * 70/88 (Prime, 18pt gap) and 66/86 (Big, 20pt gap) keep the clock
 * within one OPP step of the frame target during normal gameplay, so the
 * ramp back from an inter-frame dip completes within the rate gate
 * period (4ms) instead of across a full frame. 4 points above the
 * previous floors costs ~2% more idle-cluster power but eliminates the
 * 0.4-1% jank from floor-to-frame hunting.
 *
 * Sustained-load headroom (15%) handles benchmark sustain separately;
 * floors are for frame pacing only.
 */
#define RFX_G_PRIME_FLOOR_PCT		70
#define RFX_G_PRIME_CAP_PCT		100
#define RFX_G_PRIME_FRAME_PCT		88
#define RFX_G_BIG_FLOOR_PCT		66
#define RFX_G_BIG_CAP_PCT		100
#define RFX_G_BIG_FRAME_PCT		86
/*
 * Little floor at 55%: the compositor, input pipeline and audio thread live
 * here during gameplay, and they are latency-critical at ~30-40% demand.
 * 50% was marginal — an inter-frame dip to 48% cleared the floor gate and
 * sent the cluster to idle floor, then the next compositor callback had to
 * climb back, costing the frame its first few milliseconds. 55% sits above
 * the typical compositor demand, so the floor holds through the gap.
 * Boost floor at 74% for frame-miss recovery on the input pipeline.
 */
#define RFX_G_LITTLE_CAP_PCT		100
#define RFX_G_LITTLE_FLOOR_PCT		55
#define RFX_G_LITTLE_FLOOR_BOOST_PCT	74

/*
 * Max downward slew, percent of the ceiling per millisecond of elapsed time.
 * Time-proportional, not per-update: the update rate varies with load (and the
 * rate gate suppresses updates entirely for stretches), so a per-update step
 * meant the descent speed depended on how often the hook happened to run
 * rather than on wall-clock.
 *
 * 1%/ms sheds a full range in ~100ms - twelve frames at 120fps. The previous
 * 3%/ms did it in 33ms, which sounds harmless until it is paired with the
 * commit gate: the slew budget accumulates between commits, so what actually
 * reaches the OPP is one step of (rate * gate) every gate period. At 3%/ms
 * with a 4ms gate that is a 12% drop in a single commit, mid-scene, which is
 * a frame-time spike by construction - the governor shedding a range faster
 * than PELT can tell it the demand came back. 1%/ms holds the step at 4% and
 * still banks thermal budget within two frames of a scene ending.
 */
#define RFX_GAMING_DOWN_PCT_PER_MS	1

/* ---- Daily frequency shaping, percent of the effective ceiling ---- */
/*
 * Little daily cap lowered from 70% to 65%. Video playback and browsing text
 * layout peak at ~40-50% of Little capacity; 70% left 20 points of headroom
 * the cluster never used, holding a higher OPP through every idle evaluation.
 * 65% still covers compositing and layout; the sustained-load latch lifts to
 * 85% for genuine heavy work, and the interaction cap at 78% handles scrolls.
 */
#define RFX_D_LITTLE_CAP_PCT		65
#define RFX_D_LITTLE_BOOST_CAP_PCT	78
/*
 * Interaction floor for Little, active only while the touch or UI-burst window
 * is open.
 *
 * The daily profile had a cap for Little and no floor at all, so the entire
 * touch response on the cluster that carries the input pipeline, the
 * compositor callbacks and the animator was a CAP LIFT from 70% to 80% - which
 * does exactly nothing, because scrolling does not ask for 70% of Little in
 * the first place. Meanwhile the util EMA decays every evaluation, and between
 * two frames of a scroll the cluster fell back toward fmin; each frame then
 * started its work at ~300MHz and waited for DVFS to catch up. That is the
 * scroll and pop-up jitter: not a shortage of peak clock, but the floor
 * arriving after the frame that needed it.
 *
 * 35% of the ceiling is just below the knee of the Little V/f curve —
 * enough to remove the lower half of the ramp without paying knee-voltage
 * for every 280ms touch window. Lowered from 38%: the 3-point difference
 * saves one voltage step through the most common interaction (scroll, tap)
 * and the rate gate covers the remaining ramp in one evaluation cycle.
 * Strictly window-scoped: no touch, no UI burst, no floor.
 */
#define RFX_D_LITTLE_UI_FLOOR_PCT	35
/*
 * Sustained-load cap for Little. Lowered from 85% to 80%: the knee of
 * the V/f curve on most Little clusters sits around 75-80%, so 80% keeps
 * the same throughput as 85% at measurably lower voltage. Media scans,
 * package syncs and index rebuilds are not urgent; saving one voltage
 * step through a 30-minute background job is material battery savings.
 * Work that overflows still migrates to Big.
 */
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	80
/*
 * Sustained load latch thresholds unchanged - the sustained cap lift is about
 * long background work (media scans, syncs), not burst responsiveness.
 */
#define RFX_D_LITTLE_LIFT_PCT		72	/* latch on: sustained heavy load */
#define RFX_D_LITTLE_DROP_PCT		35	/* latch off: back to housekeeping */
/*
 * Burst floors reduced from 52%/48% to 45%/42% for better battery life.
 * Cold-start detection now uses a more accurate raw-demand threshold (40%
 * demand jump from ≤10%), so the detector fires less often and more
 * accurately. Lower floors still cover real app launches and transitions:
 * the cold-start window is 200ms, which is enough that the governor sees
 * the real demand rise and tracks it upward via headroom before the floor
 * matters. The floor's job is only the first ~2ms until demand is visible.
 */
#define RFX_D_BIG_BURST_FLOOR_PCT	45
#define RFX_D_PRIME_BURST_FLOOR_PCT	42

/*
 * Daily burst detection: 12% delta catches smoother animations (scrolling,
 * sheets) while avoiding false positives from video/background work. Lowered
 * from 15% to improve scroll momentum and keyboard popup detection.
 */
#define RFX_D_RAMP_DELTA_PCT		12
/*
 * Cadence at which the ramp/cold-start reference sample is refreshed. The
 * deltas above are only meaningful against a FIXED time base. They used to be
 * measured against the previous evaluation, whose spacing is 250us, 500us,
 * 1000us or 1500us depending on cluster and touch state - so the same physical
 * ramp was split into more, smaller steps exactly when evaluation was fastest,
 * and the detector went blind under the finger, which is the one moment it
 * exists for. PELT cannot move 12 points in 500us; over ~16ms (one 60Hz frame)
 * it easily can. Downward movement is still tracked immediately, so the
 * reference is the low-water mark of the last frame rather than a stale peak
 * that would suppress the next detection.
 */
#define RFX_D_RAMP_SAMPLE_NS		(16 * NSEC_PER_MSEC)
/*
 * Cold start threshold lowered from 55% to 40% for better responsiveness.
 * Old 55% threshold missed moderate app switches and UI transitions, causing
 * visible lag. 40% catches real launches while avoiding noise from background
 * work and sensor callbacks.
 */
#define RFX_D_COLDSTART_DELTA_PCT	40
#define RFX_D_COLDSTART_BASE_PCT	10
/*
 * UI boost extended to 280ms to fully cover animations plus scroll momentum.
 * Modern UI animations (sheets, dialogs, transitions) run 150-200ms, but
 * momentum scrolling continues after finger lift. 280ms covers animation plus
 * tail without mid-scroll frequency drops that cause jitter.
 */
#define RFX_D_UI_BOOST_NS		(280 * NSEC_PER_MSEC)
/*
 * Cold start boost extended slightly from 180ms to 200ms for better app launch
 * coverage. Covers spawn + initial layout + first render without prolonged
 * idle-floor hold.
 */
#define RFX_D_COLDSTART_BOOST_NS	(200 * NSEC_PER_MSEC)

/*
 * Touch window extended to 280ms for keyboard popup and scroll momentum.
 * Keyboard appearance takes 180-220ms (animation + IME setup), and scroll
 * momentum continues 200-250ms after finger lift. 280ms covers both cases
 * without prolonged boost on short taps. Matches UI boost duration.
 */
#define RFX_INPUT_WINDOW_NS		(280 * NSEC_PER_MSEC)

/* ---- Util EMA (directional smoothing). Rise is instant, decay is time-normalised.
 *
 * The decay step is proportional to elapsed wall-clock time rather than being
 * a fixed shift per evaluation. This makes the filter's time constant
 * independent of the eval rate, which varies from 250us (gaming) to 1500us
 * (daily Little). Without normalisation the same /8 shift decayed 6x faster
 * per wall-second under gaming than under idle Little, and a mode switch
 * changed the decay speed discontinuously.
 *
 * RFX_EMA_DECAY_PERIOD_NS is the reference interval at which one eighth of
 * the error is removed — matching the previous gaming behaviour (250us eval,
 * shift 3). At 1500us eval the call now removes 6/8 of the error per call
 * (same wall-clock rate, fewer calls), instead of the old 1/8 (6x slower).
 */
#define RFX_EMA_DECAY_PERIOD_NS		250000	/* 250us: one gaming eval */

/* ---- Headroom (extra capacity above demand) percent ----
 *
 * These stack ON TOP of the 25% DVFS margin esk_get_util_gki510 already
 * applied. WALT uses zero extra headroom — the 25% alone is enough to land
 * on the right OPP. We keep a small residual (4%/2%) only because PELT's
 * slower decay means util lags real demand more than WALT's window, and the
 * trim from 6%/3% to 4%/2% drops total overhead from 32%/29% to 30%/27%,
 * saving one voltage step during browsing, video, and light multitask —
 * the entire active-drain difference between "WALT-like" and "not quite".
 * The saturation shortcut (RFX_SAT_TO_MAX_DAILY_PCT, 95%) still reaches fmax
 * when a frame genuinely needs it, so this only lowers the resting OPP.
 */
#define RFX_HEADROOM_DAILY_HIGH		4
#define RFX_HEADROOM_DAILY_MID		2
/*
 * Gaming headroom. The util getter already applies the standard 25% DVFS
 * margin; this is added on top so a saturating workload lands on an OPP with
 * room to spare. This - not a high floor - is what sustains max frequency
 * through a benchmark run, because it scales with demand instead of pinning.
 *
 * 12% gives total overhead of ~37% (with the 25% DVFS margin). The previous
 * 15% (40% total) locked fmax too easily — moderate gaming at 60-70% real
 * load inflated to 85%+ and sat at the highest OPP, eating thermal budget
 * that the sustained session then lacked. 12% pushes that lock-in from
 * ~59% real load to ~63%, so moderate scenes run one OPP step lower while
 * genuinely heavy load still reaches and holds fmax via the saturation
 * shortcut at 82%.
 */
#define RFX_HEADROOM_GAMING		12

/*
 * Util percent at which we stop interpolating and request fmax outright.
 * Gaming trips at 82% - earlier detection provides better frame protection
 * and benchmark sustain. With 15% headroom, real 82% demand reaches fmax
 * reliably. Daily keeps the conservative 95% - the last OPP is a battery cost.
 */
#define RFX_SAT_TO_MAX_GAMING_PCT	82
#define RFX_SAT_TO_MAX_DAILY_PCT	95

/* ---- Thermal emergency net ----
 *
 * Hardware LMH and the vendor thermal HAL already regulate this SoC, and they
 * do it by lowering policy->max - which the gaming band already follows via
 * fceil. The governor previously ran a SECOND controller on top: a 1%-per-6ms
 * relay walking toward a temperature-proportional target. Two controllers on
 * one plant, the fast one with no deadband, is a textbook limit cycle - that
 * relay is what drew the shark-tooth sustain curve, and its 74C/60% breakpoint
 * is what cost roughly a quarter of the throttle-test score.
 *
 * What remains is a single hard net for the case where the vendor engine is
 * absent or asleep. One trip, one release, 7C apart: it cannot oscillate,
 * because it cannot re-arm without the die genuinely cooling first.
 *
 * The idle poll rate matters for a different reason: this is a delayed work
 * that reads a thermal zone, and on most platforms that read costs an ADC or
 * SMEM round trip. Waking twice a second, forever, to check for a 95C junction
 * emergency that cannot occur on an idle die is pure standby drain. 2s is still
 * an order of magnitude faster than the die's thermal time constant, and the
 * work is deferrable, so on a sleeping device it costs nothing at all.
 */
#define RFX_THERMAL_POLL_GAMING_MS	100
#define RFX_THERMAL_POLL_IDLE_MS	3000
#define RFX_TEMP_EMERGENCY_MC		95000	/* junction; LMH acts far below */
#define RFX_TEMP_EMERGENCY_CLEAR_MC	88000
#define RFX_EMERGENCY_CAP_PCT		70

/* ---- Frame pacing ---- */
#define RFX_FRAME_BOOST_NS		(120 * NSEC_PER_MSEC)

/*
 * In-kernel frame-risk detection, measured against the effective ceiling.
 * Crossing 85% of servable capacity means the next frame is at risk; demand
 * must fall back under 75% before another window can arm.
 */
#define RFX_RISK_SATURATION_PCT		85
#define RFX_RISK_CLEAR_PCT		75

/* Frame boost ramp: instant rise, gentle decay back to baseline floors. */
#define RFX_FRAME_BOOST_RAMP_DOWN_MS	120

/*
 * Gaming warmup. Lifts floors to the frame-boost level (not the cap) for long
 * enough to cover process spawn and shader/asset load. Pinning all clusters at
 * 100% here was the launch-stutter cause: the heat blast tripped LMH within
 * the first second, so the session began already throttled. 700ms covers the
 * spawn burst; the remaining half-second of the old window was spent holding
 * three idle clusters at the boost floor while the game sat on a splash
 * screen, which is exactly the budget the first heavy scene then lacked.
 */
#define RFX_GAMING_WARMUP_NS		(700 * NSEC_PER_MSEC)

/*
 * Sustained load lock: util above ENTER for HOLD_NS marks a benchmark or
 * throttle test - steady saturation rather than frame-paced bursts. Its only
 * job is to silence the frame-risk detector: there are no frames to protect,
 * so arming boost windows on steady load just burns thermal budget. It does
 * not raise any floor - saturating load already reaches fmax via the headroom
 * path on its own.
 *
 * Set to 92% to avoid false positives during heavy gaming scenes while still
 * detecting benchmarks. Real gaming rarely sustains above 90% continuously.
 */
#define RFX_SUSTAINED_LOAD_ENTER_PCT	92
/*
 * Exit threshold raised from 60% to 78%: a 32-point gap kept the lock active
 * at 65% demand (heavy gaming, not a benchmark), muting the frame-risk
 * detector through scenes that genuinely drop frames. 14 points (92-78) is
 * wide enough that a benchmark oscillating near the boundary cannot flap, and
 * narrow enough that heavy gaming (70-85%) releases the lock and re-enables
 * frame-miss recovery.
 */
#define RFX_SUSTAINED_LOAD_EXIT_PCT	78
#define RFX_SUSTAINED_LOAD_HOLD_NS	(300 * NSEC_PER_MSEC)

/*
 * Gaming floor demand gate. Floors exist to stop a loaded cluster sagging
 * mid-frame; they are not meant to hold an EMPTY cluster at 66% of fmax. An
 * idle cluster burning band-floor voltage is pure heat, and heat is what makes
 * the sustain curve decay - so below this util the floor releases entirely and
 * normal DVFS applies. A frame boost or heavy scene overrides the gate.
 *
 * Raised from 25% to 30%: a cluster at 26-29% demand is not carrying frame
 * work (the follow gate at 35% already excludes it from frame boost), yet the
 * old gate held it at the band floor for nothing — pure thermal waste.  5
 * points buys meaningful thermal budget during sustained gaming without
 * affecting any cluster that is actually in the frame path.
 */
#define RFX_G_FLOOR_GATE_PCT		30

/*
 * Demand a cluster must show before it follows the GLOBAL frame boost up to
 * its frame floor (88-90% of the ceiling).
 *
 * The boost deadline is shared - a risk crossing on Big arms it for Prime and
 * Little too - and the only thing standing between that and "every cluster at
 * its frame floor for the whole session" was the 25% release gate. A cluster
 * sitting at 30% demand is not late for anything, but it cleared the gate and
 * was lifted to ~84% of fmax anyway, at full voltage, for as long as any other
 * cluster kept re-arming the window. During real gameplay that is continuous.
 * Three clusters holding a frame floor to protect one cluster's frame is the
 * entire difference between the target power envelope and a session that heats
 * until the thermal engine takes the clock back.
 *
 * 35% separates "carrying part of the frame" from "ticking over". Below it the
 * baseline band floor still applies, so nothing falls off a cliff; the cluster
 * simply stops paying frame-floor voltage for a frame it is not in.
 *
 * It sits low deliberately. The first value tried here was 45%, which read as
 * conservative and was in fact a jank source: the cluster carrying the input
 * pipeline, the compositor callbacks and the audio thread is Little, and that
 * work is latency-critical while being genuinely light - it lives around 30-40%
 * of Little's capacity during gameplay and essentially never reaches 45%. So
 * the one cluster that is always in the frame was the one cluster excluded from
 * the frame boost. Demand is a proxy for participation and a poor one at the
 * low end; 35% keeps a truly idle cluster (which the 25% gate has already
 * released) out of the boost without excluding a busy compositor from it.
 */
#define RFX_G_BOOST_FOLLOW_PCT		35

/*
 * Floor for a gated (idle) cluster. Not zero: a released cluster that falls to
 * fmin has to climb the whole range when work lands on it, and the OPP
 * transition plus the rate gate turn that climb into the millisecond-scale
 * hitch seen when a scope opens or a scene changes - the frequency trace shows
 * the jump to maximum arriving AFTER the late frame, because it is a reaction
 * to it. A third of the ceiling costs almost nothing on an empty cluster (the
 * 38% of the ceiling sits at the knee of the V/f curve. Raised from 35%:
 * the 2-point difference costs almost nothing in power (the voltage curve
 * is flat at the bottom) but removes one additional OPP step from the
 * recovery ramp when work lands on a gated cluster mid-scene-change.
 */
#define RFX_G_IDLE_FLOOR_PCT		38

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
	unsigned int gaming_mode;
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

	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool limits_changed;
	bool need_freq_update;

	bool is_prime;			/* this policy is the Prime cluster */
	bool is_little;

	unsigned int prev_upct;		/* daily ramp reference demand% */
	u64 prev_upct_ns;		/* when that reference was sampled */
	u64 ui_boost_end_ns;		/* daily: UI render-burst floor hold */
	u64 coldstart_boost_end_ns;	/* daily: cold-start burst boost */

	/* Frame boost ramp (smooth transition, not binary) */
	unsigned int frame_boost_ramp_pct;	/* current ramp level 0-100 */
	u64 frame_boost_ramp_last_ns;		/* previous ramp evaluation */

	u64 gaming_warmup_end_ns;	/* floor lift after gaming_mode=1 */

	/*
	 * Cluster-wide smoothed util. Owned by the policy, not by the CPU that
	 * happened to run the hook: a shared policy commits one frequency, so a
	 * per-CPU EMA made the committed value depend on which CPU ticked last
	 * (visible as frequency jitter and micro-stutter).
	 */
	unsigned long filt_util;
	u64 last_ema_ns;			/* timestamp of last EMA update */

	bool risk_high;			/* frame-risk edge state */
	bool little_cap_lifted;		/* daily: sustained-load cap lift latch */

	/* Sustained load lock (benchmark / throttle test) */
	u64 sustained_since_ns;		/* first tick above ENTER, 0 = not counting */
	bool sustained_locked;
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
 * Compared against arch_scale_cpu_capacity(), which normalises the BIGGEST CPU
 * in the system to 1024. So on a two-cluster SoC (MediaTek 2+6 / 4+4) the big
 * cluster is 1024 and classifies as "prime" - it is the fastest tier present,
 * which is what these bands and the sysfs layout below actually mean. Do not
 * "fix" this by requiring a third distinct tier: esk_prime_ktype is the only
 * attribute set carrying gaming_mode, thermal_zone, temp_mc and the frame
 * nodes, so a policy set with no prime member has no gaming_mode node at all
 * and the mode becomes unreachable on every two-cluster device. The prime and
 * big rate limits are identical anyway (1000/0/8000); the only band difference
 * is 2 points of floor.
 */
static inline bool esk_cap_is_little(unsigned long cap)
{
	return cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool esk_cap_is_prime(unsigned long cap)
{
	return cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
}

/* fmax * pct / 100 */
static inline unsigned int esk_pct(unsigned int fmax, unsigned int pct)
{
	return (unsigned int)((u64)fmax * pct / 100);
}

/*
 * Thermal headroom for this policy, 0..100. 100 means nothing is holding the
 * cluster below its hardware ceiling.
 *
 * Two channels, because platforms do not agree on which one to use:
 *
 *   policy->max  - lowered through freq_qos by cpufreq_cooling, by a vendor
 *                  thermal driver, or by a userspace HAL writing
 *                  scaling_max_freq. This is what MediaTek and most AOSP /
 *                  OOS thermal stacks actually drive.
 *   thermal      - arch_scale_thermal_pressure(), the capacity a platform has
 *   pressure       taken away. Set by cpufreq_cooling, and by vendor limit
 *                  drivers (QCOM LMH) that throttle the clock in hardware
 *                  WITHOUT ever touching policy->max, so the policy still
 *                  claims a ceiling the silicon will not deliver.
 *
 * min() of the two is not a double count. cpufreq_cooling derives both from
 * the same requested frequency - policy->max becomes `frequency`, and the
 * pressure it publishes is max_cap - frequency * max_cap / cpuinfo.max_freq,
 * so (max_cap - pressure) / max_cap is that same frequency / cpuinfo.max_freq
 * ratio. When both channels are live they agree exactly and min() is a no-op;
 * when only one is live, min() is the only way to see it. Reading just one is
 * what made the governor's behaviour depend on the ROM and the SoC.
 */
static unsigned int esk_thermal_headroom_pct(struct cpufreq_policy *pol,
					     unsigned long max_cap)
{
	unsigned int fmax = pol->cpuinfo.max_freq;
	unsigned int pmax = READ_ONCE(pol->max);
	unsigned int qos_pct = 100, press_pct = 100;
	unsigned long press;

	if (fmax && pmax && pmax < fmax)
		qos_pct = (unsigned int)((u64)pmax * 100 / fmax);

	press = arch_scale_thermal_pressure(cpumask_first(pol->related_cpus));
	if (max_cap) {
		if (press >= max_cap)
			press_pct = 0;
		else if (press)
			press_pct = (unsigned int)((u64)(max_cap - press) *
						   100 / max_cap);
	}

	return min(qos_pct, press_pct);
}

/*
 * Every timestamp compared against the util hook's @time must come from
 * sched_clock(), never ktime_get_ns(). @time is rq_clock(rq), which is
 * sched_clock-derived; CLOCK_MONOTONIC is a different time base with its own
 * epoch and NTP rate correction, so mixing them makes (time - ts) meaningless
 * and, when the stamp reads ahead of the hook, wrap to a huge unsigned value -
 * a window that silently never opens. Touch boost, gaming warmup and the rate
 * gates all live in the sched_clock domain for that reason. Only the thermal
 * poller keeps ktime, and only because it compares against itself.
 */
static inline bool esk_input_active(u64 time)
{
	u64 ts = (u64)atomic64_read(&esk_input_ts_ns);

	return ts && (time - ts) < RFX_INPUT_WINDOW_NS;
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
	delta_ns = time - p->frame_boost_ramp_last_ns;

	step = (unsigned int)min_t(u64,
		(delta_ns * 100) / ((u64)RFX_FRAME_BOOST_RAMP_DOWN_MS * NSEC_PER_MSEC), 100);

	/*
	 * Consume only the elapsed time that actually produced a step. One ramp
	 * percent is 1.2ms of decay, but gaming updates arrive every ~250us, so
	 * this division floors to zero on nearly every call. Accumulate the
	 * sub-step remainder by NOT advancing the timestamp until we actually
	 * have a step to consume. This lets delta_ns grow until it crosses the
	 * 1.2ms boundary.
	 */
	if (step > 0) {
		p->frame_boost_ramp_last_ns = time;
		p->frame_boost_ramp_pct -= min(p->frame_boost_ramp_pct, step);
	}

	return p->frame_boost_ramp_pct;
}

/*
 * Sustained load lock. Steady saturation for HOLD_NS means a benchmark or
 * throttle test rather than frame-paced game load. Latches p->sustained_locked,
 * which silences the frame-risk detector (steady saturation is not a frame
 * miss). ENTER/EXIT hysteresis keeps it from flapping mid-run.
 */
static void esk_update_sustained_lock(struct esk_policy *p, unsigned int upct, u64 time)
{
	if (upct >= RFX_SUSTAINED_LOAD_ENTER_PCT) {
		if (!p->sustained_since_ns)
			p->sustained_since_ns = time;
		else if (time - p->sustained_since_ns >= RFX_SUSTAINED_LOAD_HOLD_NS)
			p->sustained_locked = true;
	} else if (upct <= RFX_SUSTAINED_LOAD_EXIT_PCT) {
		p->sustained_since_ns = 0;
		p->sustained_locked = false;
	}
}

/* ===================================================================== */
/* Util smoothing                                                        */
/* ===================================================================== */

/*
 * Directional EMA: instant rise, time-normalised decay. The slow decay is the
 * anti-yoyo filter — it stops the clock chasing micro-dips between frames —
 * but it must still reach the bottom, so the step scales with elapsed time.
 *
 * At the reference period (250us) this removes 1/8 of the error, matching the
 * old shift-3 gaming path. At 1500us (daily Little) it removes 6/8 in one
 * call — the same wall-clock rate, not 6x slower. Capped at the full diff so
 * a long idle gap converges in one step.
 */
static unsigned long esk_ema(unsigned long old, unsigned long val,
			     u64 delta_ns, bool gaming)
{
	unsigned long diff, step;
	unsigned int steps;

	if (!old)
		return val;
	if (val >= old)
		return val;	/* instant rise */

	/*
	 * Daily (gaming_mode=0): WALT-like fast fall. The anti-yoyo slow decay
	 * below exists only to keep the clock from chasing micro-dips BETWEEN
	 * FRAMES - a gaming concern. In daily use there are no frames to pace,
	 * and holding util above real demand for ~32ms of PELT half-life plus
	 * the EMA tail is exactly the resting-voltage drain WALT avoids by
	 * decaying its window fast. So daily follows demand down immediately;
	 * the rate gate (rate_limit_us / down_rate_limit_us) still bounds how
	 * often that lower request is committed, so this cannot cause churn.
	 */
	if (!gaming)
		return val;

	diff = old - val;
	/*
	 * steps = how many reference periods elapsed (integer division floors).
	 * Capped at 8 so that any gap >= 2ms converges fully in one call.
	 *
	 * Do NOT force steps=1 when delta_ns < one period: the hook re-enters
	 * faster than 250us on need_freq_update / limits_changed bypasses, and
	 * giving a sub-period call the same weight as a full period makes the
	 * filter decay faster than intended during burst re-evaluations —
	 * bleeding util between frames and causing the governor to drop
	 * frequency mid-scene, then climb back one frame late (= jank).
	 */
	steps = (unsigned int)min_t(u64, delta_ns / RFX_EMA_DECAY_PERIOD_NS, 8);
	if (!steps)
		return old;	/* sub-period: hold, don't decay */
	step = diff * steps / 8;
	if (!step)
		step = 1;	/* avoid stall at small diff */
	return old - step;
}

/*
 * Headroom: request slightly more capacity than measured so we land on an OPP
 * with room to spare (avoids running pinned at 100% util, which is both a
 * latency and a load-percent problem). Gaming uses fixed constant headroom
 * (eliminates multiplicative variance); daily uses a tiered curve that adds
 * little at low util (battery) and more as util climbs (responsiveness).
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

	/* Below 45%: no headroom. WALT adds nothing at low util, and the 25%
	 * DVFS margin from the util getter already covers OPP granularity.
	 * The old 2% residual held a higher voltage step through every idle
	 * evaluation — pure standby drain with no responsiveness benefit. */
	return util;
}

/* ===================================================================== */
/* Thermal emergency clamp (final clamp)                                 */
/* ===================================================================== */

/*
 * Last clamp before OPP resolution. A flat, latched cap - no walking, no
 * proportional target, no per-cluster stepping. Normal throttling is the
 * platform's job (LMH / thermal HAL lower policy->max, and the gaming band
 * tracks that through fceil); this only fires if the die reaches
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
 * RFX_RISK_SATURATION_PCT. Demand must fall below RISK_CLEAR_PCT (or the
 * boost window must expire) before another can arm. Little excluded.
 * Measures raw demand (before headroom inflation).
 */
static void esk_frame_risk_check(struct esk_policy *p, unsigned int demand_pct,
				 unsigned int boost_fl, u64 time)
{
	if (likely(p->is_little))
		return;

	/* Steady saturation is a benchmark, not a frame miss - no boost. */
	if (p->sustained_locked) {
		p->risk_high = false;
		return;
	}

	if (demand_pct < RFX_RISK_SATURATION_PCT) {
		/*
		 * Clear risk_high on demand < clear OR boost window expired.
		 * Without expiry check, demand in (CLEAR, SATURATION) latches
		 * risk_high forever, killing future re-arms.
		 */
		if (demand_pct <= RFX_RISK_CLEAR_PCT ||
		    !esk_frame_boost_active(time))
			p->risk_high = false;
		return;
	}

	if (p->risk_high)
		return;

	/*
	 * Nothing to gain: this cluster is already committed at or above the
	 * floor a boost would install, so arming a window cannot raise a
	 * single OPP - it would only hold every OTHER cluster pinned, and feed
	 * a throttle that saturation while clamped is resolved by waiting out,
	 * not by boosting into.
	 *
	 * This replaces a `policy->max < cpuinfo.max_freq` test, which disabled
	 * the detector entirely for as long as anything held the policy below
	 * the hardware maximum - the thermal HAL during a throttled session,
	 * or a userspace performance daemon parking scaling_max_freq. Frames
	 * are missed precisely while throttled, so the frame-miss recovery
	 * path was switched off in the exact window it exists for; that is
	 * where the min-fps floor and the jank rate come from. Comparing the
	 * committed frequency against the reachable boost floor asks the
	 * question that actually matters - "is there clock left to win?" - and
	 * is independent of how the clamp got there.
	 */
	if (p->next_freq >= boost_fl)
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
	unsigned int fmax = pol->cpuinfo.max_freq;
	unsigned int fmin = pol->cpuinfo.min_freq;
	bool little = esk_cap_is_little(max_cap);
	bool prime = esk_cap_is_prime(max_cap);
	unsigned int freq;
	unsigned long raw_util = util;	/* demand before headroom inflation */
	/*
	 * One ceiling for both bands, honouring every throttle channel (see
	 * esk_thermal_headroom_pct). Every floor, cap and frame percentage
	 * below is a percentage of THIS, so the whole shape slides down under
	 * a clamp instead of colliding with it: load keeps breathing, the die
	 * cools, the clamp lifts. A floor computed against hardware fmax sits
	 * above the clamp, pins the cluster flat at the limit with no
	 * demand-following left, and the platform answers by clamping harder.
	 * That ratchet is what "heats even with a cooler" looks like from the
	 * outside, and why it only showed up on ROMs and SoCs whose throttle
	 * channel this governor was not reading.
	 */
	unsigned int fceil;

	if (unlikely(!fmax || !max_cap))
		return pol->cur;

	fceil = esk_pct(fmax, esk_thermal_headroom_pct(pol, max_cap));
	fceil = clamp(fceil, fmin, fmax);

	util = esk_apply_headroom(util, max_cap, gaming, little);

	freq = (unsigned int)((u64)fmax * util / max_cap);
	freq = clamp(freq, fmin, fceil);

	if (gaming) {
		bool fboost_active, warmup_active;
		unsigned int fboost_ramp_pct;
		unsigned int fl, boost_fl, demand_pct;
		u64 down_step, slew_ns;

		warmup_active = p->gaming_warmup_end_ns && time < p->gaming_warmup_end_ns;

		/*
		 * Demand before esk_apply_headroom's inflation - what the
		 * sustained-load lock, the risk detector and the floor gate
		 * judge against. `util` has been inflated above, so it is not a
		 * load measurement; feeding it to these three made every
		 * threshold fire ~20 points early.
		 *
		 * CAVEAT, and do NOT re-tune the thresholds without reading
		 * this: raw_util is not real load either. It is filt_util, and
		 * esk_get_util_gki510 has already applied the standard 25% DVFS
		 * margin, so demand_pct reads ~1.25x measured demand (saturating,
		 * because that getter clamps to max_cap). Every threshold
		 * compared against it - the gate, the follow gate, the lift and
		 * drop latches, the cold-start and ramp deltas - therefore trips
		 * at about four fifths of its nominal number in real load.
		 *
		 * The constants below were tuned empirically WITH that skew
		 * present, so the skew and the numbers are a matched pair.
		 * Removing the 1.25x here without re-deriving all of them would
		 * raise every trip point by a quarter and cost responsiveness.
		 * Fix both together or neither.
		 */
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);

		/*
		 * One parameterised band for all three clusters. The three
		 * copies this replaces were identical apart from their
		 * percentages, and each had to be fixed separately - the exact
		 * shape that lets a bug survive in one branch after being fixed
		 * in the others.
		 */
		if (prime) {
			fl = esk_pct(fceil, RFX_G_PRIME_FLOOR_PCT);
			boost_fl = esk_pct(fceil, RFX_G_PRIME_FRAME_PCT);
		} else if (!little) {		/* Big: carries most load */
			fl = esk_pct(fceil, RFX_G_BIG_FLOOR_PCT);
			boost_fl = esk_pct(fceil, RFX_G_BIG_FRAME_PCT);
		} else {			/* Little: compositor / audio / input */
			fl = esk_pct(fceil, RFX_G_LITTLE_FLOOR_PCT);
			boost_fl = esk_pct(fceil, RFX_G_LITTLE_FLOOR_BOOST_PCT);
		}

		esk_update_sustained_lock(p, demand_pct, time);
		/*
		 * The emergency net clamps the final result, so the floor a
		 * boost could actually install is the clamped one - hand the
		 * detector the reachable value, not the nominal one, or it
		 * arms windows that resolve to the clamp it is already sitting
		 * on.
		 */
		esk_frame_risk_check(p, demand_pct,
				     esk_thermal_clamp(boost_fl, fceil), time);

		/*
		 * Bounded slew: fall limited to RFX_GAMING_DOWN_PCT_PER_MS
		 * of ceiling per millisecond. Measured in ns from last commit
		 * (not last eval) so budget accumulates correctly.
		 *
		 * Cap the accumulation window at the down-rate gate period.
		 * Without this, slew budget grows unbounded while the gate
		 * blocks commits, then the first accepted commit dumps the
		 * entire accumulated step in one shot — a frequency cliff
		 * that registers as a jank frame even though FPS stays high.
		 * With the cap, one gate period = one maximum step, so the
		 * descent is smooth even when commits are sparse.
		 */
		slew_ns = time - max(p->last_upfreq_time, p->last_downfreq_time);
		slew_ns = min_t(u64, slew_ns,
				(u64)RFX_GAMING_DOWN_US * NSEC_PER_USEC);
		down_step = (u64)esk_pct(fceil, RFX_GAMING_DOWN_PCT_PER_MS) *
			    slew_ns / NSEC_PER_MSEC;
		if (down_step < fceil && p->next_freq > (unsigned int)down_step &&
		    freq < p->next_freq - (unsigned int)down_step)
			freq = p->next_freq - (unsigned int)down_step;

		fboost_active = esk_frame_boost_active(time);
		fboost_ramp_pct = esk_update_frame_boost_ramp(p, fboost_active, time);

		/*
		 * Idle gate: demand below 25% releases to idle floor.
		 * Warmup bypasses the idle gate (demand hasn't appeared yet)
		 * but does NOT pin to boost_fl — that cooked idle clusters.
		 * Frame-boost ramp lifts the floor proportionally when a
		 * frame-risk event fired on any cluster.
		 */
		if (demand_pct < RFX_G_FLOOR_GATE_PCT && !warmup_active)
			fl = esk_pct(fceil, RFX_G_IDLE_FLOOR_PCT);
		else if (fboost_ramp_pct > 0 &&
			 demand_pct >= RFX_G_BOOST_FOLLOW_PCT)
			fl = fl + (boost_fl - fl) * fboost_ramp_pct / 100;

		if (freq < fl)
			freq = fl;
		/* RFX_G_*_CAP_PCT are all 100 — cap == fceil, clamp is a no-op.
		 * Re-add if a future rev lowers one. */
	} else {
		bool ui_active, coldstart_active;
		unsigned int demand_pct;

		/*
		 * Raw demand, before headroom - the same correction the gaming
		 * band needed. `util` has been inflated by esk_apply_headroom
		 * above, and the daily tiers are stepped (+10% over 65/70,
		 * +5% over 40/45), so post-headroom percent is not a load
		 * measurement and is not even monotone in step size: crossing a
		 * tier boundary jumps the reported value by up to seven points
		 * with no change in load. Every threshold below was reading
		 * that. The Little cap latch nominally set at 72% was really
		 * latching at 66% real load, and the 15-point ramp trigger
		 * could be satisfied by a tier crossing alone - a burst boost
		 * armed by arithmetic rather than by a burst.
		 *
		 * Same caveat as the gaming band: raw_util still carries the
		 * 25% DVFS margin from esk_get_util_gki510, so this is ~1.25x
		 * real demand and the constants below were tuned with that skew
		 * in place. See the longer note there before re-tuning.
		 */
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);

		/* Cold-start: a 0->high demand jump arms aggressive floors. */
		if (demand_pct >= RFX_D_COLDSTART_DELTA_PCT &&
		    p->prev_upct <= RFX_D_COLDSTART_BASE_PCT)
			p->coldstart_boost_end_ns = time + RFX_D_COLDSTART_BOOST_NS;

		/*
		 * Detect a render burst: a sharp rise in demand re-arms the UI
		 * window. Catches caption draws / open-close animations.
		 */
		if (demand_pct > p->prev_upct &&
		    demand_pct - p->prev_upct >= RFX_D_RAMP_DELTA_PCT)
			p->ui_boost_end_ns = time + RFX_D_UI_BOOST_NS;

		/*
		 * Refresh the reference on a fixed cadence (see
		 * RFX_D_RAMP_SAMPLE_NS) and on NOTHING else.
		 *
		 * An earlier version also snapped the reference down the instant
		 * demand fell, on the theory that a rise should be measured from
		 * the trough. That turned the detector into a peak-to-trough
		 * comparator over an unbounded window: any oscillating load -
		 * video playback, a sync, a sensor batch - drops, re-arms the
		 * reference at its own minimum, and the next upswing reads as a
		 * 12-point ramp. Each false positive arms a 280ms window that
		 * pins the Little floor, lifts its cap and drags all three
		 * clusters to the 250us evaluation rate, and the windows overlap
		 * faster than they expire, so the device never leaves the
		 * interaction state while anything at all is running. The
		 * detector went from never firing to always firing; both are
		 * broken, and the second one costs battery.
		 *
		 * Against a fixed 16ms reference the comparison means what it
		 * says - demand is 12 points higher than it was one 60Hz frame
		 * ago - which a real scroll or animation satisfies and load
		 * noise does not.
		 */
		if (!p->prev_upct_ns ||
		    time - p->prev_upct_ns >= RFX_D_RAMP_SAMPLE_NS) {
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

			/*
			 * Sustained heavy load relaxes the cap: the 60%
			 * battery cap otherwise strangles every long
			 * multithread workload (compile, media scan,
			 * untouched-screen game) once the touch and burst
			 * windows expire. It relaxes to 85%, not to fmax -
			 * see RFX_D_LITTLE_SUSTAINED_CAP_PCT. Light use never
			 * reaches this demand, so idle battery is unaffected.
			 */
			if (demand_pct >= RFX_D_LITTLE_LIFT_PCT)
				p->little_cap_lifted = true;
			else if (demand_pct <= RFX_D_LITTLE_DROP_PCT)
				p->little_cap_lifted = false;
			if (p->little_cap_lifted)
				cap = max(cap, esk_pct(fceil,
					RFX_D_LITTLE_SUSTAINED_CAP_PCT));

			if (freq > cap)
				freq = cap;

			/*
			 * Interaction floor, applied AFTER the cap so a lifted
			 * cap can never be undercut by it and the floor can
			 * never exceed the cap it was just clamped to. Only
			 * while the touch or UI-burst window is open - see
			 * RFX_D_LITTLE_UI_FLOOR_PCT for why the cluster that
			 * runs the input pipeline needs a floor and not just a
			 * higher cap.
			 */
			if (ui_active) {
				unsigned int fl = min(cap,
					esk_pct(fceil, RFX_D_LITTLE_UI_FLOOR_PCT));

				if (freq < fl)
					freq = fl;
			}
		} else if (coldstart_active) {
			/*
			 * Big and Prime differ only in floor percentage; the
			 * two identical branches this replaces were a place
			 * for one to be fixed and the other forgotten.
			 */
			unsigned int fl = esk_pct(fceil, prime ?
					RFX_D_PRIME_BURST_FLOOR_PCT :
					RFX_D_BIG_BURST_FLOOR_PCT);

			if (freq < fl)
				freq = fl;
		}
	}

	freq = esk_thermal_clamp(freq, fceil);
	freq = clamp(freq, fmin, fceil);

	/*
	 * Cache the raw request only once it is actually committed (see
	 * esk_commit_freq). Writing it here unconditionally poisoned the cache
	 * whenever the rate-limit gate rejected the commit: the next tick
	 * recomputed the same raw value, hit the cache, returned the stale
	 * next_freq, and the pending change was lost until demand happened to
	 * move again - long flat segments in the frequency trace with no load
	 * reason behind them.
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
	 * Escalate toward a per-cluster ceiling. Little stays modest - IO
	 * completions there are housekeeping.
	 *
	 * Big/Prime are split by profile. Gaming ceiling lowered to 60% to
	 * reduce thermal load during asset streaming phases. The old 87.5%
	 * ceiling pushed cores high while waiting on storage, generating
	 * unnecessary heat. 60% covers CPU-side work (decompression, parsing)
	 * without paying for the IO wait itself. Daily ceiling at 25% covers
	 * sqlite commits, log flushes, and image decoding at the V/f knee
	 * without pushing cores to a high OPP for millisecond-scale waits.
	 */
	if (esk_c->iowait_boost) {
		max_cap = arch_scale_cpu_capacity(esk_c->cpu);
		if (esk_cap_is_little(max_cap))
			cap = SCHED_CAPACITY_SCALE / 6;
		else if (esk_gaming_enabled())
			cap = SCHED_CAPACITY_SCALE * 3 / 5;	/* 60% */
		else
			cap = SCHED_CAPACITY_SCALE / 4;		/* 25% */
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

/* up-rate-limit: instant up while gaming, tunable otherwise. */
static inline void esk_pol_up_delay(struct esk_policy *p, bool gaming)
{
	if (gaming)
		p->up_rate_delay_ns = 0;
	else
		p->up_rate_delay_ns =
			(s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
}

/*
 * Evaluation delay for this update. Must be set BEFORE esk_should_update_freq,
 * so it can only depend on state available up front - which is exactly right:
 * every condition that justifies a sub-millisecond DVFS cadence is known
 * without looking at util.
 *
 * The fast gate covers the whole interaction, not just the finger: gaming, the
 * touch window, and the UI-burst window that outlives a lifted finger through
 * momentum and animation tails. Everything else - including an idle Little
 * cluster - uses its tunable, so writing rate_limit_us from sysfs still means
 * what it says and standby costs one evaluation per 1.5ms rather than three.
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

static void esk_update_single_freq(struct update_util_data *hook, u64 time,
				   unsigned int flags)
{
	struct esk_cpu *esk_c = container_of(hook, struct esk_cpu, update_util);
	struct esk_policy *p = esk_c->esk_policy;
	bool gaming = esk_gaming_enabled();
	unsigned long max_cap, boost, eff, irqflags;
	unsigned int next_f;
	bool do_deferred = false;
	u64 ema_delta;

	max_cap = arch_scale_cpu_capacity(esk_c->cpu);

	esk_iowait_boost(esk_c, time, flags);
	esk_c->last_update = time;
	esk_ignore_dl_rate_limit(esk_c);
	esk_set_eval_delay(p, gaming, time);

	if (likely(!esk_should_update_freq(p, time)))
		return;

	p->last_eval_time = time;
	boost = esk_iowait_apply(esk_c, time, max_cap);
	esk_get_util(esk_c, boost);
	eff = max(esk_c->util, boost);
	ema_delta = p->last_ema_ns ? time - p->last_ema_ns : RFX_EMA_DECAY_PERIOD_NS;
	p->last_ema_ns = time;
	p->filt_util = esk_ema(p->filt_util, eff, ema_delta, gaming);

	esk_set_down_delay(p, gaming);
	esk_pol_up_delay(p, gaming);

	next_f = esk_target_freq(p, p->filt_util, max_cap, time, gaming);

	if (!esk_commit_freq(p, time, next_f))
		return;

	if (p->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(p->policy, p->next_freq);
	} else {
		raw_spin_lock_irqsave(&p->update_lock, irqflags);
		if (!p->work_in_progress) {
			p->work_in_progress = true;
			do_deferred = true;
		}
		raw_spin_unlock_irqrestore(&p->update_lock, irqflags);

		if (do_deferred)
			irq_work_queue(&p->irq_work);
	}
}

static unsigned int esk_next_freq_shared(struct esk_cpu *esk_c, u64 time,
					 bool gaming)
{
	struct esk_policy *p = esk_c->esk_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(esk_c->cpu);
	unsigned long max_util = 0;
	unsigned int j;
	u64 ema_delta;

	/*
	 * Aggregate max util across the policy's CPUs first, then filter once.
	 * The EMA lives on the policy, not on the CPU that ran the hook: a
	 * shared policy commits a single frequency, so a per-CPU filter left
	 * every CPU with a different idea of cluster demand and the committed
	 * value flipped with whichever CPU ticked last - frequency jitter and
	 * micro-stutter with no change in load.
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

	ema_delta = p->last_ema_ns ? time - p->last_ema_ns : RFX_EMA_DECAY_PERIOD_NS;
	p->last_ema_ns = time;
	p->filt_util = esk_ema(p->filt_util, max_util, ema_delta, gaming);

	esk_set_down_delay(p, gaming);
	esk_pol_up_delay(p, gaming);

	return esk_target_freq(p, p->filt_util, max_cap, time, gaming);
}

static void esk_update_shared(struct update_util_data *hook, u64 time,
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
		next_f = esk_next_freq_shared(esk_c, time, gaming);
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
	cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
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

	/*
	 * Latching emergency net with 7C hysteresis. Trip once, hold until the
	 * die has genuinely cooled, then release once. It cannot chatter, which
	 * is the entire point: the proportional relay this replaces re-evaluated
	 * a temperature-derived target every 50ms against a 6ms/1% walk, and
	 * fought both hardware LMH and its own effect on temperature. That
	 * fight is what the shark-tooth sustain graph was a picture of.
	 */
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
		atomic_set(&esk_emergency_cap_pct, 100);
	}

	delay_ms = esk_gaming_enabled() ? RFX_THERMAL_POLL_GAMING_MS :
					  RFX_THERMAL_POLL_IDLE_MS;
	schedule_delayed_work(&esk_thermal_work, msecs_to_jiffies(delay_ms));
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

/* Reset transient gaming residue on every live policy (all clusters). */
static void esk_reset_all_policies(void)
{
	struct esk_policy *p;
	unsigned long flags, pflags;

	spin_lock_irqsave(&esk_policy_list_lock, flags);

	atomic64_set(&esk_frame_boost_end_ns, 0);

	list_for_each_entry(p, &esk_policy_list, gov_node) {
		raw_spin_lock_irqsave(&p->update_lock, pflags);
		p->frame_boost_ramp_pct = 0;
		p->frame_boost_ramp_last_ns = 0;
		p->gaming_warmup_end_ns = 0;
		p->risk_high = false;
		p->sustained_since_ns = 0;
		p->sustained_locked = false;
		p->need_freq_update = true;
		raw_spin_unlock_irqrestore(&p->update_lock, pflags);
	}
	spin_unlock_irqrestore(&esk_policy_list_lock, flags);
}

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_esk_tunables(attr_set)->gaming_mode);
}
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct esk_tunables *t = to_esk_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;

	t->gaming_mode = val;
	atomic_set(&esk_gaming, val);

	if (!val) {
		esk_reset_all_policies();
	} else {
		struct esk_policy *p;
		unsigned long flags, pflags;
		u64 now = sched_clock();

		spin_lock_irqsave(&esk_policy_list_lock, flags);
		list_for_each_entry(p, &esk_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			p->need_freq_update = true;
			/* Warmup floor lift covers process spawn / asset load. */
			p->gaming_warmup_end_ns = now + RFX_GAMING_WARMUP_NS;
			/* Clear daily state so it cannot shape gaming decisions. */
			p->little_cap_lifted = false;
			p->ui_boost_end_ns = 0;
			p->coldstart_boost_end_ns = 0;
			p->prev_upct = 0;
			p->prev_upct_ns = 0;
			p->sustained_since_ns = 0;
			p->sustained_locked = false;
			/* Stale risk latch from a previous session. */
			p->risk_high = false;
			raw_spin_unlock_irqrestore(&p->update_lock, pflags);
		}
		spin_unlock_irqrestore(&esk_policy_list_lock, flags);

		/* Sample temperature sooner once gaming begins. */
		mod_delayed_work(system_wq, &esk_thermal_work,
				 msecs_to_jiffies(RFX_THERMAL_POLL_GAMING_MS));
	}
	return count;
}
static struct governor_attr gaming_mode = __ATTR_RW(gaming_mode);

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
	return count;
#else
	return -ENODEV;
#endif
}
static struct governor_attr thermal_zone = __ATTR_RW(thermal_zone);

static struct attribute *esk_little_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(esk_little);

static struct attribute *esk_big_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(esk_big);

static struct attribute *esk_prime_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&gaming_mode.attr,
	&temp_mc.attr,
	&thermal_zone.attr,
	NULL
};
ATTRIBUTE_GROUPS(esk_prime);

static void esk_tunables_free(struct kobject *kobj)
{
	kfree(to_esk_tunables(esk_to_gov_attr_set(kobj)));
}

static struct kobj_type esk_little_ktype = {
	.default_groups = esk_little_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = esk_tunables_free,
};
static struct kobj_type esk_big_ktype = {
	.default_groups = esk_big_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = esk_tunables_free,
};
static struct kobj_type esk_prime_ktype = {
	.default_groups = esk_prime_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = esk_tunables_free,
};

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

static int esk_kthread_create(struct esk_policy *p)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = p->policy;
	struct sched_param sp = { .sched_priority = MAX_RT_PRIO / 2 };
	int ret;

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

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &sp);
	if (ret) {
		kthread_stop(thread);
		pr_warn("esk: failed to set SCHED_FIFO\n");
		return ret;
	}

	p->thread = thread;
	if (policy->dvfs_possible_from_any_cpu)
		set_cpus_allowed_ptr(thread, policy->related_cpus);
	else
		kthread_bind_mask(thread, policy->related_cpus);

	init_irq_work(&p->irq_work, esk_irq_work);
	mutex_init(&p->work_lock);
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
	struct kobj_type *ktype;
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

	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	p->is_prime = esk_cap_is_prime(max_cap);
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

	if (p->is_little) {
		t->rate_limit_us = RFX_LITTLE_RATE_US;
		t->up_rate_limit_us = RFX_LITTLE_UP_US;
		t->down_rate_limit_us = RFX_LITTLE_DOWN_US;
		ktype = &esk_little_ktype;
	} else if (p->is_prime) {
		t->rate_limit_us = RFX_PRIME_RATE_US;
		t->up_rate_limit_us = RFX_PRIME_UP_US;
		t->down_rate_limit_us = RFX_PRIME_DOWN_US;
		ktype = &esk_prime_ktype;
	} else {
		t->rate_limit_us = RFX_BIG_RATE_US;
		t->up_rate_limit_us = RFX_BIG_UP_US;
		t->down_rate_limit_us = RFX_BIG_DOWN_US;
		ktype = &esk_big_ktype;
	}

	policy->governor_data = p;
	p->tunables = t;

	ret = kobject_init_and_add(&t->attr_set.kobj, ktype,
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
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned long flags;
	unsigned int cpu;
	u64 now = sched_clock();

	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;

	p->last_upfreq_time = now;
	p->last_downfreq_time = now;
	p->last_eval_time = now;
	p->next_freq = policy->cur > 0 ? policy->cur : policy->cpuinfo.min_freq;
	p->cached_raw_freq = 0;
	p->pending_raw_freq = 0;
	p->work_in_progress = false;
	p->limits_changed = false;
	p->need_freq_update = false;
	p->prev_upct = 0;
	p->prev_upct_ns = 0;
	p->ui_boost_end_ns = 0;
	p->coldstart_boost_end_ns = 0;
	p->filt_util = 0;
	p->last_ema_ns = 0;

	p->frame_boost_ramp_pct = 0;
	p->frame_boost_ramp_last_ns = 0;
	p->gaming_warmup_end_ns = 0;
	p->risk_high = false;
	p->little_cap_lifted = false;
	p->sustained_since_ns = 0;
	p->sustained_locked = false;

	spin_lock_irqsave(&esk_policy_list_lock, flags);
	list_add(&p->gov_node, &esk_policy_list);
	spin_unlock_irqrestore(&esk_policy_list_lock, flags);

	for_each_cpu(cpu, policy->cpus) {
		struct esk_cpu *esk_c = per_cpu_ptr(&esk_cpu, cpu);

		memset(esk_c, 0, sizeof(*esk_c));
		esk_c->cpu = cpu;
		esk_c->esk_policy = p;
	}

	uu = policy_is_shared(policy) ? esk_update_shared : esk_update_single_freq;
	for_each_cpu(cpu, policy->cpus)
		cpufreq_add_update_util_hook(cpu, &per_cpu_ptr(&esk_cpu, cpu)->update_util, uu);
	return 0;
}

static void esk_stop(struct cpufreq_policy *policy)
{
	struct esk_policy *p = policy->governor_data;
	unsigned long flags;
	unsigned int cpu;

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

static int __init esk_gov_init(void)
{
	int ret;

	pr_info("ESK Governor v%s by %s\n", CPUFREQ_ESK_VERSION,
		CPUFREQ_ESK_AUTHOR);

	INIT_DEFERRABLE_WORK(&esk_thermal_work, esk_thermal_fn);
	schedule_delayed_work(&esk_thermal_work,
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
MODULE_DESCRIPTION("ESK CPUFreq Governor v2.1");
