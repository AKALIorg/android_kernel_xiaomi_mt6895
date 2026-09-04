# ESK Reborn Kernel — Developer Handoff Document

**Purpose:** This file is a complete context-transfer document. Give this file plus the
kernel source repository to any AI model or developer, and they can continue development
without any prior session knowledge.

- **Repo (kernel):** https://github.com/AKALIorg/android_kernel_xiaomi_mt6895
- **Branch:** `16.2-rebase`
- **Repo (releases):** https://github.com/AKALIorg/ESK-Kernel-Reborn-Releases
- **Maintainer / owner:** AKALIorg (alirahsepar199@gmail.com), GitHub user `AKALIorg`
- **Device:** Xiaomi POCO X4 GT (`xaga`), 8GB RAM — Dimensity **8100** (MT6895Z):
  4×Cortex-A78 @ 2.85GHz (CPU4-6 = Big cluster, CPU7 = **Prime, separate DVFS domain**)
  + 4×Cortex-A55 @ 2.0GHz (CPU0-3 = Little). DTS: `capacity-dmips-mhz` = 380 (little) /
  1024 (big+prime). THREE cpufreq performance-domains in `mt6895.dts` (0=CPU0-3, 1=CPU4-6, 2=CPU7).
- **OS base:** Google **android12-5.10** GKI common kernel (device ships with Android 12);
  ROM support target: Android 16/17 custom ROMs (untested).
- **Current stable sublevel:** **5.10.269** (tracked; check kernel.org for newer).
- **Localversion convention:** `CONFIG_LOCALVERSION="-ESK-Reborn_V0.X"` in
  `arch/arm64/configs/vendor/xaga.config` — **bump per release** (V0.3 currently).
  `uname -r` shows `5.10.269-android12-...-ESK-Reborn_V0.3/<git-sha12>`.

---

## 1. CRITICAL BUILD & TEST INFRASTRUCTURE

### 1.1 Builder
- Builds happen via a private builder at `~/esk_builder/` on the maintainer's PC
  (scripts: `build.sh`, `config.sh`, `build/compile.sh`, `build/setup.sh`).
- **Config merge order** (from `build/compile.sh`):
  `scripts/kconfig/merge_config.sh -m -r gki_defconfig vendor/xiaomi_mt6895.config vendor/xaga.config`
- **Builder patches** applied at setup time (in this order):
  1. SuSFS: `git clone gitlab.com:simonpunk/susfs4ksu -b gki-android12-5.10`, copies
     `kernel_patches/fs/*` + `include/*`, then `patch -p1 --fuzz=3 < 50_add_susfs_in_gki-android12-5.10.patch`
  2. **LXC support:** `~/esk_builder/kernel_patches/lxc_support.patch` (adds SYSVIPC,
     POSIX_MQUEUE, namespaces, CGROUP_DEVICE, NAT netfilter bits to gki_defconfig —
     uses ANDROID_KABI_RESERVE(6/7/8) in `include/linux/sched.h`; ESK's BORE uses reserves 1-4,
     no conflict)
  3. `stock_config.patch` (only for stock builds)
  4. KernelSU variants: clones `ReSukiSU/ReSukiSU` (branch main), enables `CONFIG_KSU`
  5. Variants: VNL (vanilla), KSU-SUSFS, KSU-SUSFS-LXC (LXC flag applies lxc_support.patch)
- **Toolchain:** Android clang (currently r596125-based, "clang 22.0.2") at `~/esk_builder/clang/`.
  Build flags: `LLVM=1 LLVM_IAS=1`, `ARCH=arm64`, out-dir = `~/esk_builder/work`,
  source copy = `~/esk_builder/kernel` (the builder syncs/pulls from GitHub; if manually syncing,
  copy every changed file into BOTH `~/esk_builder/kernel/` and `~/esk_builder/work/`).
- Outputs: `~/esk_builder/out/` → AnyKernel3 zips named
  `ESK-Reborn-<ver>-<variant>-<git-sha12>-AnyKernel3.zip` + `module.tar.xz` (608 modules).

### 1.2 Build verification protocol (ALWAYS do before committing kernel changes)
Single-file compile tests with the builder's clang against the prepared work tree:

```bash
K="/home/akali/Documents/Default Project/android_kernel_xiaomi_mt6895"
W=~/esk_builder/work
# copy changed files to BOTH $W/<path> AND ~/esk_builder/kernel/<path>
cd $W && export PATH=~/esk_builder/clang/bin:$PATH
# config: merge fresh when needed
bash ~/esk_builder/kernel/scripts/kconfig/merge_config.sh -m .config <(fragment) 
make O=. ARCH=arm64 CC=clang HOSTCC=clang LLVM=1 LLVM_IAS=1 olddefconfig
make O=. ARCH=arm64 CC=clang HOSTCC=clang LLVM=1 LLVM_IAS=1 <path/to/file.o>
```
Notes:
- `O=` builds read sources from `~/esk_builder/kernel` (srctree) — the work-tree copies
  of files that are only in `work/` are ignored for headers; when in doubt copy to both.
- vdso prepare may fail on the host (bfd vs llvm emulation) — ignore, build single objects.
- NEVER trust `git push` — **always verify with `git log --oneline origin/<branch> -1`** after
  pushing. Pushes have silently failed before.

### 1.3 Debugging crashes on device (proven workflow)
- Cmdline contains `aee_aed.poffreason=AP_WDT` → `AP_WDT` = hardware watchdog reset
  (full system hang, NO kernel panic logged). `ramoops.mem_address=0x48090000`,
  console_size=0x40000 (wraps fast — the panic moment may be missing).
- Grab logs **immediately** (before another reboot):
```bash
adb shell su -c 'ls /sys/fs/pstore/'
adb shell su -c 'cat /sys/fs/pstore/console-ramoops-0' > ramoops-console.txt
adb shell su -c 'dmesg' > dmesg-now.txt
adb logcat -d > logcat-main.txt
adb logcat -d -b events > logcat-events.txt
```
- For recoverable stalls (20s freeze, no reboot): enable hung-task detection FIRST,
  then reproduce, then pull dmesg — this gives the exact lock + holder:
```bash
adb shell su -c 'sysctl -w kernel.hung_task_timeout_secs=10'
adb shell su -c 'sysctl -w kernel.hung_task_warnings=100'
# reproduce, then:
adb shell su -c 'dmesg' > hung_task_dump.txt
```
- Red flags in logs: `SPM system_bus didn't enter low power` every ~5s +
  `[RC] ratio ... dram:0% syspll:0%` = 100% busy livelock. `sched: RT throttling activated`
  = RT overload (symptom, usually not root cause). `audit_lost=<huge>` = SELinux denial
  flood (userspace noise, MI ROM).
- Known harmless: `mali_submit not set to RT prio` (Mali RT kthread bind fails under
  cpuset/thermal churn — pre-existing), `gpufreq_fix_target_oppidx: fail to fix STACK OPP
  index` (userspace booster procfs write rejected by GPUEB — thermal guard).

---

## 2. FEATURE INVENTORY (what is in the tree, commit references)

Base: `dd3b1030` = 5.10.266 vendor tree. Current HEAD sequence (all pushed to `16.2-rebase`):

| Commit | What |
|---|---|
| `f8e3dbe2` | ANDROID: imgsensor frame_sync 4K stack warning fix (first ESK commit) |
| `221f9dae`/`df46125d` | 5.10.267 + 5.10.268 stable merges |
| `03e2cd2b` | huge_memory `flags` build fix after 267 |
| `3680ab2b` | **NoMount** (maxsteeel/nomount, keyring-based) built-in at `fs/nomount/`, `CONFIG_NOMOUNT=y`; Droidspaces netfilter bits + `TMPFS_POSIX_ACL` + `WQ_POWER_EFFICIENT_DEFAULT` in `vendor/xiaomi_mt6895.config` |
| `2256037a` | **EEVDF + BORE backport** (from Templar-Kernel-GKI-5.10 commits `3e7145c6`,`bb8bac7d`,`e37771cf`,`8c0ad4c4`,`f65f0962`) — applied as patch files with `git apply`, some hunks needed manual application (`git apply --exclude=kernel/sched/features.h` + manual features.h hunk) |
| `7760bd3f` | **BBRplus** (UJX6N/bbrplus-5.10 + Templar tuning); added `bbrplus_tso_segs()` shim because 5.10 `tcp_congestion_ops` has `tso_segs(sk,mss)` not `min_tso_segs(sk)` |
| `3092c431` | **le9uo** working set protection (Templar `c0762abeb0` + `6476d4d5c8`) |
| `db442051` | **ESK governor** = Templar Vorpal v2.1 renamed (vorpal→esk, rfx_→esk_); `drivers/cpufreq/cpufreq_esk.c`; helpers `esk_get_util_gki510`/`esk_dl_bw_exceeded_gki510` appended to `kernel/sched/cpufreq_schedutil.c`, declared in `include/linux/sched/cpufreq.h` |
| `7f86a52f` | `CONFIG_BRIDGE_NETFILTER=y` |
| `9a8402c7` | ESK v2 daily retune (later partially softened — see history) |
| `b55531ca` | **5.10.269** stable merge (clean; brought upstream u64-to_ratio fix already) |
| `fe6ff006` | `ZRAM_WRITEBACK=y` (later REMOVED), battery bits |
| `5a64ca47` | **ZRAM_WRITEBACK removed** (UFS health), **KSM=y** added (off at boot) |
| `eacd4cc4` | localversion → `_V0.3` |
| `a0810861` | default governor → schedutil (ESK opt-in) after freeze reports |
| `ecfce530` | ESK skippability probe (fast-switch idle re-eval skip) |
| `1bf9ef41` | BBRv3 mobile tuning (pacing margin 2%, PROBE_RTT 140ms, probe base 1.9s; min_rtt window KEPT at 10s per Templar's later cellular fix `f94513b439`) |
| `87c22842` | **PELT half-life 16ms** (Templar's `pelt.c` runtime-switcher + `sched-pelt.h` tables pelt8/12/16/32 + Kconfig choice in `init/Kconfig`), `RCU_BOOST=y delay=15`, `IDLE_PAGE_TRACKING=y` |
| `91d0b1ff` | ESK irq_work/work_lock unconditional init |
| `47b9606d` | limits_changed 1ms gate + clamp-only fast path |
| `b1d029ca` | **CRITICAL FIX: two early-returns inside raw_spin_lock in `esk_update_shared()`** (clamp path + skippable path) — held `update_lock` with IRQs off → AP_WDT. Both now unlock before returning |
| `db15178e` | PELT `LOAD_AVG_PERIOD`/`LOAD_AVG_MAX` compile-time constants when fixed half-life selected; `LOAD_AVG_MAX` moved inside `#else` branch (Werror macro-redefined fix) |
| `527111b4` | **binder freeze fix**: `binder_install_single_page()` uses `mmap_write_lock_killable()` — cgroup-frozen app holding mmap_sem read during page fault blocked all binder txns into it (root cause of 20s game freezes + app kills) |
| `25176a53` | **le9uo ratios DISABLED (0/0/0)** — root cause of game-specific freezes: `clean_min_ratio=25%` (~1GB floor) made `shrink_folio_list()` keep_every_page under gaming memory pressure → direct reclaim zero-progress livelock. Sysctls remain runtime-tunable |
| `90bac8e9` | **EEVDF-functional sched_yield** (skip-buddy is a no-op under EEVDF; now refreshes slice/deadline for entitled entities) + **BORE reweight deadline refresh** (reweight_task_by_prio now recomputes slice/deadline; needs forward decls of `sched_slice`/`calc_delta_fair`) + removed dead `set_skip_buddy` |
| `25462d21` | **ESK three-tier classification + Prime-selective gaming boost**: prime = policy owning topmost CPU (`esk_policy_is_prime()`), NOT capacity alone; new sysfs `prime_gaming_floor_pct` (0-100, default 70) on prime tunables; big tier releases to idle floor at demand<40% in gaming (prime holds 25%) |

### 2.1 Known-in-tree-but-inert features
- **NoMount**: dentry-op hooks only attach to dentries with registered rules; zero rules
  (default) = dormant. Userspace = NoMount metamodule (KSU/APatch) or `nm` CLI via keyring.
- **KSM**: `CONFIG_KSM=y` but `ksm/run=0` at boot; enable via `/sys/kernel/mm/ksm/run`.
- **Droidspaces**: config support only (in LXC variant via lxc_support.patch).
- **DriveDroid**: already supported by GKI (`USB_F_MASS_STORAGE=y` builtin).

### 2.2 What was evaluated and REJECTED (do not re-add)
- `sched_rt_runtime_us` bump to 980000: RT throttling was a symptom, not root cause;
  weakening the safety valve risks hard hangs. Runtime sysctl if ever needed.
- EAS energy-model hand-tuning: no in-tree table for MT6895 (MediaTek EAS supplies
  out-of-tree); hand-tuning without profiling is dangerous. Only safe step: distinct
  capacity-dmips-mhz for CPU7 in DTS (not done, not needed for governor-level work).
- Templar adios 3.2.2 / ssg tuning: our adios 3.2.0 already more battery-aggressive.
- NoMount v1.1.1 (Templar): genl-based, breaks keyring metamodule userspace compat.
- NTSYNC: driver absent in their tree too (dead defconfig option).
- BBR min_rtt window 7s: Templar reverted it themselves (cellular PROBE_RTT churn) — keep 10s.

---

## 3. ESK GOVERNOR — CURRENT STATE & NEXT STEPS

File: `drivers/cpufreq/cpufreq_esk.c` (~2500 lines). Governor name: `esk`.
Structural facts (MUST KNOW):
- **MT6895 uses `mediatek,cpufreq-hw` → `fast_switch_possible=true`** → ESK kthreads are
  NEVER created (`esk_kthread_create()` early-returns); **all commits run inline in the
  scheduler hook with IRQs disabled** via `cpufreq_driver_fast_switch()` (just a
  `writel_relaxed` to HW). Templar's Poco F5 (Qualcomm) uses the kthread path —
  their governor was never exercised on the inline path. ALL ESK bugs so far were
  inline-path bugs. **Rule: every early-exit inside `esk_update_shared()` MUST unlock
  `p->update_lock` (raw_spin_lock_irqsave) before returning.** Audit all 5 lock sites
  after touching this file.
- sysfs: `gaming_mode` (prime policy), `prime_gaming_floor_pct` (NEW), `temp_mc`,
  `thermal_zone`, per-cluster `rate_limit_us`/`up/down_rate_limit_us`.
- Tier classification (25462d21): little = cap≤614; prime = cap≥1000 AND policy owns
  topmost CPU (`esk_policy_is_prime`); big = the rest. Two-tier SoCs: top policy=prime,
  big tier unused (preserves 2-cluster behavior).
- Governor must be enabled+default via `vendor/xiaomi_mt6895.config`
  (`CONFIG_CPU_FREQ_GOV_ESK=y`, `CONFIG_CPU_FREQ_DEFAULT_GOV_ESK=y`). **Currently
  default=schedutil, ESK=opt-in** (revert happened after freeze reports; crashes since
  fixed). Making ESK default again = pending user validation.
- ESK renames: vorpal→esk, rfx_→esk_ everywhere. Original author credit
  (Templar Dev / Steambot12) preserved in header. Upstream repo:
  https://github.com/Steambot12/Templar-Kernel-GKI-5.10 and
  https://github.com/Steambot12/Governor-Config (their tunables/logs repo).

### 3.1 NEXT MAJOR TASK: Vorpal/ESK v2.2 port (unstarted)
Templar's **v2.2** exists on branches `Linux1/2/3-Staging`
(`drivers/cpufreq/cpufreq_vorpal.c`, 2431 lines; commits `848abc923b`, `fa014b42ca`,
`497845d2b0`, `f88ce2fba3`, `07fa38fc87`, dated Aug 31–Sep 2 2026). 1550-line delta
from v2.1. Key changes to port (patches in `/tmp/opencode/patches/v22_*.patch` will be
gone after reboot — re-download from GitHub commit .patch URLs):
- New constants: `RFX_LITTLE_RATE_US 3000`, `RFX_BIG_RATE_US 3000`, `RFX_BIG_DOWN_US 2500`,
  `RFX_PRIME_DOWN_US 2500`, `RFX_UI_RATE_US 1500` (was 700)
- Gaming band rebalance: PRIME floor 70→64, frame 88→92; BIG floor 66→58, frame 86→90;
  LITTLE floor 55→60, boost 74→80
- Daily: `RFX_D_LITTLE_BOOST_CAP_PCT 80`, `UI_FLOOR 32`, `DROP_PCT 55`, **NEW Big/Prime
  daily caps** (`RFX_D_BIG_CAP_PCT 70`, `RFX_D_BIG_BOOST_CAP_PCT 80`, `RFX_D_PRIME_CAP_PCT 68`,
  `RFX_D_BIG_LIFT_PCT 65`, `RFX_D_BIG_SUSTAINED_CAP_PCT 94`, `RFX_D_PRIME_SUSTAINED_CAP_PCT 85`)
- `RFX_D_UI_BOOST_NS 280→150ms`, `RFX_INPUT_WINDOW_NS 280→230ms`
- **EMA rework**: `RFX_EMA_GAMING_DIVISOR 100`, `RFX_EMA_MAX_STEPS 32`, iterative per-step decay
- **Headroom**: GAMING 12→2, SAT_GAMING 82→98 (near-raw demand in gaming)
- **NEW warmup logic**: extendable warmup (`RFX_GAMING_WARMUP_MAX_NS 600ms`,
  EXTEND_PCT 90, RELEASE_PCT 40, `warmup_low_demand_since_ns` state)
- **NEW thermal_cooling hysteretic band**: `RFX_G_COOL_ENTER_PCT 80 / EXIT_PCT 85`,
  `RFX_G_COOL_STEADY_FLOOR_PCT 52`, `RFX_G_COOL_BOOST_FLOOR_PCT 72` (floors drop to steady
  while ceiling is clamped, restore on exit)
- **NEW floor hysteresis**: `floor_gated` bool + `RFX_G_FLOOR_GATE_EXIT_PCT 35` (exit at 35,
  enter at 25 — no chatter)
- **NEW gaming ramp detector**: `RFX_G_RAMP_DELTA_PCT 15`, `RFX_G_RAMP_HOLD_NS 1ms`,
  `RFX_G_RAMP_SAMPLE_NS 8ms`, `prev_gaming_demand_pct/ns` state
- **sustained-lock REMOVED** entirely (state fields deleted)
- **`max_seen` per-policy high-water mark** replaces `qos_pct` in thermal headroom
  (`rfx_thermal_headroom_pct(p, max_cap, &baseline)` signature change)
- **`rfx_ntiers()`**: counts distinct capacities; `is_prime = is_top && ntiers>=3`
  (better than our cpu7-identity heuristic — adopt it)
- `is_top` bool added (gaming_mode node host); `is_prime` = "prime band applies (3+ tiers)"
- Frame-boost window 120→25ms, ramp-down 120→60ms with consumed-time accounting
- FRAME_BOOST_RAMP: consumed_ns bookkeeping (`frame_boost_ramp_last_ns += consumed`)
- Thermal poll: idle 3000→5000ms, new WARM 2000ms poll at `RFX_TEMP_WARM_MC 70000`
- RISK_SATURATION 85→90
Port strategy: hand-port onto our `cpufreq_esk.c` (rename rfx→esk), NOT patch files
(they won't apply — we renamed + added 3-tier). Take the FINAL v2.2 file
(`Linux3-Staging` raw URL) and merge per-hunk, keeping our `prime_gaming_floor_pct`
and our unlock-discipline. Compile-test `cpufreq_esk.o`, then device-test gaming_mode
before shipping.

### 3.2 Known remaining perf items (from user testing + PUBG SF-latency data)
- PUBG SF captures: avg FPS stable (~89) but 1% low/min FPS and jank vary run-to-run;
  CPU7 oscillates 300↔2850MHz; fixes shipped: EEVDF yield, BORE reweight deadline,
  prime-selective boost. NOT yet verified on device — test next build.
- `surfaceflinger`/`RenderThread` RT prioritization: deliberately NOT implemented
  (no blanket RT — no watchdog safety in tree; existing uclamp/top-app path preferred).
  If pursued: SCHED_RR prio 1-5 max, runtime-quota safeguard, name-matched via cgroup
  attach, userspace wiring required.
- EAS energy model: only safe step = distinct `capacity-dmips-mhz` for CPU7 in DTS
  (currently 1024 same as big). Prerequisite for any EAS-level prime awareness.
  Do NOT hand-tune power tables.

---

## 4. RELEASE WORKFLOW (releases repo)

Repo: `ESK-Kernel-Reborn-Releases` (branch `main`), files: `README.md` (variants +
supported devices xaga/xagapro/xagain table + versioning), `INSTALL.md` (AOSP-recovery
Apply-update→Apply-from-ADB flow only — no TWRP exists for this device; no backup
section per user request).

Release naming: tag `0.X`, title `ESK Reborn 0.X [Beta N]`. Current: **0.3 Beta 2
(prerelease)** with asset `ESK-Reborn-5.10.269-KSU-SUSFS-LXC-25176a53e9b8-AnyKernel3.zip`.
Tag renames: use `gh release edit <tag> --tag <newtag>`; **watch for orphaned git tag
refs** (delete via `gh api repos/.../git/refs/tags/<name> -X DELETE`). Prerelease
cannot be marked `--latest`.

Release notes style: markdown, device table, base/kernel info, Builds table
(file | features | source commit), Changelog grouped (Stable base / Scheduler & CPU /
Crash fixes / Memory & UFS / Networking / Containers / Power), install link.
Add "variants coming soon" note when partial variants upload. Verify EVERY upload with
`gh release view <tag> --json assets` and verify the zip's Image version string:
`unzip -p zip Image.zst | zstd -d | strings | grep "Linux version"` — a stale zip was
almost shipped once; also verify the commit hash inside matches the intended build.

User workflow: user builds variants locally, drops zips in `/home/akali/ESK_Reborn/<ver>_AKALIorg/`
folders, asks for upload. **ALWAYS verify zip's Image version string matches the intended
commit BEFORE uploading.**

Commit message format (Android Common Kernel rules):
- `ANDROID:` for vendor/device-specific, `BACKPORT:` for stable/mainline backports
- Body: what/why, Link: to source repos, `Fixes:` tag when fixing a prior commit
- `Change-Id: I<40-hex>` (generate: `I$(echo <unique> | sha1sum | cut -c1-40)`)
- `Signed-off-by: akaliorg <alirahsepar199@gmail.com>`

---

## 5. DEVICE-SPECIFIC FACTS (learned the hard way)

- **UFS health is poor** (user report): ZRAM writeback must stay OFF (removed in
  `5a64ca47`); KSM added as flash-free alternative.
- **Battery is aged**: 1783 cycles; sags to 3.6-3.9V under load at ~20% SOC.
  Charging + gaming = thermal_level up to 15, charge FCC cut to 500mA.
  Game freezes originally correlated with charging+gaming.
- **Freeze/crash history (all root-caused & fixed):**
  1. Boot-unlock/warm-switch AP_WDT → ESK inline-path spinlock double-holds (`b1d029ca`)
  2. Limits livelock (thermal min/max churn × full walk) (`47b9606d`)
  3. Game freezes + app kills + soft reboot, both governors → **le9uo reclaim livelock**
     (`clean_min_ratio=25%` floor) (`25176a53`) + **binder/cgroup-freezer mmap_sem wedge**
     (`527111b4`) — found via hung_task dumps (`sysctl kernel.hung_task_timeout_secs=10`)
  4. First 0.3 build "worked" by luck — races are timing-dependent.
- **UI lags first reported on ESK v1** → v2 retune + PELT16 + yield/reweight fixes address it.
- MT6895 GPU = Mali-Valhall + **GED/gpufreq stack** (GPUEB firmware owns GPU DVFS via IPI;
  kbase devfreq uses MTK "dummy" governor). **Never put simple_ondemand on the kbase
  devfreq** — two controllers fight. GPU errors (`gpufreq_fix_target_oppidx fail`) come
  from userspace writing debug procfs; GPUEB rejects under thermal guard. Harmless-ish.
- MT6895 CPU freq = `mediatek,cpufreq-hw` with `fast_switch_possible=true` (3 domains).
- Phone used on OmniROM(A16-style) w/ ReSukiSU+SUSFS; Roblox = the repro game.

---

## 6. TESTING CHECKLIST FOR EVERY NEW BUILD

1. `uname -r` shows expected version + commit sha
2. Boot → **unlock immediately** (historical crash trigger), repeat 3-4×
3. Heavy Roblox game **while charging** (thermal storm trigger), 10+ min
4. Background app switching while warm
5. Check `dmesg` for: RT throttling, SPM 0%-idle lines, audit flood rate, esk errors
6. If governor=esk: verify `gaming_mode` node exists at
   `/sys/devices/system/cpu/cpufreq/policy7/esk/gaming_mode` (prime policy),
   tune `prime_gaming_floor_pct`
7. sysctl sanity: `vm.anon_min_ratio` etc = 0; `kernel.sched_bore=1`;
   `net.ipv4.tcp_congestion_control=bbr` (bbrplus selectable)
8. 5+ hour daily use before promoting a prerelease to stable

---

## 7. UPSTREAM SOURCES TO WATCH

- **Steambot12/Templar-Kernel-GKI-5.10** (branches: Templar-LinuxStable = main,
  Linux1-7-Staging = newer governor work, Templar-MGLRU = unexplored, GoogleLTS-Staging,
  Templar-OSS, Templar-RC). **Governor-Config** repo = their defconfig + governor files + LOG.
- **maxsteeel/nomount** (dev branch; we use keyring version, v1.1.1 = genl — incompatible)
- **ravindu644/Droidspaces-OSS** (container runtime; requirements = our LXC config set)
- **simpunk/susfs4ksu** branch `gki-android12-5.10` (builder pulls it)
- **UJX6N/bbrplus-5.10** (bbrplus source)
- **kernel.org stable** (v5.10.x — rebase when new sublevel lands; procedure: fetch tag,
  `git merge --no-commit v5.10.26X`, resolve conflicts keeping vendor blocks, verify
  BORE/EEVDF KABI block in sched.h survives, commit `Update to Linux 5.10.26X`)
- **google/bbr** v3 branch (BBRv3 upstream; our in-tree bbr IS BBRv3 already,
  `BBR_VERSION=3`, from the GCE android12-5.10 tree)
- **MGLRU**: NOT in 5.10 tree; Templar has `Templar-MGLRU` branch — big future item,
  requires serious port work. Highest-impact remaining feature.

---

## 8. SESSION HANDOFF — DO THIS FIRST IN A NEW SESSION

1. Read this file fully.
2. `git -C <repo> log --oneline -30` and compare against §2 table — confirm no drift.
3. Check `git log origin/16.2-rebase -1` matches local (push verification habit).
4. Check kernel.org for new 5.10.x; check Templar branches for governor updates
   (v2.2 port is queued — §3.1).
5. Ask the user what variant they built and what they tested; pull fresh logs if issues.
6. Commit format: §4. Build/verify: §1.2. Debugging: §1.3.
7. **Communication style with user**: casual, direct, technical; user is hands-on
   (builds/tests locally, provides logs); explain root causes with evidence from THEIR
   logs; never push unverified fixes; always compile-test with their clang before push;
   always verify pushes AND uploaded zips.
