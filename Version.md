# SCARA_F103 Version Log

## 2026-06-14 v0.29.10 Fault-stop recovery and position re-anchor

- Fix a lockup: after a boundary/edge move (or a Stop during streaming), every subsequent
  motion — even a simple jog — was rejected with `error:15` until homing or a power cycle.
- Root cause: segment preparation used `target_position_pulse` as the first-segment pulse
  base, but an interrupted/decelerated stop leaves that at the old (unreached) target while
  `position_pulse` holds the true stop point. The first segment's pulse delta then equals the
  unfinished span and never shrinks, repeatedly forcing `preparation_fault` (latched →
  `error:15` on every following block).
- `MotionPlanner_Loop` now bases the first segment of a new stream (line and dwell) on
  `position_pulse` (true machine position) instead of `target_position_pulse`.
- The Ctrl-X (`0x18`) soft-reset path now re-anchors the parser via
  `resync_parser_to_stepper_if_diverged()` (matching the `0x85` / coordinate-set paths), so the
  parser position no longer lags the real stop point after an interrupted move.
- `MotionPlanner_Clear()` now also clears `s_preparation_fault_count`, so the host no longer
  re-aborts on a stale `Pf` after a soft reset.
- Net effect: the “停止(清除队列)” button (Ctrl-X) is a reliable one-press recovery, and normal
  streaming no longer deadlocks on a stale base. UI unchanged.

## 2026-06-14 Custom-pattern dropdown: SVG line art + image halftone (UI only)

- "固定轨迹 → 自定义轨迹" dropdown now auto-scans `SCARA_UI/trajectory/patterns/`:
  `.svg` files become vector line-art trajectories (✏), bitmap files (png/jpg/jpeg/bmp)
  become laser halftone patterns (🔥). The filename is the dropdown label; dropping a
  file in is enough — no code change.
- Added `SCARA_UI/trajectory/svg_loader.py`: a dependency-free SVG→QPainterPath parser
  (path `M/L/H/V/C/S/Q/T/Z/A`, basic shapes, transforms, nested `<g>`). SVG strokes reuse
  the existing glyph-outline pipeline (`_qt_path_contours` → `generate_stroke_motion`), so
  pen-drawing vs. laser engraving follows the existing "激光开启" switch.
- Image halftone: OpenCV grayscale → grid sample → dark cells become dwell-marked dots,
  streamed as `G0 → M4 S → G4 P → M5` per dot (firmware `G4` inherits the current laser
  mode, so the dwell marks the spot). Reuses the `laser_task_active`/`LASER ARM` safety
  chain — not armed = safe dry run. Spacing/dwell are `HALFTONE_*` constants; power reuses
  the UI power box.
- Firmware unchanged; `APP_FW_VERSION` stays `0.29.9-grbl-scara`.
- Added `SCARA_UI/tests/pattern_check.py` offline regression check, plus example assets
  `patterns/笑脸.svg` and `patterns/爱心.png`.

## 2026-06-13 v0.29.9 Exact waypoint preservation

- Removed the remaining 2 mm car-body corner fillets so every declared body
  control point is emitted as an exact G1 endpoint.
- Disabled default writing/polyline RDP simplification and 0.20 mm point
  filtering. Input writing points are retained down to G-code's 0.001 mm
  coordinate resolution, including the handwriting mouse-release endpoint.
- True arcs are always streamed as G2/G3 rather than rewritten into short G1
  chords.
- A rapid connector now exact-stops at the declared geometry start before the
  first drawing block; internal corners retain normal non-zero junction speed.
- Preview downsampling now preserves every marked geometry endpoint, and MCU
  segment preparation uses the original integer Cartesian endpoint for the
  first and final sample of every planner block.

## 2026-06-13 v0.29.8 Semantic text paths and short-block continuity

- Replaced Qt `toSubpathPolygons()` text extraction with semantic
  `MoveTo`/`LineTo`/`CurveTo` extraction. True font corners remain exact
  junctions, while only real Bézier curves are adaptively flattened.
- Increased the ARM planner from 32 to 48 blocks and starts continuous motion
  after collecting up to 12 look-ahead blocks, increasing the combined distance
  available for GRBL reverse/forward planning on short text-outline blocks.
- Added a dynamic tail look-ahead holdback: while at least 8 step segments are
  buffered, segment preparation retains the final unknown-successor planner
  block so newly streamed text blocks can still form a continuous junction.
- Extended junction-deviation limiting into physical SCARA joint space. Planner
  blocks retain start/end IK slopes, so a mild Cartesian glyph corner that
  reverses a motor is slowed before it can create an audible joint-speed shock.
- The UI now derives planner capacity from live `Bf.free + Q.used` status rather
  than relying on a fixed constant, keeping backpressure and Idle recovery
  correct across future planner-size changes.

## 2026-06-13 v0.29.7 Sharp-corner lazy writing stream

- Removed handwriting and text-outline corner fillets. Every retained writing
  control point is now emitted as a true `G1` endpoint. GRBL junction deviation
  selects a safe non-zero transition speed from the real corner angle, so the
  arm reaches the sharp point without replacing it with an arc or stopping.
- Replaced recursive RDP simplification with an iterative implementation so
  large handwriting/font contours cannot fail with Python recursion errors.
- Replaced the fully materialized writing command tuple with a counted lazy
  command stream. Only the normal 64-command sender window is generated ahead,
  regardless of total writing point count.

## 2026-06-13 v0.29.6 GRBL segment throughput and fault recovery

- Moved SCARA pulse-rate and joint-acceleration limiting ahead of look-ahead, so
  every planner block is constrained before reverse/forward recalculation.
- Removed the post-planner adjacent-PPS rejection loop. Dense step candidates
  are now shortened until the fixed-rate DDA can represent them without
  discarding the planner or latching a recoverable throughput condition.
- Restored tangent car, handwriting, and text fillets and removed the UI's
  per-segment feed/whole-path acceleration workaround.
- Restored a fixed 64-line/224-byte GRBL character-counting sender window.
  Instantaneous `Bf` RX-free reports remain diagnostic and no longer collapse
  long-stream throughput.
- Added `Rl` adaptive-rate-limit and `Pg` maximum-refill-gap diagnostics. The UI
  now establishes the cumulative `Pf` baseline and never clears an idle stream
  because of an old controller fault count.

## 2026-06-13 v0.29.5 Motion preflight and stream recovery

- Routed every formal motion job through a reusable full-path workspace preflight before profile, laser, or G-code transmission. Invalid points are blocked and reported with point coordinates and limit excess.
- Isolated `$H`/`$HS` as long-running commands: they are sent alone after `$X`/`M17` acknowledgements, polled without ordinary ACK timeout warnings, and protected from stale `error:15` replies.
- Added state-aware ACK recovery. A running/full planner may wait normally, while a dead serial worker, silent controller, or Idle controller with empty planner/segments and a stuck ACK clears the stream and restores UI control.
- Sampled routine TX/ACK terminal progress and immediately schedules another serial drain pass while RX data remains, preventing long jobs from starving the Qt event loop while preserving errors and periodic progress.
- Prevented a new formal task from replacing a pending/Run/Hold stream, and blocked manual G0/G1/G2/G3/`$J` text commands from bypassing the verified-preview motion entry.
- Added a latched planner segment-preparation fault counter (`Pf`) and increased adaptive segment retries so a rare IK/quantization failure can no longer silently clear motion without host diagnostics.

## 2026-06-13 v0.29.4 Continuous multi-block segment preparation

- Changed the MCU main loop to drain all currently acceptable G-code lines
  before preparing step segments, so short blocks see their successors before
  being planned as an apparent program end.
- Preserved the fixed entry-speed anchor when a predecessor has already moved
  from the planner into the step-segment FIFO.
- Changed short final block tails from a forced 5 ms duration to a variable
  duration of at most 5 ms, preserving the planned junction speed across line,
  arc, car, and writing geometry boundaries.
- Added small tangent G2/G3 fillets to decorative car, handwriting, and text
  contours so GRBL does not need to nearly stop at every sampled sharp vertex;
  ordinary explicit trajectory commands remain exact.

## 2026-06-13 v0.29.3 GRBL handwriting geometry and profile sync

- Changed handwriting and text-outline execution from sampled, per-point feed
  streams to compact Cartesian G0/G1/G2/G3 geometry with a constant programmed
  feed, leaving velocity planning to the MCU GRBL planner.
- Synchronized each formal UI motion task's selected maximum speed,
  acceleration, and junction deviation through `$110`, `$120`, and `$11`.
- Corrected the UI preview junction half-angle model to match GRBL's direct
  tangent convention.
- Kept exact-stop barriers only at writing stroke transitions and fixed dwell
  segment preparation to hold the last prepared endpoint instead of rewinding
  toward the currently executing segment target.

## 2026-06-12 v0.29.2 Formal homing and unified GRBL stream

- Fixed the no-motion-after-homing regression caused by rejecting the first
  quantized 5 ms joint segment against an impossible sub-pulse acceleration
  threshold.
- `$H` and `$HS` now remain unacknowledged until homing actually reaches
  `Done`; normal G-code consumption pauses while homing is active.
- Moved homing, PPR settings, unlock, enable, jog, and trajectory commands onto
  the same bounded character-counting `GcodeJob` path, eliminating stale `ok`
  replies from the removed out-of-band protocol.
- Removed remaining HOST_TIMED/binary pulse planner code and obsolete binary
  stress tools; long previews are decimated independently from the send stream.
- Added a UI-level regression test that completes homing and then exercises
  jog, G1/G2/G3, motor bring-up, and fixed trajectory controls.
- Added MCU TX-queue backpressure so an accepted G-code line can never lose
  its `ok/error` response when the eight-message DMA TX queue is full.
- Kept ordinary G-code blocked until the final homing ACK is queued and reset
  planner segment history on homing/coordinate resets.
- Removed the remaining UI checksum ACK compatibility, stale `ok` fallback,
  visual binary-path call, old host-segment queue alias, and pulse-motion
  planner wrappers. UI stop now uses GRBL Ctrl-X.

## 2026-06-12 v0.29.1 Homing command diagnostics

- Fixed simulated homing command routing: the UI now sends `HOME_SIM`, and the
  firmware also accepts `$H S` as simulated homing instead of treating it as
  real switch homing.
- Added real homing precheck for already-active HOME inputs; `$H` now refuses
  to start when PB0/PB1 are active and `STATUS` exposes `he=4` for that case.
- Added UI homing diagnostics: real homing requests `HOME_SENSOR` before `$H`,
  both homing buttons request `HOME_SENSOR`, `STATUS`, and `?` afterward, and
  the status panel displays `hs/he/en/pps/tgt/h`.
- Improved UI serial logging so system-level `OK ...` responses and
  `OK HOME_SENSOR ...` are not misreported as G-code ACK mismatches.
- Extended `verify_project.ps1` with static checks for the homing protocol and
  diagnostics path.

## 2026-06-12 v0.29.0 GRBL-SCARA four-layer streaming core

- Removed BinaryTraj, host-timed point upload, and the UI transport mode switch.
- Added lazy 64-command/224-byte character-counting streaming on a serial QThread.
- Added a 256-byte MCU RX ring, 32 Cartesian planner blocks, reverse/forward
  look-ahead, junction deviation, and 16 five-millisecond step segments.
- Added real G0/G1/G2/G3 geometry, I/J and R arcs, `$J` exact-stop jog, realtime
  hold/resume/reset/Jog Cancel, standard `ok`/`error:n`, and GRBL-style status.
- Moved SCARA inverse kinematics into segment preparation and added adaptive
  segment shortening for joint PPS limits.
- Replaced blocking STEP pulse delays with TIM1/TIM4 active-low one-pulse output.
- Added M4 speed-scaled laser power and exact-stop laser mode barriers.

## 2026-06-11 v0.28.0 Relay-cut laser lift and live power

- Enabled commissioned laser capability for the JQC3F relay wiring: PA2 high energizes the relay, PA7 remains active-high PWM.
- Changed laser control to OFF/PREP/MARK: ARM only authorizes machining, PREP energizes the relay with PWM off, and MARK keeps the relay on while applying the configured PWM duty.
- Added binary `LASER_PREP=0x0010` and ASCII `L2` for relay pre-energize dwell segments.
- Updated the UI laser control from a one-shot checkbox to a colored enable button, with live `LASER POWER` updates from the percentage input.

## 2026-06-11 v0.27.5 6400-PPR Cartesian pulse-event planning

- Replaced fixed 10 ms rounded joint targets with a shared Cartesian-error-aware pulse-event planner for connected jog, G1/G2/G3, teach, car and handwriting trajectories.
- Added width-16 beam search at half-pulse crossing times, exact firmware timed-DDA replay compression, laser/stop/reversal boundaries, trajectory-error metrics and a 600 segments/s sustained-rate refusal gate.
- Unified UI, firmware, monitor, test and stress-tool defaults at `6400 PPR`.
- Raised `APP_PARAM_FLASH_VERSION` to `6`; the first run restores all current source defaults.

## 2026-06-11 v0.27.4 Restore verified active-high laser PWM

- Restored active-high laser PWM after the active-low diagnostic produced higher idle laser power.
- Non-working states continue to stop TIM3 and clamp PA7 as a push-pull GPIO at 0V.
- Kept `APP_LASER_COMMISSIONED=0`; the relay remains required as the physical power cutoff before enabling laser operation.

## 2026-06-10 v0.27.3 Active-low laser PWM trial

- Reversed the laser PWM polarity for hardware diagnosis: PA7 high is now the configured off state and low is the active state.
- Kept `APP_LASER_COMMISSIONED=0`, so the diagnostic firmware cannot ARM or intentionally emit marking PWM.

## 2026-06-10 v0.27.2 Laser idle GPIO clamp

- Moved the first PA7/relay safe clamp before `HAL_Init()` and added the same clamp to the fatal-error handler.
- PWM is now stopped and physically disconnected from PA7 whenever the laser is idle, silent, disarmed or faulted. PA7 remains a push-pull GPIO at the configured off level; TIM3 reaches PA7 only after a real marking segment starts.
- Changed TIM3 post-initialization to leave PA7 in the safe GPIO mode, eliminating the previous alternate-function interval during startup.
- Direct STM32F103 GPIO-register switching makes segment-boundary shutoff faster and keeps the safety hardening within the available Flash.

## 2026-06-10 v0.27.1 Laser power-on fail-safe hardening

- Added an early boot-safe GPIO clamp so PA7 and the relay output are driven to their configured inactive levels before normal peripheral initialization.
- Added explicit active-high/active-low PWM mapping, a 1 second boot lockout, PWM-start readiness checks, and ARM rejection when the output is not ready.
- Added `APP_LASER_COMMISSIONED=0` as the safe default. Laser ARM and UI laser capability remain disabled until wiring, common ground, voltage and polarity have been verified with laser power disconnected.
- Clarified that power-on must be `0%/off`, not the 1.0% minimum marking power, and that direct-powered laser wiring cannot be made fail-safe by firmware alone.

## 2026-06-10 v0.27.0 Laser PWM and relay safety control

- Added 1 kHz laser PWM on `PA7/TIM3_CH2` and active-high relay control on `PA2`; both default off.
- Added centralized armed/ready/marking state with 100 ms relay settling, 1.0% default power, 5.0% software maximum, and automatic idle disarm.
- Added `LASER POWER`, `LASER ARM`, `LASER DISARM`, `LASER STATUS`, ASCII `L0/L1`, binary `LASER_MARK=0x0008`, and binary laser-status telemetry.
- Laser state changes now occur only when a segment actually starts or at timed-DDA ISR handoff; prefetched segments cannot turn the laser on early.
- STOP, ESTOP, reset, homing, G0 rapid moves, motion errors and completed tasks force PWM off and release the relay.
- Added one-shot UI laser enable, power selection, actual lower-controller status, silent connector suppression, and automatic UI reset.

## 2026-06-10 v0.26.3 GRBL-style execution split and realtime command bypass

- Removed integer-milliradian double rounding from host joint-to-pulse conversion. Motion targets now retain floating-point radians and use exact `2*pi` until the final absolute-pulse rounding.
- Unified host-timed motion generation, binary geometry checks and pulse-to-XY checks on the same high-precision conversion.
- Confirmed the GRBL-style ownership boundary: host performs look-ahead and time resampling; the MCU 10 kHz ISR only dispatches prefetched timed DDA segments.
- Binary `RUN` now requires `min(total_points, APP_BINARY_TRAJ_MIN_PREFILL)` queued points before execution starts.
- Split cumulative underrun diagnostics from the active consecutive-underrun timeout so an old recovered underrun cannot trigger a later false timeout.
- Replaced compiler-dependent volatile NOP pulse delays with Cortex-M3 DWT cycle-counted microsecond delays. The configured dual-axis worst-case blocking delay is now 24 us inside the 100 us control tick, guarded by a compile-time half-period budget.
- Added `IC`/`ic` status telemetry for the maximum observed 10 kHz control ISR cycles; the no-motion capability check rejects firmware reaching the 7200-cycle budget at 72 MHz.
- Changed DDA output to collect per-event axis step bits before emitting one grouped pulse, following GRBL's simultaneous step-port approach. Dual-axis pulse skew is reduced from one full 6 us pulse cycle to only the back-to-back GPIO write latency, and the configured worst blocking delay drops to 18 us.
- Updated binary trajectory stress tooling to accept the current `hz=10000` capability field and build feedback expectations from the controller-reported `ppr1/ppr2` values instead of a stale hard-coded 1600 PPR.
- Moved binary trajectory point preparation and stepper dispatch out of `BinaryTraj_Tick10kHz()` into `BinaryTraj_Loop()`, matching GRBL's prepare-in-main-loop/execute-in-ISR split. The 10 kHz binary tick now only maintains lightweight underrun timing diagnostics.
- Tightened active binary stress validation: feedback polling no longer starves refill, the refill loop runs at 5 ms, and the test now fails on any stream underrun, empty low-water mark, dispatch gap over one tick, or active-motion ISR peak at/above 7200 cycles.
- Corrected binary `min_buffer` semantics so the expected final drain after all points have been accepted does not masquerade as a streaming low-water event.
- Added `-HostTimed` active stress mode so the normal UI timed-segment FIFO and ISR handoff path is covered by the same underrun, endpoint, and ISR-cycle gates.
- Fixed the stress harness to wait for controller `Done` rather than merely an empty binary input queue before disabling axes; it now hard-fails if final software pulse position differs from the final uploaded target.
- Flash verification now defaults to automatic ST-Link/CMSIS-DAP probing, and binary stress supports `-Port AUTO`, matching the documented validation flow.
- Fixed shared PowerShell AUTO-port selection so a single `COMx` result remains an array element instead of being indexed as the first character `C`.
- Hardware-verified on COM12 with motor power disconnected: idle `IC=682`; 3000-point velocity stress reached `IC=2308`; final v0.26.3 1000-point HOST_TIMED stress reached `IC=3907`, with `underrun=0`, `max_gap=1`, `min_buffer=92`, and exact endpoint `(0,800)`.
- Added a GRBL-style atomic binary-trajectory diagnostic snapshot, marked ISR-shared run/state flags `volatile`, and removed the unused raw pointer getter that exposed mutable stepper state.
- Added a separate realtime-character queue so `?`, `!`, `~` and Ctrl-X bypass normal command backlog; the main loop now handles realtime input first and limits ordinary parsing to one line per pass so motion preparation remains responsive.
- Extended the no-motion burst check to require a realtime `?` status response before acknowledgements for ordinary commands received in the same UART burst.
- Added regressions for one-pulse conversion resolution, exact revolution scaling, timed-path duration fidelity, HOST flags, final pulse target and adjacent slice speed jumps.
- Verified Debug firmware build, project structure checks, 3000-segment interpolation simulation and UI planner/sender tests.

## 2026-06-09 v0.26.0 Buffered streaming and deterministic timed handoff

- Restored buffered binary trajectories as the normal UI transport.
- Added GRBL-style ASCII send-window accounting and separate sender strategies.
- Added planner, RX-byte, timed-segment FIFO, low-water and underrun diagnostics.
- Added ISR-local timed-segment handoff so queued segments do not restore interrupts inside the 10 kHz ISR.
- Normal UI trajectories now execute as approximately 10 ms timed joint segments instead of restarting acceleration at every geometry keypoint.
- Buffered motion starts after a 64-segment prefill, then refills every 10 ms through the MCU 128-point ring and stepper timed-segment FIFO.
- STOP now clears host pending/inflight data, binary refill state, active trajectory references, MCU trajectory data and prefetched step segments.
- Done/Error/Idle binary states immediately release the active host trajectory instead of leaving the UI motion-busy.
- UI checks `HOSTCAP` before binary upload and only sends timed segments when the controller advertises `binary_timed=1`; older firmware falls back instead of misreading duration ticks as PPS.
- Added non-motion capability and sender-window regression checks.

## 2026-06-04 v0.24.5 上位机仿真视野锁定为鼠标控制

### 完成

- 删除主 UI 仿真图在点动、前进/后退、轨迹预览和清空轨迹时自动重新缩放的行为。
- 仿真图只在首次初始化时自动适配一次工作区；后续视野比例和位置只由鼠标滚轮缩放、左键拖动控制。

### 验证

- Python 关键文件内存编译通过。
- `git diff --check` 通过。

## 2026-06-04 v0.24.4 上位机仿真铺满、真实速度监控与停止/急停拆分

### 完成

- 主 UI 仿真坐标纸改为铺满右侧绘图区：压缩 Figure 边距，保持真实坐标比例缩放/拖动，删除坐标纸图注，避免 legend 遮挡轨迹细节。
- `SCARA_UI/V_monitor.py` 删除速度/加速度低通滤波和最小 20 ms 截断：
  - 只使用下位机状态帧 `M:x,y` 换算后的真实反馈坐标。
  - 通过相邻真实反馈点反解关节角，再按真实接收时间差计算电机速度和加速度。
  - 横轴改为 10 s 滚动窗口，数据超过窗口后像流水一样向前推进，避免轨迹时间过长时堆叠在同一张图上。
- 主 UI 将原“紧急停止”拆分为两个按钮：
  - “停止（清除队列）”：清空上位机待发送队列，发送 `STOP`。
  - “急停（保留队列）”：保留上位机待发送队列，并把等待 ACK 的当前点放回队首，发送 `ESTOP`。
- 固件协议未修改，固件版本仍为 `0.24.1`；当前下位机 `ESTOP` 没有走 `MotionPlanner_Stop()` 清规划器，`STOP` 仍负责受控停止。

### 验证

- Python 关键文件内存编译通过：
  - `V_monitor.py`、`app_bootstrap.py`、`plotting.py`、`ui_mixin.py`、`motion_mixin.py`、`main_window.py`、`serial_mixin.py`。
- 检索确认主 UI 坐标纸不再调用 `ax.legend()`，速度监控文件不再包含 `alpha_v`、`alpha_a`、`last_v_filt`、`last_a_filt` 和“平滑滤波”逻辑。
- `SCARA_UI/tests/trajectory_planner_check.py` 通过，确认轨迹规划改动未受显示/监控更新影响。
- `SCARA_F103/tools/verify_project.ps1` 通过。
- 本次未执行烧录和 COM13 压力运动测试；改动限定在上位机 UI/监控与按钮队列语义。

## 2026-06-04 v0.24.3 上位机监控显示拆分

### 完成

- 删除主 UI 控键区右侧图中的速度图像，主 UI 只保留轨迹预览、已发送轨迹、下位机反馈轨迹和五连杆机构仿真。
- `SCARA_UI/main.py` 启动主 UI 后，同步启动 `SCARA_UI/V_monitor.py` 中的电机速度/加速度监控窗口。
- 速度/加速度监控数据只由下位机状态帧 `M:x,y` 换算得到：
  - 主 UI 收到状态帧后先做 MCU 坐标到 UI 坐标转换。
  - 再将真实反馈坐标以 `X... Y...` 形式送入 `MonitorWindow.process_new_data()`。
  - ACK 回显仍只用于通信校验，不作为速度/加速度监控数据源。
- `V_monitor.py` 保留 pyqtgraph 路径；若当前 Python 环境没有 `pyqtgraph`，自动退回 Matplotlib 绘图，避免主 UI 因缺依赖无法启动。

### 验证

- Python 内存编译通过：
  - `V_monitor.py`、`app_bootstrap.py`、`plotting.py`、`ui_mixin.py`、`main_window.py`、`serial_mixin.py`。
- 检索确认主 UI 已无 `speed_ax`、`speed_line`、`preview_f` 残留。
- `SCARA_UI/tests/trajectory_planner_check.py` 通过，确认轨迹规划改动未被监控显示拆分影响。
- `tools/verify_project.ps1` 通过，并检查 `SCARA_UI/V_monitor.py` 和 `SCARA_UI/tests/trajectory_planner_check.py` 均存在。

## 2026-06-04 v0.24.2 上位机轨迹规划更新

### 完成

- 固件协议保持 `0.24.1` 不变，下位机仍只接收 `G1 X Y F ;ID=... LIM=1`。
- 重做上位机速度规划：
  - 规划器改为真实几何段 + 累计弧长 + 梯形/三角速度曲线。
  - `G2/G3` 圆弧作为真实圆弧段规划，不再把圆弧离散点当作大量短折线拐角。
  - `运行速度` 明确为 `mm/s`，发送前换算为 G-code 的 `F mm/min`。
- 改造 UI 轨迹显示：
  - 点击直线、顺圆、逆圆、小车 1、小车 2 时，先显示同一份真实规划结果作为预览，再装载发送队列。
  - 绘图分层显示规划预览、已发送点、下位机状态帧反馈点和当前五连杆姿态。
  - 速度曲线显示规划器输出的真实 `F mm/min`，不做平滑美化。
  - 轨迹图支持鼠标滚轮缩放和左键拖动；运动刷新不再强制重置用户视野。
- 修正固定小车轨迹：
  - 小车轨迹 1 按 `SOURCE\小车轨迹1.png` 尺寸生成，左下基准点来自 UI 输入，宽 `120mm`、车身高 `24mm`、轮心 `(36,0)/(96,0)`、轮拱 `R12`、车厢高到 `48mm`。
  - 小车轨迹 2 按 `SOURCE\小车轨迹2.png` 尺寸生成，左下基准点来自 UI 输入，宽 `160mm`、车身高 `20mm`、轮心 `(32,0)/(128,0)`、轮拱 `R12`、车顶 `60mm` 宽。
- 新增 `SCARA_UI/tests/trajectory_planner_check.py`，检查 G1/G2/G3 速度连续性、小车尺寸和五连杆限位。
- `tools/ui_trajectory_stress.ps1` 扩展为五段压力测试：G1、G2、G3、小车轨迹 1、小车轨迹 2。

### 验证

- `SCARA_UI/tests/trajectory_planner_check.py` 通过：
  - G1 输出 `106` 点，G2 输出 `116` 点，G3 输出 `116` 点。
  - 小车轨迹 1 输出 `500` 点，小车轨迹 2 输出 `564` 点。
  - 所有路径通过五连杆限位预检查。
  - 相邻点速度满足规划加速度约束，圆弧中段不再出现周期性跌落。
- Python 内存编译通过：
  - `look_ahead.py`、`motion_mixin.py`、`plotting.py`、`ui_mixin.py`、`main_window.py`、`serial_mixin.py`、`trajectory_planner_check.py`。
- PowerShell AST 解析通过：
  - `tools/ui_trajectory_stress.ps1`。
  - `tools/verify_project.ps1`。
- `tools/verify_project.ps1` 通过，并检查新增 `SCARA_UI/tests/trajectory_planner_check.py` 存在。
- `tools/gcode_stream_check.ps1 -Port COM13` 通过：
  - `VERSION` 返回固件 `0.24.1`。
  - `HOSTCAP` 返回 `host_plan=1 host_limit=1 mcu_soft_limit=0`。
  - 状态为 `Idle/Q:0/E:0`。
- 新版 `tools/ui_trajectory_stress.ps1 -Port COM13 -Count 3000 -FeedMmMin 900` 通过：
  - G1、G2、G3、小车轨迹 1、小车轨迹 2 各 600 点。
  - 五段均显示 `PATH SAFE`，并通过 ACK 精确回显。
  - 最终 `UI_TRAJECTORY_STRESS PASS total=3000`。
- 压力测试后再次执行 `tools/gcode_stream_check.ps1 -Port COM13` 通过：
  - `HEARTBEAT err=0 motion=Idle gbuf=32,0`。
  - 状态帧 `Idle/Q:0/E:0`，`A1/A2` 均已释放。
  - 最终软件位置为 `M:0.049,201.151`、`P:14,-14`，这是小车闭合轮廓后的量化位置，不是通信错误。

### 原因

- 原规划器将圆弧采样点交给折线 junction 限速，导致圆弧内部反复降速，真实 `F` 序列出现锯齿。
- 原 UI 每个轨迹点都 `ax.clear()` 并重绘工作空间、历史轨迹和姿态，使规划点发送频繁时视觉抖动明显。
- 原小车轨迹只是从当前点到起始点再走一条直线，和 `SOURCE` 图纸尺寸不一致。

## 2026-06-04 v0.24.1

### 完成

- 修正五连杆构型分支：实际机构不是交叉五连杆，而是左右对称的并联五连杆。
  - 上位机逆解从左臂 `-`、右臂 `+` 改为左臂 `+`、右臂 `-`。
  - 固件默认 `APP_SCARA_IK_LEFT_ELBOW_SIGN` 改为 `1`，`APP_SCARA_IK_RIGHT_ELBOW_SIGN` 改为 `-1`。
  - UI 绘图将随新的逆解分支显示为两侧主动臂向外上方展开、被动臂在上方末端汇合的非交叉构型。
- 重新计算默认软件零点 `UI X=75, Y=220`：
  - 左关节约 `128.984 deg`，右关节约 `51.016 deg`。
  - 固件零点偏置更新为 `APP_MOTOR1_ZERO_MRAD=2251`、`APP_MOTOR2_ZERO_MRAD=890`。
  - `APP_PARAM_FLASH_VERSION` 提升到 `4`，使旧交叉构型零点参数失效。
- 同步更新 `tools/verify_project.ps1` 和 `tools/ui_control_matrix_check.ps1` 的分支与零点检查。
- 修复上位机轨迹规划按钮只显示“参数错误”的问题：
  - `plan_trajectory()` 不再裸 `except`，会区分输入数字错误、半径错误、路径预检查失败和规划异常。
  - `G1 直线` 会从当前点到目标点生成直线点流。
  - 新增 `G3 逆圆`；`G2 顺圆` 和 `G3 逆圆` 都由上位机按半径离散成连续 `G1` 点流，下位机仍只执行 `G1`。
  - 轨迹发送前遍历整条路径，检查 XY 工作空间、左右基座距离、M1/M2 角度、主动臂是否交叉、主动臂是否低于基座线；超限时显示具体轴/结构、限值和超出量。
- 新增 `tools/ui_trajectory_stress.ps1`：
  - 按 UI 轨迹按钮逻辑生成一段直线、一段 G2 顺圆、一段 G3 逆圆。
  - 发送前执行同一套非交叉五连杆路径预检查。
  - 再转换为 MCU 中点坐标逐条发送 `G1`，校验 `ok seq/cs/line`。

### 验证

- 离线运动学抽检通过：
  - `UI X=75,Y=220`、四方向点动点、默认轨迹点和小车路径端点均可逆解。
  - 正解回代误差为 `0`。
  - 左肘点 X 坐标始终小于右肘点 X 坐标，确认绘图为非交叉构型。
- `tools/verify_project.ps1` 通过：
  - 固件版本为 `0.24.1`。
  - 固件零点偏置为 `2251/890`。
  - 固件分支为左 `+`、右 `-`。
  - 参数页版本为 `4`。
- 已重新烧录当前 `build/Debug/SCARA_F103.elf`，OpenOCD verify 输出 `** Verified OK **`。
- 烧录后 `tools/gcode_stream_check.ps1 -Port COM13` 通过：
  - `VERSION` 返回 `0.24.1`。
  - `HEARTBEAT` 返回 `err=0 motion=Idle gbuf=32,0`。
  - 状态帧为 `Idle/Q:0/E:0`，`P:0,0` 对应 MCU `M:0.053,219.966`，符合新零点量化误差。
- 上位机模拟点击验证通过：
  - `G1 直线`、`G2 顺圆`、`G3 逆圆` 均能从当前点到目标点生成可发送轨迹。
  - 半径过小会被拦截。
  - 超出工作区/关节/结构限制的目标会被拦截并输出错误日志。
- `tools/ui_trajectory_stress.ps1 -Port COM13 -Count 300 -FeedMmMin 600` 通过：
  - G1/G2/G3 各 100 点。
  - 三段均显示 `PATH SAFE`。
  - 终态 `Idle/Q:0/E:0`。
- `tools/ui_trajectory_stress.ps1 -Port COM13 -Count 3000 -FeedMmMin 900` 通过：
  - G1 直线 1000 点、G2 顺圆 1000 点、G3 逆圆 1000 点。
  - 三段均通过路径预检查和 ACK 精确回显。
  - 最终 `UI_TRAJECTORY_STRESS PASS total=3000`，终态 `Idle/Q:0/E:0`。
- 压力测试后 `tools/gcode_stream_check.ps1 -Port COM13` 通过：
  - `HEARTBEAT err=0 motion=Idle gbuf=32,0`。
  - 状态帧 `Idle/Q:0/E:0`，电机已释放。
- 烧录后再次执行 `tools/ui_control_matrix_check.ps1 -Port COM13` 通过：
  - 覆盖四方向点动、M1/M2 单轴点动、默认轨迹、小车路径、急停、清错和释放电机。
  - 结束后 `tools/gcode_stream_check.ps1 -Port COM13` 通过，状态为 `Idle/Q:0/E:0`，`A1/A2` 均已释放。
  - 因最后一段为小车路径，最终软件位置停在 `M:159.366,200.286`、`P:-237,-203`，不是零点；这是测试路径终点，不是通信错误。

### 原因

- v0.24.0 虽然统一了上下位机分支，但统一到的是交叉构型；用户提供的机构图和真实机械结构是对称并联构型。
- 如果继续使用交叉分支，UI 动画会显示左右连杆交叉，固件 `ZERO` 后的脉冲零点也会对应错误姿态。
- 注意：步进电机是开环系统，烧录或 `ZERO` 只改变软件坐标解释，不会自动把实体杆件移动到新对称零点；真实点动前需要先确认机械姿态与 `UI X=75,Y=220` 的对称零点一致。

## 2026-06-04 v0.24.0

### 完成

- 按本仓库 `SOURCE` 五连杆资料、MathWorks five-bar robot 参考页和本地 GRBL 1.1h 工程重新审查上下位机分工：
  - 轨迹仍由上位机规划和限位，MCU 只做 G-code 接收、几何可达性兜底、队列执行和状态回传。
  - 正逆解统一为五连杆左臂 `-`、右臂 `+` 分支，并在正解时选择 Y 更高的圆交点，避免 UI 与固件使用不同构型分支。
  - 参考资料中的连杆公式只取运动学形式，不直接套用资料中的样机尺寸；当前项目继续使用 `base=150 mm`、`active=160 mm`、`passive=200 mm`。
- 修复上下位机 X 坐标原点不一致：
  - `SCARA_UI` 显示/操作仍使用左电机为原点的 UI 坐标，工作区中心在 `X=75 mm`。
  - 固件 G-code 和状态帧使用双电机中点为原点的 MCU 坐标。
  - 上位机发送 G-code 时自动执行 `X_mcu = X_ui - 75`，读取 `M:x,y` 或 ACK 回显时自动执行 `X_ui = X_mcu + 75`。
- 统一软件零点：
  - UI 安全零点改为 `X=75.0, Y=220.0`，保证上下左右 10 mm 点动都可达。
  - 固件零点偏置更新为 `APP_MOTOR1_ZERO_MRAD=233`、`APP_MOTOR2_ZERO_MRAD=2908`。
  - `APP_PARAM_FLASH_VERSION` 提升到 `3`，强制旧 Flash 参数失效，避免继续使用旧零点。
- 新增 `SCARA_F103/tools/ui_control_matrix_check.ps1`：
  - 1:1 复刻 UI 控键串口行为，覆盖 VERSION/HOSTCAP、清错、使能、软零、四方向点动、M1/M2 点动、默认轨迹、小车路径、急停和释放电机。
  - 对 `error:8` 诊断为发送太快或 pending/buffer 忙，对 `error:15` 诊断为运动拒绝、逆解失败、电机未使能、急停或错误位未清。
- 固件版本号更新为 `0.24.0`，并同步更新自检脚本、坐标零点检查和文档。

### 验证

- `tools/verify_project.ps1` 通过：
  - Debug 构建通过。
  - 固件大小 `51052 <= 64512 bytes`。
  - 固件版本、零点偏置、参数页版本和控键矩阵脚本检查通过。
- `SCARA_UI` 关键 Python 文件 `py_compile` 通过。
- 已烧录 `0.24.0` 到 STM32F103，OpenOCD verify 通过。
- 烧录后基础串口检查已通过一次：`VERSION` 返回 `0.24.0`，`HOSTCAP` 正常，状态帧 `E:0`，零点状态约为 MCU `M:0.115,220.000`，对应 UI `X=75.115, Y=220.000`。
- `tools/ui_control_matrix_check.ps1 -Port COM13` 真实硬件通过：
  - 覆盖 `VERSION`、`HOSTCAP`、`WATCHDOG OFF`、`CLEAR_ERROR`、`ENABLE 1`、`ZERO`。
  - 覆盖前进、后退、左移、右移、M1+/M1-、M2+/M2-、默认轨迹、小车轨迹起点/终点。
  - 每个运动指令均收到 `ok seq/cs/line` 精确回显，运动后回到 `Idle/Q:0/E:0`，未出现 `error:8` 或 `error:15`。
  - 最后 `ESTOP`、`CLEAR_ERROR`、`ENABLE 0` 通过。
- `tools/host_planned_stream_stress.ps1 -Port COM13 -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion -QuietLines` 通过：
  - `ok=3000`。
  - 总耗时约 `36.3 s`。
  - 终态 `<Idle|...|Bf:32,16|Q:0|E:0|...>`。
- 故意 5 行 burst 不等待 ACK 的小测试全部被下位机缓冲并正常 ACK；结合固件源码，`error:8` 只会在 32 段规划队列已满且已有 1 条 pending 未释放时触发。
- 压力测试后 `tools/gcode_stream_check.ps1 -Port COM13` 通过：`HEARTBEAT err=0 motion=Idle gbuf=32,0`，状态帧 `Idle/Q:0/E:0`，电机已释放。
- 最终再次执行 OpenOCD `program build/Debug/SCARA_F103.elf verify reset exit`，烧录和校验均通过，输出 `** Verified OK **`。
- 最终烧录后 `tools/gcode_stream_check.ps1 -Port COM13` 通过：
  - `VERSION` 返回 `0.24.0`。
  - 初始状态回到 MCU `M:0.115,220.000`、`P:0,0`、`Idle/Q:0/E:0`。
  - `HEARTBEAT` 返回 `err=0 motion=Idle gbuf=32,0`。

### 根因

- “点动只运行一下随后报错”不是单一问题：
  - v0.23.2 已修复系统 `OK ...` 被旧上位机误当作 G-code `ok seq/cs/line` 的 ACK，导致队列推进错乱。
  - v0.24.0 继续修复 UI/MCU 坐标原点和正逆解分支不一致，避免目标点在 UI 看起来可达、但下位机按另一套坐标或构型判断后返回运动拒绝。
- 当前实测结论：
  - 不是点数太多：3000 点密集流通过。
  - 不是正常 UI 节奏太快：逐条等待 ACK 时没有 `error:8`。
  - 不是五连杆正逆解分支错误：控键矩阵和轨迹点均被 MCU 接受并执行。
  - 真正风险是上位机若不区分系统 `OK` 和 G-code ACK，或直接用 UI 坐标当 MCU 坐标发给串口，就会造成队列错乱或 `error:15`。

## 2026-06-04 v0.23.2

### 完成

- 修复上位机点动时“下位机只动一下/随后报错”的通信状态机问题：
  - `SCARA_UI` 现在只把带 `ok seq=<n> cs=<hex> line=<原始行>` 且与最近一条 G-code 完全匹配的响应当作运动 ACK。
  - `OK ENABLE 1`、`OK CLEAR_ERROR`、`OK ZERO`、`OK VERSION` 等系统响应只记录日志，不再误触发点动队列继续发送。
  - 点动发送第一条 G-code 前自动发送 `CLEAR_ERROR` 和 `ENABLE 1`，避免刚烧录、刚连接或上一轮错误后电机未使能导致 `error:15`。
- 调整上位机错误处理：
  - `error:8` 仍按下位机 pending/buffer 忙处理，暂停发送并等待状态恢复。
  - `error:15` 改为暂停当前点动队列、查询 `ERRORS` 和 `?` 状态，不再自动发送 `ESTOP`，避免把可恢复的运动拒绝升级成急停锁定。
- 固件版本号更新为 `0.23.2`，并同步更新自检脚本和 `Control.md`。
- 修正 `tools/verify_project.ps1` 的文件清单，移除当前工程已不再保留的 `tools/robot_upper_sim` 检查项，避免自检被历史文件要求误判失败。

### 原因

- 下位机协议中普通系统命令返回 `OK ...`，正式 G-code 返回 `ok seq/cs/line`。
- 旧上位机逻辑只判断响应是否以 `ok` 开头，可能把系统 OK 当成点动 ACK。
- STM32F103 固件上电后步进电机默认未使能；若上位机直接发送点动 G-code，下位机接收/入队会返回 `ok`，但实际启动运动块时会因为 `STEPPER_ERR_DISABLED` 返回 `error:15`。

## 2026-06-03 v0.23.1

- Changed trajectory limit ownership to the host side:
  - Added `APP_HOST_OWNS_LIMIT_CHECKS = 1`.
  - Disabled MCU software pulse range and joint angle range checks for formal streamed trajectory limits.
  - Kept MCU-side G-code syntax checks, geometric IK feasibility, enable/ESTOP checks, homing input and status reporting.
- Updated `HOSTCAP` to report `host_limit=1 mcu_soft_limit=0`.
- Improved homing restart behavior:
  - `$H` can be sent again after `HS:Done` or `HS:Error`.
  - `$H` still returns `error:5` while homing/search/backoff is active, because the first `ok` only means the homing state machine started.
- Updated `Control.md` and `Work.md` with host-side limit responsibilities and the `error:5` explanation.

## 2026-06-03 v0.23.0

### Done

- Re-scoped the project for course-design goals:
  - reliable serial receive/ACK
  - high-frequency host-planned G-code stream testing
  - complete queue drain and status feedback
- Disabled the communication watchdog by default:
  - `APP_COMM_WATCHDOG_DEFAULT_MS = 0`
  - this prevents confusing `err=4` during control-board-only testing
  - `WATCHDOG ON timeout_ms` remains available for later experiments
- Removed old or inactive test scripts and launchers:
  - legacy pulse protocol test
  - old COM9 debug scripts
  - older pointcloud/line-arc/short-segment split tests
  - old Python/PowerShell smoke scripts
- Kept the focused course-design test path:
  - `tools/serial_link_check.ps1`
  - `tools/gcode_stream_check.ps1`
  - `tools/host_planned_stream_stress.ps1`
  - `Run_COM13_HostPlanned_3000.bat`
- Added Chinese comments to the maintained `UserApp` modules and test scripts.
- Rewrote `Control.md` as a concise Chinese course-design build/flash/serial-test guide.
- Added protocol character/field explanations for direct serial-terminal debugging.
- Added PyQt upper-computer simulator under `tools/robot_upper_sim`:
  - host-side interpolation
  - GRBL-style forward/reverse feed planning
  - XY trajectory preview
  - feed curve plotting
  - optional serial G-code streaming and ACK observation

### Notes

- `err=4` means `STEPPER_ERR_COMM_TIMEOUT`, not a required trajectory error. With default watchdog off, it should not appear unless `WATCHDOG ON ...` is enabled manually.

### Verification

- `cmake --build --preset Debug` passed.
- `tools/verify_project.ps1` passed.
- Flashed v0.23.0 to the connected STM32F103 board through CMSIS-DAP, OpenOCD verified OK.
- `tools/gcode_stream_check.ps1 -Port COM13` passed:
  - `VERSION` returned `0.23.0`
  - `HOSTCAP` returned `role=pulse_executor host_plan=1 host_limit=1`
  - status showed `E:0`
- `tools/host_planned_stream_stress.ps1 -Port COM13 -Count 20 -FeedMin 500 -FeedMax 1800 -EnableMotion` passed with live TX/RX/MATCH output.
- `tools/host_planned_stream_stress.ps1 -Port COM13 -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion -QuietLines` passed:
  - `ok=3000`
  - final status reached `Idle`
  - planner queue reached `Q:0`
  - error field stayed `E:0`

## 2026-06-03 v0.22.2

### Done

- Re-scoped the firmware as a host-planned pulse executor:
  - upper computer owns trajectory generation, speed planning, and formal limit judgment
  - MCU owns receive, ACK echo, buffering, pulse execution, status feedback, and safety backstop
- Removed MCU corner-speed scaling from the G-code enqueue path.
- Kept only thin adjacent-block handoff:
  - if the host sends continuous G1 blocks with planned F values, MCU can use the lower adjacent pps as execution exit speed
  - MCU no longer decides corner slowdown from XY vector angle
- Added `HOSTCAP` command:
  - reports `role=pulse_executor`
  - reports `host_plan=1 host_limit=1`
  - documents that comments are ignored by motion parsing but echoed in ACK
- Added host-planned high-frequency stream test:
  - `tools/host_planned_stream_stress.ps1`
  - script-side simulates host workspace/feed checks
  - sends `G1 X... Y... F... ;ID=xxxx LIM=1`
  - verifies `ok seq/cs/line` for every point
- Rewrote `Work.md` around the new division of responsibility.

### Notes

- The MCU soft limit remains as a safety backstop only. Formal trajectory validity must be decided by the upper computer before sending.
- The test comments after `;` are a reserved host metadata interface. They are echoed for verification but do not affect motion.

## 2026-06-03 v0.22.1

### Done

- Reviewed the v0.22 continuous-trajectory path and fixed the main velocity-planning gap:
  - previous code still treated each block as a zero-exit-speed move
  - short G1 blocks therefore braked toward zero at every block boundary
- Added blended absolute moves:
  - `MotionPlanner_MoveAbsBlend()`
  - `Stepper_MoveAbsBlend()`
  - each queued G1 block can carry per-axis exit pps calculated from the next block
- Added a small G1 blend start delay:
  - first G1 waits until at least two planner blocks are buffered, or until `APP_GCODE_BLEND_START_DELAY_MS`
  - this lets the MCU calculate the first block exit speed before motion starts
- Fixed idle pulse accounting:
  - idle axes no longer keep outputting PWM pulses while `current_pps` ramps down
  - this avoids extra software pulse counts outside an active move block
- Updated line+arc stress tooling:
  - `tools/gcode_line_arc_stress.ps1` now supports `-VaryFeed -FeedMin ... -FeedMax ...`
  - straight-line feed ramps across the line
  - arc feed varies periodically across the circle
- Added `Work.md` for upper-computer integration responsibilities and protocol details.

### Notes

- This is still a lightweight C8T6 planner, not a full GRBL segment executor. It now avoids intentional per-segment zero-speed braking, but true industrial continuous motion still depends on keeping the planner buffered and later adding a deeper precomputed step-segment queue if required.

## 2026-06-03 v0.22.0

### Done

- Increased queue depths for G-code streaming:
  - planner blocks: 32
  - RX line queue: 16
  - TX response queue: 8
- Added lightweight continuous segment handoff:
  - G-code can load the next move when the previous segment is complete even if axis speed has not fully decelerated to zero
  - segment completion no longer forces an immediate `axis_stop_now()`
  - simple corner speed scaling is applied from adjacent XY vectors
- Added automatic MCU homing controller for `$H`:
  - axis 1 searches first, then axis 2
  - default search direction is `-1` for both axes
  - home state appears in status as `HS:<state>`
- Added `HEARTBEAT seq` for upper-computer polling.
- Expanded status push with planner queue, RX free, axis enable/run/speed, home state, and switch bits.
- Added short-segment 3000-point stress tooling:
  - `tools/gcode_short_segment_stress.ps1`
  - `Run_COM13_Gcode_ShortSegment_3000.bat`

### Verification

- `cmake --build --preset Debug` passed.
- `tools/verify_project.ps1` passed.
- Current memory:
  - RAM: 8176 B / 20 KB, 39.92%
  - FLASH: 50784 B / 63 KB, 78.72%
- Flashed via CMSIS-DAP and verified with OpenOCD.
- `tools/gcode_stream_check.ps1 -Port COM13` passed, including `HEARTBEAT 42`.
- `tools/gcode_pointcloud_stress.ps1 -Port COM13 -Count 3000 -Feed 1200 -EnableMotion` passed in 32.6 s.
- `tools/gcode_line_arc_stress.ps1 -Port COM13 -LinePoints 1000 -ArcPoints 2000 -Feed 1200 -EnableMotion` passed in 28.4 s.
- `tools/gcode_short_segment_stress.ps1 -Port COM13 -Count 3000 -Feed 900 -EnableMotion` passed in 28.9 s and drained to `Idle/Q:0`.

## 2026-06-03 v0.21.0

### Done

- Trimmed F103 firmware build for the G-code path:
  - removed old pulse protocol from the build
  - removed old teach module from the build
  - removed old queued trajectory module from the build
  - reduced legacy ASCII protocol to essential debug and safety commands
- G-code acceptance responses now echo verification data:
  - `ok seq=<n> cs=<hex> line=<received line>`
- Added high-volume point-cloud stream test:
  - `tools/gcode_pointcloud_stress.ps1`
  - `Run_COM13_Gcode_Pointcloud.bat`
- The stress script validates that each acknowledged G-code line has the expected checksum and exact line echo.
- Removed retired `pulse_protocol`, `trajectory`, and `teach` source/header files from `UserApp`.
- Reclaimed the unused teach Flash page:
  - application Flash region is back to 63 KB
  - the final 1 KB parameter page remains reserved at `0x0800F800`
- `$G` now sends a normal stream `ok seq=<n> cs=<hex> line=$G` after the modal report.
- Fixed a short-segment MOVE stall where deceleration could stop at zero speed with `remaining_pulse > 0`, leaving the planner buffer full forever.
- Verified on hardware over COM13 at 115200:
  - `gcode_stream_check.ps1` passed
  - `gcode_pointcloud_stress.ps1 -Count 3000 -Feed 1200 -EnableMotion` passed with 3000 echoed ACKs in 37.4 s
- Optimized joint/pulse conversion to integer arithmetic to reduce F103 soft-float work in common kinematic conversions.
- Added disabled-motion protection:
  - real motion while an axis is disabled returns an error instead of entering a non-draining MOVE state
  - `ERRORS` now reports `disabled=1` when this occurs
- Added long straight + full-circle arc 3000-point stress tooling:
  - `tools/gcode_line_arc_stress.ps1`
  - `Run_COM13_Gcode_LineArc_3000.bat`
- Verified the new line + arc stress on hardware over COM13 at 115200:
  - `gcode_line_arc_stress.ps1 -LinePoints 1000 -ArcPoints 2000 -Feed 1200 -EnableMotion`
  - passed with 3000 echoed ACKs in 29.0 s
  - final status reported planner free and `Err:0`

### Notes

- Default point-cloud script keeps motors disabled and verifies receive/parse/ack behavior. Add `-EnableMotion` only when the machine is physically safe.

## 2026-06-03 v0.20.0

### Done

- Added a local GRBL-style G-code stream layer without directly importing the full GRBL AVR codebase.
- Added send-response flow control:
  - accepted G-code returns `ok`
  - invalid G-code returns `error:<code>`
  - planner-full input is retained and acknowledged later when space is available
- Added an 8-block G-code planner queue for `G0/G1 X/Y/F`.
- Added modal support for:
  - `G90/G91`
  - `G20/G21`
  - `G4 P`
  - `M0/M2/M30`
  - `$G`
  - `$X`
  - `$H`
- Added real-time character handling for:
  - `?`
  - `!`
  - `~`
  - `Ctrl-X`
- Added automatic 5 Hz GRBL-style status push messages:
  - `<Idle|MPos:x,y|Pulses:p1,p2|Bf:planner_free,rx_free|Err:n|Home:h1,h2>`
- Added G-code stream test helper:
  - `tools/gcode_stream_check.ps1`
  - `Run_COM9_Gcode_Check.bat`

### Notes

- This version still uses the existing stepper output backend. It adds the GRBL-style protocol and planner shell first; a fuller GRBL segment/Bresenham stepper backend should be the next optimization if continuous short-segment motion still start-stops.
- Flash is near the C8T6 application limit. If more GRBL features are added, old MOVL/TEACH/legacy pulse helpers should be trimmed.

## 2026-06-02 v0.19.0

### Done

- Pulse protocol signed-field parsing is now explicit and supports negative `p1/p2` absolute pulse targets.
- Added home microswitch inputs:
  - `HOME1`: PB0
  - `HOME2`: PB1
  - default wiring is active-low with internal pull-up
- `STATUS` and `QSTAT` now report `home=h1,h2`.
- Added `HOME_SENSOR` command to read switch state.
- `HOME` now performs sensor-confirmed zeroing only when both home switches are active.

### Notes

- The microswitch datasheet in `SOURCE/C231409_BEBDE6DE725C7F41F42AEF4FAD962318.pdf` describes V-series miniature basic switches with SPDT/SPST variants. For the current firmware default, wire switch COM to GND and NO to PB0/PB1.
- If only one shared home switch is installed, connect the unused input to GND during homing or adjust `HomeSensor_AllActive()` for single-switch behavior.

## 2026-06-02 v0.18.0

### Done

- Changed `STATUS` to a shorter debug-safe response using 32-bit pulse text output.
- Added `QSTAT` as a compact status alias for scripts and upper-computer polling.
- Updated the COM9 motion script so high-frequency motion testing does not depend on the long `STATUS` path.
- `Run_COM9_Motion_Debug.bat` now sends 40 continuous tiny absolute pulse frames at 5000 pps after confirmation.

### Notes

- If `VERSION` stops responding after an old `STATUS` test, reset the board and flash v0.18.0 before running motion validation again.

## 2026-06-02 v0.17.0

### Done

- Pulse protocol success responses now echo command and include motor debug feedback:
  - current pulse position
  - target pulse position
  - busy flag
  - combined error bits
- Added automatic COM9 serial debug script:
  - `tools/auto_com9_debug.ps1`
  - `Run_COM9_Auto_Debug.bat`
- Added opt-in COM9 motion debug launcher:
  - `Run_COM9_Motion_Debug.bat`
- The automatic COM9 test validates:
  - `VERSION`
  - `PING`
  - `STATUS`
  - bad checksum rejection
  - software zero ACK
  - pulse status fields
- `tools/auto_com9_debug.ps1` can now run:
  - burst `PING/STATUS` communication testing
  - high-frequency tiny absolute pulse trajectory-frame testing
  - ACK target-feedback validation for each motion point
  - optional `ERR BUSY` retry for continuous point streaming

### Notes

- The automatic COM9 debug path does not send motion unless `-MotionTest` or `-HighFreqMotionTest` is explicitly provided to the PowerShell script.
- The double-click motion launcher asks for confirmation before enabling motors or sending trajectory points.

## 2026-06-02 v0.16.0

### Done

- Changed USART1 and all serial test scripts back to `115200` baud.
- Added progress messages to `serial_link_check.ps1` after port open:
  - waiting for banner
  - sending `VERSION`
  - sending repeated `PING`
  - sending `STATUS`
- Kept the double-click launcher:
  - `Run_Serial_Test.bat`

### Debug Notes

- If the launcher appears to stop after `PASS port opened`, wait for the following progress lines. The script waits briefly for boot/banner text before sending commands.
- A pulse move with `speed=1` pps and a target such as `1512` pulses can keep the motor busy for about 1512 seconds. This does not mean the serial link is stuck, but `STATUS` will report the controller as busy.
- After flashing v0.16.0, use `115200` baud. Older v0.15.0 firmware still uses the previous configured baud until reflashed.

## 2026-06-02 v0.15.0

### Done

- Slimmed the TIM2 interrupt trajectory path:
  - `Trajectory_Tick1kHz()` now only sets a next-segment pending flag.
  - `Trajectory_Loop()` runs in the main loop and calls `MotionPlanner_MoveAbs()`.
- Added the formal pulse-controller text protocol:
  - `<mode,p1,p2,speed,checksum>`
  - `mode=0`: software zero/calibration
  - `mode=1`: absolute pulse target move
  - `mode=9`: emergency stop
- The pulse protocol uses an 8-bit ASCII payload sum, represented as two uppercase hex digits.
- `STATUS` now reports:
  - `pulse_proto=1`
  - `pulse_mode`
  - `pulse_err`
- Removed the v0.14 binary protocol source and 256-sample binary trajectory buffer to recover Flash/RAM.
- Added pulse protocol serial helper:
  - `tools/pulse_protocol_check.ps1`

### Verification

- `cmake --build --preset Debug` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 57060 bytes
  - application limit: 63488 bytes
- Current memory usage:
  - RAM: 7120 B / 20 KB, 34.77%
  - FLASH: 57060 B / 62 KB, 89.88%

### Notes

- `Trajectory_Tick1kHz()` no longer contains `MotionPlanner_MoveAbs()`.
- v0.15 treats the board primarily as a pulse controller; the upper computer owns trajectory planning.

## 2026-06-01 v0.14.0

### Done

- Added the first formal binary trajectory protocol beside the ASCII debug protocol.
- Binary frames use:
  - header `A5 5A`
  - protocol version
  - frame type
  - sequence
  - payload length
  - payload
  - CRC16
- Added binary commands:
  - `HELLO`
  - `STATUS`
  - `STOP`
  - `ESTOP`
  - `CLEAR_ERROR`
  - `TRAJ_BEGIN`
  - `TRAJ_CHUNK`
  - `TRAJ_VALIDATE`
  - `TRAJ_COMMIT`
  - `TRAJ_RUN`
  - `TRAJ_ABORT`
- Added binary ACK/NACK responses with standardized error codes.
- Added 256-sample binary trajectory buffer using host-generated absolute pulse samples:
  - `int32 p1_abs`
  - `int32 p2_abs`
  - `uint16 dt_ms`
  - `uint16 flags`
- Added binary trajectory state machine:
  - `IDLE`
  - `UPLOADING`
  - `VALIDATED`
  - `READY`
  - `RUNNING`
  - `DONE`
  - `ERROR`
  - `ESTOP`
- `STATUS` now reports binary trajectory state and upload progress.
- USART1 default baud rate is now `460800` for practical trajectory upload speed.
- Added PC-side binary smoke-test tool:
  - `tools/binary_protocol_smoke.py`

### Performance Notes

- ASCII `<mode, p1, p2, speed, checksum>\n` style packets remain suitable for manual debug, calibration, and emergency commands.
- Formal multi-vector trajectories use binary frames because text parsing and decimal conversion are expensive on STM32F103C8T6 and waste serial bandwidth.
- Current binary trajectory execution still uses the existing stepper output backend. The protocol and preload/CRC/state-machine path are ready; hardware-exact pulse accounting remains the next control-layer upgrade before production trajectory claims.

### Verification

- `cmake --build --preset Debug` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 60584 bytes
  - application limit: 63488 bytes
- Current memory usage:
  - RAM: 10696 B / 20 KB, 52.23%
  - FLASH: 60584 B / 62 KB, 95.43%

## 2026-06-01 v0.13.0

### Done

- Reworked serial RX from a single ready-line buffer to an 8-line queue.
- Reworked serial TX from a single DMA buffer to a 4-message transmit queue.
- Main loop now drains all queued RX lines each pass, so back-to-back commands are not overwritten by later lines.
- Added serial queue counters in `STATUS`:
  - `rx_ov`
  - `tx_drop`
  - `tx_q`
- `PARAM_SAVE` and `TEACH_SAVE` now reject writes while motion or trajectory replay is active:
  - `ERR PARAM_SAVE_BUSY`
  - `ERR TEACH_SAVE_BUSY`
- Added `Stepper_GetStateSnapshot()` and moved protocol/teach 64-bit position reads to interrupt-protected snapshots.
- Updated movement command setup to use a snapshot of the current 64-bit position before preparing new absolute targets.

### Formal-Control Boundaries

- PWM position tracking is still software-estimated from commanded pulse rate; before formal trajectory control it should be upgraded to exact pulse accounting from timer update events, compare callbacks, or a hardware counter path.
- `MOVL` is still segmented point-to-point Cartesian motion; it is not continuous interpolation with blended velocity yet.
- `HOME` still moves to software pulse zero; true homing requires real origin/limit switch inputs and a switch-seeking state machine.

### Verification

- `cmake --build --preset Debug` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 55256 bytes
  - application limit: 63488 bytes
- Current memory usage:
  - RAM: 7120 B / 20 KB, 34.77%
  - FLASH: 55256 B / 62 KB, 87.03%

### Notes

- Project verification now checks that `APP_FW_VERSION` is `0.13.0`.
- RX/TX queue depths are compile-time settings:
  - `APP_SERIAL_LINE_QUEUE_DEPTH`
  - `APP_SERIAL_TX_QUEUE_DEPTH`

## 2026-06-01 v0.12.0

### Done

- Added firmware identity macros:
  - `APP_FW_NAME`
  - `APP_FW_VERSION`
- Added serial command:
  - `VERSION`
- Boot banner now includes firmware name and version.
- Serial smoke-test scripts now query `VERSION` first.
- Project verification now checks that `APP_FW_VERSION` is `0.12.0`.

### Verification

- `tools/serial_smoke.ps1` parsed successfully with PowerShell.
- `tools/serial_smoke.py` passed `py_compile`.
- `powershell -NoProfile -ExecutionPolicy Bypass -File tools\verify_project.ps1` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 54088 bytes
  - application limit: 63488 bytes
- Current memory usage:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 54088 B / 62 KB, 85.19%

### Notes

- Use `VERSION` after flashing to confirm the board is running the expected firmware.

## 2026-06-01 v0.11.0

### Done

- Added project verification script:
  - `tools/verify_project.ps1`
- Added VS Code task:
  - `Project: verify`
- Verification checks:
  - CMake Debug build passes
  - `.elf`, `.hex`, `.bin`, and `.map` artifacts exist
  - `.bin` size fits inside the 62 KB application region
  - linker reserves the final 2 KB Flash
  - parameter page remains at `0x0800F800`
  - teach page remains at `0x0800FC00`
  - CMake still generates `.hex` and `.bin`
  - VS Code task/debug/settings files exist
  - `Version.md` and `Control.md` exist

### Verification

- `powershell -NoProfile -ExecutionPolicy Bypass -File tools\verify_project.ps1` passed.
- Current firmware binary size:
  - `SCARA_F103.bin`: 53908 bytes
  - application limit: 63488 bytes

### Notes

- This script is the preferred local regression check after future edits.

## 2026-06-01 v0.10.0

### Done

- Added serial smoke-test tools:
  - `tools/serial_smoke.ps1`
  - `tools/serial_smoke.py`
- Added VS Code serial test tasks:
  - `Serial: smoke safe`
  - `Serial: smoke with tiny motion`
- The default smoke test sends only safe read/query commands.
- The tiny motion smoke test is opt-in and sends a 100-pulse move after disabling the watchdog.

### Verification

- `tools/serial_smoke.ps1` parsed successfully with PowerShell.
- `tools/serial_smoke.py` passed `py_compile` using the bundled Python runtime.
- `cmake --build --preset Debug` passed.
- Firmware memory usage unchanged from v0.9.0:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 53908 B / 62 KB, 84.91%

### Notes

- The PowerShell script uses .NET serial APIs and needs no extra Python package.
- The Python script requires `pyserial` on user machines.

## 2026-06-01 v0.9.0

### Done

- Added post-build firmware artifacts:
  - `SCARA_F103.hex`
  - `SCARA_F103.bin`
- Added VS Code workspace configuration:
  - `.vscode/tasks.json`
  - `.vscode/launch.json`
  - `.vscode/settings.json`
- Added VS Code tasks:
  - `CMake: configure Debug`
  - `CMake: build Debug`
  - `OpenOCD: flash ST-Link`
  - `OpenOCD: flash CMSIS-DAP`
- Added Cortex-Debug launch configurations:
  - `Debug SCARA_F103 ST-Link`
  - `Debug SCARA_F103 CMSIS-DAP`

### Build Verification

- `cmake --build --preset Debug` passed.
- Generated artifacts:
  - `build/Debug/SCARA_F103.elf`
  - `build/Debug/SCARA_F103.hex`
  - `build/Debug/SCARA_F103.bin`
  - `build/Debug/SCARA_F103.map`
- Current memory usage with two 1 KB Flash storage pages reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 53908 B / 62 KB, 84.91%

## 2026-06-01 v0.8.0

### Done

- Added joint-space convenience commands:
  - `JSTATUS`
  - `JREL dtheta1_mrad dtheta2_mrad vmax accel`
  - `HOME`
  - `HOME vmax accel`
- Refactored `JOINT` execution through a shared joint-to-pulse helper.
- Updated `HELP 3` to include the new joint and home commands.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with two 1 KB Flash storage pages reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 53908 B / 62 KB, 84.91%

### Notes

- `HOME` is a software return to pulse position `0,0`; it is not sensor homing.
- Without limit switches or encoders, establish a reliable mechanical zero before using `ZERO` and `HOME`.

## 2026-06-01 v0.7.0

### Done

- Split `HELP` into paged responses:
  - `HELP`
  - `HELP 1`
  - `HELP 2`
  - `HELP 3`
  - `HELP 4`
- Reduced longest help response from 376 bytes to 150 bytes.
- Added trajectory validation:
  - `TRAJ_VALIDATE`
- Added teach point validation:
  - `TEACH_VALIDATE`
- `Trajectory_Run()` now rejects queues containing pulse targets outside current runtime motor soft limits.
- `TEACH_RUN` now validates converted trajectory before starting.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with two 1 KB Flash storage pages reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 53188 B / 62 KB, 83.78%

### Notes

- The paged `HELP` design leaves TX buffer headroom for future status fields.
- `TEACH_VALIDATE` does not modify the current trajectory queue.

## 2026-06-01 v0.6.0

### Done

- Added Flash-backed teach point persistence:
  - `TEACH_SAVE`
  - `TEACH_LOAD`
- Teach points are loaded automatically during `Teach_Init()` if the saved record is valid.
- Teach Flash record includes:
  - magic
  - version
  - count
  - CRC
  - up to `APP_TEACH_MAX_POINTS` pulse points
- Split the final 2 KB of Flash into two independent pages:
  - parameter page: `0x0800F800`
  - teach page: `0x0800FC00`
- Reduced application Flash region from 63 KB to 62 KB to reserve both pages.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with two 1 KB Flash storage pages reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 52432 B / 62 KB, 82.59%

### Notes

- `HELP` is now 376 bytes, under the 384-byte TX buffer. Add future commands carefully or split help into multiple responses.
- Full-chip erase clears both saved parameters and saved teach points.

## 2026-06-01 v0.5.0

### Done

- Added communication watchdog safety stop:
  - default timeout: 3000 ms
  - if no command is received while motion is active, trajectory execution is stopped and axes decelerate to stop
  - watchdog trip sets `STEPPER_ERR_COMM_TIMEOUT`
- Added watchdog commands:
  - `WATCHDOG`
  - `WATCHDOG ON timeout_ms`
  - `WATCHDOG OFF`
- Extended `STATUS` with watchdog fields:
  - `wd`
  - `wd_ms`
  - `idle_ms`
- Added readable error query:
  - `ERRORS`
- Exposed stepper error bit definitions in `stepper_driver.h`:
  - `STEPPER_ERR_SOFT_LIMIT`
  - `STEPPER_ERR_ESTOP`
  - `STEPPER_ERR_COMM_TIMEOUT`
- Added `Stepper_SetErrorAll()` for system-level fault reporting.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with the 1 KB parameter page reserved:
  - RAM: 5280 B / 20 KB, 25.78%
  - FLASH: 51608 B / 63 KB, 80.00%

### Notes

- `CLEAR_ERROR` / `RESET` clears the watchdog trip and stepper error bits.
- Continuous `SPEED` mode now requires periodic commands, such as `PING`, `STATUS`, or any valid command, unless `WATCHDOG OFF` is used.

## 2026-06-01 v0.4.0

### Done

- Added Flash-backed parameter persistence on the last 1 KB flash page:
  - parameter page address: `0x0800FC00`
  - page size: 1024 bytes
  - magic/version/size/CRC validation
- Reserved the last Flash page by changing linker Flash length from 64 KB to 63 KB.
- Startup now loads saved parameters automatically after applying defaults.
- Added parameter persistence commands:
  - `PARAM_SAVE`
  - `PARAM_LOAD`
  - `PARAM_DEFAULTS`
- Increased serial TX buffer to 384 bytes so the expanded `HELP` response is not truncated.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage with the 1 KB parameter page reserved:
  - RAM: 5264 B / 20 KB, 25.70%
  - FLASH: 50676 B / 63 KB, 78.55%

### Notes

- If an external flashing tool performs a full-chip erase, saved parameters will be erased.
- After changing calibration over serial, run `PARAM_SAVE` to persist it.
- `PARAM_DEFAULTS` restores compile-time defaults in RAM; run `PARAM_SAVE` after it if defaults should replace saved Flash parameters.

## 2026-06-01 v0.3.0

### Done

- Added RAM runtime parameter layer:
  - `app_params.h`
  - `app_params.c`
- Kinematics now uses runtime parameters instead of only compile-time macros.
- Motor pulse soft limits now use runtime parameters.
- Added parameter query/configuration commands:
  - `PARAMS`
  - `GET_PARAMS`
  - `SET_PULSE ppr1 ppr2 rr1 rr2`
  - `SET_GEOM base_um active1_um active2_um passive1_um passive2_um`
  - `SET_LIMIT theta1_min theta1_max theta2_min theta2_max`
  - `SET_MOTOR_LIMIT min_pulse max_pulse`
  - `SET_ZERO zero1_mrad zero2_mrad`
  - `SET_DIR dir1 dir2`
  - `SET_IK left_sign right_sign`
  - `SET_MOVL_SEG segment_um`
- Added point inspection commands:
  - `TRAJ_GET index`
  - `TEACH_GET index`
- Increased serial TX buffer to 320 bytes so long `HELP` and `PARAMS` responses fit.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage:
  - RAM: 5168 B / 20 KB, 25.23%
  - FLASH: 48804 B / 64 KB, 74.47%

### Current Limits

- Runtime parameters are RAM-only. Power cycling restores values from `app_config.h`.
- Flash-backed `PARAM_SAVE` / `PARAM_LOAD` is still pending.
- Flash usage is now about 74%, so future additions should stay compact.

## 2026-06-01 v0.2.0

### Done

- Added configurable five-bar SCARA geometry in `UserApp/app_config.h`:
  - motor pulse/rev
  - motor direction sign
  - motor zero offset
  - base distance
  - active/passive arm lengths
  - joint soft limits
  - inverse-kinematics branch signs
- Extended `scara_kinematics`:
  - joint angle to motor pulse
  - motor pulse to joint angle
  - five-bar inverse kinematics
  - five-bar forward kinematics
  - workspace and joint-limit checks
- Added XY and trajectory protocol commands:
  - `WHERE`
  - `GOTOXY x_um y_um vmax accel`
  - `MOVL x_um y_um vmax accel`
  - `TRAJ_CLEAR`
  - `TRAJ_BEGIN`
  - `TRAJ_POINT p1 p2 v1 v2`
  - `TRAJ_XY x_um y_um vmax`
  - `TRAJ_RUN`
  - `TRAJ_END`
  - `TRAJ_STOP`
  - `TRAJ_STATUS`
- Added RAM trajectory queue with 48 points.
- Added basic teach replay:
  - `TEACH_RUN`
- Added motor pulse soft-limit rejection for `MOVE_REL`, `MOVE_ABS`, `JOINT`, `GOTOXY`, trajectory execution, and teach replay.
- Improved realtime boundaries:
  - TIM2 ISR no longer transmits serial status directly.
  - `STREAM ON` only sets a pending flag in ISR; DMA TX is started from the main loop.

### Build Verification

- `cmake --build --preset Debug` passed.
- Current memory usage:
  - RAM: 4952 B / 20 KB, 24.18%
  - FLASH: 45464 B / 64 KB, 69.37%

### Current Limits

- SCARA dimensions are safe placeholder values. Measure the real mechanism and update `app_config.h`.
- `MOVL` uses fixed spatial segmentation, not a full continuous-velocity Cartesian planner yet.
- Trajectory and teach points are RAM-only.
- Without encoders or limit switches, `WHERE`, `MOVL`, and teach replay depend on software-estimated position.

## 2026-06-01 v0.1.0

### Done

- Added `SCARA_F103/UserApp` application layer:
  - `app_config`
  - `board_pins`
  - `stepper_driver`
  - `motion_planner`
  - `serial_dma`
  - `protocol`
  - `scara_kinematics`
  - `trajectory`
  - `teach`
  - `app_main`
- Integrated `App_Init()` and `App_Loop()` into `Core/Src/main.c`.
- Integrated all user application sources into CMake.
- Implemented TIM1_CH1 / TIM4_CH1 dual stepper PWM basics:
  - enable/disable
  - direction
  - pps speed
  - stop / emergency stop
  - 1 kHz software position estimate
- Implemented TIM2 1 kHz motion scheduler:
  - velocity ramp
  - MOVE-mode deceleration
  - PWM ARR/CCR update
- Implemented USART1 DMA circular line protocol:
  - DMA circular RX
  - LF/CRLF line framing
  - DMA non-blocking TX
- Supported initial commands:
  - `PING`
  - `HELP`
  - `STATUS`
  - `STREAM ON`
  - `STREAM OFF`
  - `ENABLE 1/0`
  - `SPEED pps1 pps2`
  - `STOP`
  - `ESTOP`
  - `CLEAR_ERROR` / `RESET`
  - `ZERO`
  - `ACCEL a1 a2`
  - `MOVE_REL dp1 dp2 v1 v2`
  - `MOVE_ABS p1 p2 v1 v2`
  - `JOINT theta1_mrad theta2_mrad vmax accel`
  - `TEACH_CLEAR`
  - `TEACH_ADD`
  - `TEACH_LIST`

### Build Verification

- `cmake --preset Debug` passed.
- `cmake --build --preset Debug` passed.
- Memory usage:
  - RAM: 3784 B / 20 KB, 18.48%
  - FLASH: 30420 B / 64 KB, 46.42%

### Hardware Assumptions

- MCU: STM32F103C8T6
- Clock: 72 MHz
- PWM timer counter clock: 1 MHz
- M1:
  - PUL: PA8 / TIM1_CH1 / AF Open-Drain
  - DIR: PB12 / Open-Drain
  - ENA: PB13 / Open-Drain
- M2:
  - PUL: PB6 / TIM4_CH1 / AF Open-Drain
  - DIR: PB7 / Open-Drain
  - ENA: PB8 / Open-Drain
- Recommended DM556 common-anode wiring:
  - PUL+/DIR+/ENA+ to +5V
  - STM32 pins to PUL-/DIR-/ENA-
- Placeholder pulse settings:
  - M1: 1600 pulse/rev
  - M2: 1600 pulse/rev
