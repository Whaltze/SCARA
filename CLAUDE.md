# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A five-bar **parallel SCARA** robot (two stepper-driven active arms, open-loop, DM556 drivers + STM32F103C8T6) with optional laser marking. The repo has two cooperating halves:

- **`SCARA_F103/`** — STM32F103 firmware (C, CubeMX + CMake). The realtime motion kernel.
- **`SCARA_UI/`** — PySide6 host application (Python). Planning, control GUI, vision, velocity monitor.

`grbl-1.1h.20190825/` (reference port source, gitignored) and `SOURCE/` (datasheets/images, gitignored) are not part of the build. Code comments and the `.md` docs are written in **Chinese** — match that when editing.

## The one motion path that matters

The **only** official motion entry is a **GRBL-style character-counting G-code stream**. The host plans geometry, emits real Cartesian G-code, and the MCU does look-ahead + per-segment SCARA inverse kinematics + step generation. Older paths (`BinaryTraj`, host-timed point trajectories, UI binary-send, experimental text links) are **retired — do not reintroduce them.** Files for those were deleted; if you find a stray helper, it is dead code.

Responsibility split (reported by the `HOSTCAP` command, `APP_HOST_OWNS_LIMIT_CHECKS=1`):

- **Host owns**: workspace/IK-existence/joint-range/path limit checks, and all trajectory speed planning. It must walk every point before sending.
- **MCU owns**: look-ahead planning, five-bar IK at segment-prep time, timed-DDA step output, safety stop, homing, status reporting, G-code syntax + geometric IK-existence checks.

**Exact-stop policy**: every segment comes to a full stop at each point for accuracy. Do **not** propose corner blending/rounding (see memory `motion-exact-stop-not-blending`).

## Build / flash / test

### Firmware (run from `SCARA_F103/`)
```bash
cmake --preset Debug          # configure (Ninja + arm-none-eabi toolchain) — first time / after CMake edits
cmake --build --preset Debug  # build -> build/Debug/SCARA_F103.{elf,hex,bin}
```
Offline static gate (run before claiming firmware is good):
```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\verify_project.ps1
```
Flash via VS Code tasks (OpenOCD, target `stm32f1x`): **"OpenOCD: flash CMSIS-DAP"** or **"OpenOCD: flash ST-Link"** (both depend on the build task and program the `.elf`). User code lives in `UserApp/`; `Core/` and `Drivers/` are CubeMX-generated. `UserApp/*.c` is compiled `-Os`.

### Host UI (run from repo root)
Conda env is **`robot`** (PySide6, pyserial, numpy, OpenCV). The UI framework is **PySide6** — not PyQt5.
```bash
conda activate robot
python SCARA_UI/main.py        # launches the main window + the V_monitor velocity scope
```
Tests are plain runnable scripts under `SCARA_UI/tests/` (no pytest). Run one directly:
```bash
python SCARA_UI/tests/trajectory_planner_check.py    # a single "test"
python SCARA_UI/tests/sender_strategy_check.py
python SCARA_UI/tests/sender_benchmark_check.py
```

### Hardware-in-the-loop stress (PowerShell, need a COM port)
`SCARA_F103/tools/` holds serial stress/validation scripts, e.g.:
```powershell
powershell -File tools\host_planned_stream_stress.ps1 -Port COM13 -Count 3000 -EnableMotion
powershell -File tools\ui_trajectory_stress.ps1 -Port AUTO -Count 3000
powershell -File tools\ui_control_matrix_check.ps1 -Port AUTO
```

## Firmware architecture (`SCARA_F103/UserApp/`)

Three execution contexts (wired in `app_main.c`):

- **`App_Loop()`** — main loop, non-realtime: serial RX poll, protocol/G-code parse + enqueue, homing progress. Applies GRBL-style backpressure (stops draining the RX ring when planner or TX is full).
- **`App_Tick1kHz()`** — **SysTick @ 1 kHz** (`stm32f1xx_it.c`): planner refill, gcode-stream/protocol housekeeping, homing tick, laser safety gating.
- **`App_StepEventIrq()`** → `Stepper_StepEventIrq()` — **TIM2 @ 10 kHz** (`APP_CONTROL_HZ`, `BOARD_TICK_TIM`): the realtime step-event consumer.

Motion pipeline, top to bottom:

| Module | Role |
|---|---|
| `gcode_stream.c` | G-code parser, RX ring, `ok`/`error:<n>` ACK, realtime chars `? ! ~ ^X`, `$H`/`$X`/`$G` |
| `motion_planner.c` | 48 Cartesian planner blocks; junction-deviation reverse/forward look-ahead; G0/G1 + G2/G3 arcs; 16 timed-DDA segments; calls SCARA IK per segment, rate-limits segments to joint PPS |
| `scara_kinematics.c` | Five-bar parallel IK/FK + workspace checks |
| `stepper_driver.c` | TIM1/CH1 (M1) + TIM4/CH1 (M2) one-pulse STEP, DIR/ENA GPIO, software pulse position |
| `home_controller.c` / `home_sensor.c` | `$H` homing state machine, HOME1/HOME2 switch inputs |
| `laser_control.c` | TIM3/CH2 1 kHz laser PWM + relay (PA2); M3/M4/M5, S0..1000, L0/L1/L2 (lift/mark/relay-prep); forced off on idle/stop/fault |
| `protocol.c` | Non-G-code commands (VERSION/STATUS/ENABLE/HOME_SENSOR/WATCHDOG/...) and the `<...>` status-frame builder |
| `serial_dma.c` | USART1 DMA RX (circular) + TX queue |
| `app_params.c` | Flash-persisted params (last page `0x0800F800`); bumping `APP_PARAM_FLASH_VERSION` restores source defaults on next boot |

**`app_config.h` is the central tuning file** (geometry in µm, `6400` PPR, speeds/accels, planner sizing, homing angles in mrad, laser, flash). It is the single source of truth — its geometry/PPR/zero-offset values **must stay in sync** with the UI's `FiveBarConfig` and `current_ppr`, or the two kinematic models disagree.

Pin/timer map: M1 STEP `PA8`/TIM1_CH1, M1 DIR `PB12`, M1 ENA `PB13`; M2 STEP `PB6`/TIM4_CH1, M2 DIR `PB7`, M2 ENA `PB8`; control ISR TIM2; laser `PA7`/TIM3_CH2 + relay `PA2`; USART1 `PA9`/`PA10` @115200, RX DMA1_Ch5 circular, TX DMA1_Ch4; SWD `PA13`/`PA14`.

## Host UI architecture (`SCARA_UI/`)

`FiveBarSerialGUI` (in `ui/main_window.py`) is composed by **mixin inheritance** — behavior is spread across mixins, not one class:
`ScaraUiMixin` (`ui/ui_mixin.py`) · `ScaraUtilityMixin` (`core/utility_mixin.py`) · `ScaraVisionMixin` (`vision/`) · `ScaraSerialMixin` (`communication/serial_mixin.py`) · `ScaraMotionMixin` (`motion/motion_mixin.py`) · `ScaraPlotMixin` (`ui/plotting.py`). When adding a feature, extend the matching mixin.

Key non-GUI pieces:

- `core/kinematics.py` — `FiveBarKinematics` IK/FK + reachability. Mirrors the firmware geometry.
- `trajectory/look_ahead.py` — `LookAheadPlanner`: real-geometry segments → junction-deviation reverse/forward passes → arc-length-sampled `G1 ... F` point stream. Arcs are sampled as arcs (feed does not dip inside an arc).
- `communication/motion_senders.py` — `GrblGcodeSender` + `GcodeJob`: the **only** sender; lazy command source, ≤64 buffered, character-counting send window. Serial I/O runs in a `QThread` (`communication/serial_worker.py`); never block the UI thread waiting for an ACK.
- `vision/` — camera, hand-eye + undistortion calibration, coordinate transforms (OpenCV).
- `V_monitor.py` — standalone velocity/acceleration monitor window, launched alongside the main window by `ui/app_bootstrap.py`.

## Protocol essentials

Line-based G-code over USART1 @ 115200 8N1. On the host send loop:

- **`ok`** (lowercase) — line accepted/enqueued; this is the ACK that advances the send queue.
- **`error:<code>`** — line rejected; stop sending the trajectory.
- **`<...>`** — async status frame (~5 Hz), **not** an ACK. Realtime chars (`?`, `!`, `~`, Ctrl-X, jog-cancel) bypass the line queue.
- System OKs (`OK ENABLE 1`, `OK CLEAR_ERROR`, `OK ZERO`) are not the G-code `ok` and must not advance the queue.

Full command list, status-frame fields, `error:`/`E:` code tables, homing flow, and laser safety commissioning are documented in **`Control.md`**.

**Coordinate convention (critical):** UI X=0 at the left motor, X=150 at the right, UI home `X=75, Y=220`; MCU X=0 is the midpoint between motors. The UI auto-converts `X_mcu = X_ui − 75` on send and `X_ui = X_mcu + 75` on receive. **Manual serial G-code must use MCU coordinates** (UI center ≈ `X0 Y220`, not `X75`).

**Open-loop reality:** no encoders. `ZERO` and `$H`/`HOME` only reinterpret software coordinates — they do not move the physical arm to a new pose. Don't claim true drag-teaching.

## Laser safety (do not bypass)

`PA7` PWM drives a laser; safety is gated in firmware (boot lockout, idle disarm, off on stop/fault). **Never change `APP_LASER_PWM_ACTIVE_HIGH` polarity while the laser is powered**, and software is not a substitute for the relay/MOSFET kill path. The full commissioning checklist is in `Control.md`.

## Project docs (authoritative, keep in sync)

- **`PROMPT.md`** — the project plan. §0 ("2026-06-12 续接项目计划：GRBL-SCARA 四层流式运动内核") is the current priority and overrides older sections on conflict.
- **`Control.md`** — operations/debug guide: protocol, error codes, status frame, homing, laser commissioning, test commands.
- **`Version.md`** — per-version changelog. **Append a new entry on every version iteration** (firmware version lives in `APP_FW_VERSION`).
- **`Work.md`** — working notes.
