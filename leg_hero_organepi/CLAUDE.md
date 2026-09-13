# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Working Rules

- Do not modify source code or comments without the user's explicit permission. When the user is asking a question or describing a problem, diagnose and report findings only; do not apply a fix unless requested.
- Before every response, state whether code or comments were modified. Use `代码改动：...` and `注释改动：...`.
- 永远不要动我的注释，除非我要求你改注释
## Project overview

Embedded control firmware for a **RoboMaster wheel-leg robot** (Leg Hero, "organepi"). It runs on an **Orange Pi (Linux, ARM64)** as the high-level "brain": it reads IMU / RC / motor feedback from the DAMIAO **MC02** motor controller over USB (libusb), runs attitude estimation, state machines, and leg/wheel torque control (VMC / LQR / MPC / LESO), then sends 4 joint + 2 wheel torque setpoints back over the same USB link.

The codebase is ~11k lines of C++20/CMake. Comments are in Chinese and frequently render as mojibake (UTF-8/GBK mismatch) 鈥? treat garbled comments as untrustworthy, and much commented-out code is stale.

## Build

No test framework, no linter, no scripts. Build is CMake only.

**Target (Orange Pi / Linux):** the source is POSIX/Linux-only (`termios.h`, `netinet/in.h`, `/dev/input/event*`, `linux/input.h`, `libusb`). Configure and build on the robot:

```sh
cmake -B build && cmake --build build
```

Binary: `build/wheel_leg_sys`.

**Windows (this machine):** `build/` is configured with a MinGW toolchain but cannot link (Linux-only headers). It is only useful for IDE/syntax checks; `.vscode` IntelliSense targets `linux-gcc-arm64`. A syntax-only check works locally (matches the allowed command in `.claude/settings.local.json`):

```sh
g++ -std=c++20 -fsyntax-only -Isrc -Isrc/Algorithm/EKF -Isrc/Algorithm/KalmanFilter -Isrc/Algorithm/MPC -Isrc/Algorithm/PID -Isrc/Task -Isrc/Bsp -Isrc/Devices -Isrc/Algorithm/CRC src/Interaction/robot.cpp
```

Dependencies (must exist on the build machine): **Eigen3, qpOASES, cppad, libusb-1.0, CasADi**. `CMakeLists.txt` falls back to a hardcoded CasADi `.so` path if `find_package(casadi)` fails.

## Architecture

Layered design in the RoboMaster convention (Bsp 鈫? Devices 鈫? Algorithm 鈫? Interaction 鈫? Task):

```
src/main.cpp        鈥? calls Task_Init() then sleeps forever
src/Task/           鈥? timing + wiring: the global Robot, USB callback, periodic timer threads
src/Interaction/    鈥? robot-level behavior (Robot, Chassis)
src/Algorithm/      鈥? control & estimation math (MPC, NMPC, VMC/LQR, LESO, PID, FSM, AHRS, ...)
src/Devices/        鈥? peripheral drivers (MC02, INS, GamePad, Vofa_TCP)
src/Bsp/            鈥? board support (libusb USBManager, termios SerialPort, high-res Timer)
```

### Execution model (`src/Task/task_and_callback.cpp`)

`Task_Init()` builds a **single global `Robot robot`**, registers the USB receive callback on `USBManager`, then starts `class_Timer` threads:

- **1 ms** `TIM_1ms_PeriodElapsedCallback` 鈥? INS attitude update + `Chassis.Self_Observe()` (wheel-odometry velocity estimate via Kalman filter + slip detection).
- **"2 ms"** `TIM_2ms_PeriodElapsedCallback` 鈥? the main control loop: lock `rx_mutex` 鈫? FSM steps (chassis / jump / up-stairs) 鈫? `Chassis.feedback_update()` 鈫? `Robot::Chassis_Control()` 鈫? ground detection 鈫? Vofa telemetry 鈫? unlock 鈫? `mc02_send_data()`.
- **10 ms** `TIM_10ms_PeriodElapsedCallback` 鈥? reserved; NMPC solve is currently commented out.

Gotcha: despite the `timer_2ms` name, both the 1 ms and "2 ms" timers are started with `0.001` interval, so the control loop actually runs at **1 kHz**. Yet `dt = 0.002` constants in `Chassis.h`/`class_vmc_leg` assume 2 ms. Also, `mc02_send_data()` deliberately runs **outside** the `rx_mutex` because `libusb_bulk_transfer` can block up to 1 s.

### Data flow

1. USB RX 鈫? `mc02_callback_usb()` 鈫? CRC16 verify 鈫? copy into `robot.mc02_SerialPort.Rx_Data` (IMU, RC channels + keyboard bitfield, motor positions/velocities). RC channels and the `key` field pass through a 19-tap median filter.
2. Control loop turns `RC` inputs into target setpoints and feeds `Chassis`.
3. `mc02_send_data()` packs `joint_torque_set[4]` + `wheel_torque_set[2]` + `spin_flag` with header `0xA5` + CRC16 and sends over USB.

**Watch the sign flips** in `mc02_send_data()`: right-side joint torques and the right wheel torque are negated relative to the internal convention.

### Remote control (`src/Interaction/robot.h`)

The input path is selected at compile time by macros `FUS_I6X` / `GAMEPAD` / `VT03` (only one set to 1; currently `FUS_I6X 1`). `FUS_I6X`: MC02 link supplies RC channels (`axis[]`) and a keyboard bitfield (`key`, see `KEY_PRESSED_OFFSET_*`). `GAMEPAD`: Xbox controller on `/dev/input/event6` via `class_GamePad`.

`Robot::Remote_update()` maps raw RC/keys to `Remote_Control` flags (`zero_force`, `competition`, `check_in_pos`, `spin`, `jump`, ...), which select `chassis_mode` in `Robot::Chassis_Control()`.

### State machines (`src/Algorithm/FSM/fsm.h` + `src/Interaction/robot.cpp`)

`Class_FSM` is a reusable base (status count + per-state `Count_Time`). `Robot` owns three FSM subclasses with transition logic in `robot.cpp`:

- `Class_FSM_Chassis` 鈥? NORMAL / OVER_TURN / OVER_TURN_UPSIDE1/2 / OVER_TURNING1/2 / ABOVE_GROUND. Self-righting + flip detection from `INS.pitch` and leg angles.
- `Class_FSM_Jump` 鈥? BEND_LEG 鈫? STRETCH_LEG 鈫? BEND_LEG_AIR 鈫? STRETCH_LEG_AIR 鈫? STOP (jumping via `Jump_force` + height targets).
- `Class_FSM_Up_staris` 鈥? APPROACH / GET_POSITION / UP_STARIS_BEND_LEG / OVER (stair climbing).

### Chassis control stack (`src/Interaction/chassis/Chassis.cpp`)

`class_Chassis` is where the physics happens, called every control cycle:

- `feedback_update()` 鈥? maps motor angles to leg virtual state via `class_vmc_leg`.
- `Self_Observe()` 鈥? velocity estimate (Kalman filter on wheel odometry + IMU).
- `Follow_Gimbal_Control()` / `Control_leg()` / `Check_in_Control()` 鈥? mode-specific control:
  - **VMC** (`class_vmc_leg`, `src/Algorithm/VMC/`): converts desired leg force `F0` (along the leg) and joint torque `Tp` into the 2 motor torques, using MATLAB Coder鈥揼enerated leg geometry/speed/convert functions.
  - **LQR** gains are interpolated by leg length from the large `P[40][6]` table (`lqr_k_calc`).
  - **Linear MPC** (`MPC_Controller`, qpOASES) regulates body roll + leg-length/height tracking; a quintic `TrajectoryPlanner` (in `src/Algorithm/Slope/slope.h`) shapes the height reference.
  - **LESO** (linear extended state observer, `src/Algorithm/LESO/`, generated C) estimates disturbances.
- `Ground_Detection()` / `Slip_Detection_And_Suppression()` guard wheel traction.

### MPC / NMPC

Two solvers exist:

- `MPC_Controller` (linear MPC via qpOASES) 鈥? **active** (`MPC_body` in `Chassis`, horizon `N=15`).
- `class_NMPC` (SQP with CppAD linearization, `src/Algorithm/MPC/NMPC.cpp`) 鈥? recently implemented, **not tested**, not wired into the control loop (call site commented out in `TIM_10ms`). `class_NMPC_casADi` is a CasADi-solver variant. `src/Algorithm/system_dynamic/` holds generated dynamics (`x_ddot`, `theta_ddot`, `phi_ddot`) for the NMPC model.

## Conventions / gotchas

- Many blocks are commented out and stale; don't treat commented code as documentation of current behavior.
- A lot of the math (VMC leg kinematics, LESO, LQR gains, system dynamics) is **generated by MATLAB Coder / CasADi** 鈥? regenerate with the same codegen settings, don't hand-edit.
- Control gains and geometry constants (mass `m=25`, wheel radius `r=0.16`, spring constants, jump forces, slope limits) live inline in `Chassis.h`/`Chassis.cpp` and `robot.h`.
- `build/` and `cmake-build-debug/` are committed to git; don't rely on them as the authoritative build.
