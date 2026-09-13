# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working in this repository.

## Build And Validation

This is an STM32H723VGT6 firmware project. The root `CMakeLists.txt` configures an `arm-none-eabi` cross-toolchain directly and requires CMake 3.26 or newer.

- Build the existing CLion/CMake Debug tree:
  ```sh
  cmake --build cmake-build-debug --target CtrlBoard-H7_IMU.elf
  ```
- The build target is `CtrlBoard-H7_IMU.elf`. Post-build steps emit `CtrlBoard-H7_IMU.hex`, `CtrlBoard-H7_IMU.bin`, and a map file in the build directory.
- The checked-in `cmake-build-debug/CMakeCache.txt` uses a machine-local ARM GCC 10.3 toolchain under `C:/CLionToolchains/gcc-arm-none-eabi-10.3-2021.10/`. A clean configure requires `arm-none-eabi-gcc`, `arm-none-eabi-g++`, and `arm-none-eabi-objcopy` to be available; no portable configure script or preset is provided.
- There is no project lint/format command or configuration (`.clang-format`, `.clang-tidy`, `.editorconfig`, or equivalent).
- There is no test framework, CTest target, test directory, or supported single-test command. Firmware compilation is the available automated validation.
- `daplink.cfg` is an OpenOCD CMSIS-DAP/SWD configuration at 10 MHz, but the repository does not provide a portable flash command. CLion's local OpenOCD settings contain machine-specific paths and should not be treated as project-wide instructions.

## Architecture

The target is a single-core Cortex-M7 STM32H723 firmware. Reset enters the ST startup assembly and `Core/Src/main.c`. Startup configures HAL, the clock tree and generated peripherals, then calls `Task_Init()`, blocks until the BMI088 IMU initializes over SPI2, enables board power outputs, and starts CMSIS-RTOS2/FreeRTOS.

`CtrlBoard-H7_IMU.ioc` is the STM32CubeMX source of truth for pins, clocks, DMA, NVIC, peripheral instances, FDCAN setup, and generated RTOS declarations. Generated startup and HAL code is under `Core`, `Drivers`, `Middlewares`, and `USB_DEVICE`. Handwritten firmware is under `User`:

- `User/Bsp` owns board-support wrappers for CAN, UART DMA/idle reception, timers, DWT timing, PWM, and callback registration.
- `User/Devices` owns stateful hardware/protocol abstractions: BMI088, DM and DJI motors, referee data, remote controls, buzzer/LED, VOFA, and inter-board CAN.
- `User/Algorithm` contains reusable control and estimation code, including Quaternion EKF, PID, filters, FSMs, and the LESO code under `ESO`.
- `User/APP` contains the periodic tasks and behavior/state selection for INS, RC arbitration, gimbal, shooting, vision, inter-board CAN, and telemetry.

### Runtime Flow

`Task_Init()` performs the pre-scheduler bootstrap: DWT setup; FDCAN initialization and callback registration; UART1 referee reception, UART5 Fusi remote double-buffer reception, and USART10 VT13 reception; timer callbacks/interrupts; buzzer setup; and finally enables `bsp_init_finished_flag`. Interrupt callbacks must remain gated until this initialization is complete.

The RTOS creates eight threads in `Core/Src/freertos.c`. `INS_TASK` has realtime priority; gimbal, shooting, vision, RC, inter-board CAN, and telemetry tasks are normal priority. Most application loops run at approximately 1 kHz using `osDelay`/`vTaskDelay`; several have startup delays. Application state is primarily exchanged through global structures and pointers rather than queues or mutexes.

The main data path is:

`UART/CAN/USB callbacks` -> decoded global device state -> RC/INS/vision normalization -> gimbal/shoot behavior selection -> PID/LQR/observer control -> motor CAN output.

- INS reads BMI088, filters/fuses gyro and accelerometer data with Quaternion EKF, and publishes attitude and motion data used by gimbal and vision.
- RC arbitration currently prefers VT13, then DR16, then Fusi when links are online. DR16 UART reception is present in code but its active UART5 initialization is commented out in `Task_Init()`.
- Gimbal controls yaw DM J4310 on FDCAN3 and pitch DM J4340 on FDCAN2, using INS attitude plus motor feedback. Mode transitions reset/retarget control state and motor health checks can disable output.
- Shooting controls the trigger DM J4310 on FDCAN3 and left/right DJI friction motors on FDCAN1. Referee heat data and motor status affect firing and friction-wheel output.
- Vision exchanges packets through USB CDC. The vision task transmits attitude/status data and consumes decoded target/control data maintained by the USB receive path.
- `Can_Comm_Task` publishes motor, control, remote, VT13, and referee summaries over FDCAN3 to the companion board.

### Communications And Interrupts

- FDCAN1 carries the DJI friction motors (`0x201`, `0x202`) and is classic CAN at 1 Mbit/s.
- FDCAN2 carries pitch feedback/commands (`0x66`/`0x06`); FDCAN3 carries yaw (`0x55`/`0x05`), trigger (`0x77`/`0x07`), and inter-board/referee traffic. Although CubeMX configures FDCAN2/3 for FD+BRS, `Task_Init()` reconfigures them at runtime to classic 1 Mbit/s.
- HAL IRQ handlers are dispatched in `Core/Src/stm32h7xx_it.c`. Application CAN, UART, and timer callback routing is implemented in `User/Bsp` and `User/APP/task_config_and_callback.c`.
- UART and CAN callbacks execute in interrupt-driven paths. Keep ISR work short and nonblocking; parsers update shared device state for tasks to consume.

## Generated Code And Change Boundaries

Preserve application edits inside `/* USER CODE BEGIN */` / `/* USER CODE END */` regions in CubeMX-generated files. Regeneration can overwrite code outside those regions. This applies especially to `Core/Src/main.c`, `Core/Src/freertos.c`, peripheral sources and headers, interrupt/MSP files, and USB device files. Change the `.ioc` when the hardware/peripheral configuration itself must change, then inspect the generated diff carefully.

CMake recursively compiles `Core`, `Middlewares`, `USB_DEVICE`, `Drivers`, and `User`. New source files under an existing `User` subtree are picked up automatically; a new include directory may require an explicit addition to `CMakeLists.txt`. Avoid adding duplicate source copies.

Before editing, inspect the current git diff. This working tree contains pre-existing modifications across CubeMX-generated files and application code, untracked LESO pitch files under `User/Algorithm/ESO`, and deleted local DLLs. Preserve unrelated user changes and do not regenerate or reset the tree without reviewing their impact.

## Configuration Notes

CMake targets Cortex-M7 with hard floating point but currently defines `ARM_MATH_CM4`, uses `fpv4-sp-d16`, and includes the FreeRTOS `ARM_CM4F` port path. CubeIDE metadata uses a different FPV5-D16 setting. Treat this as existing project configuration drift; do not silently “correct” it while making an unrelated change.

The CMake linker script is `STM32H723VGTX_FLASH.ld` (1 MB flash, with ITCM/DTCM/D1/D2/D3 RAM regions); `STM32H723VGTX_RAM.ld` also exists but is not the selected script. CMSIS-DSP is linked from `Middlewares/ST/ARM/DSP/Lib` using `libarm_cortexM7lfsp_math.a`.
