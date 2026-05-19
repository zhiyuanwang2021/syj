# AGENTS.md

## Project Identity

- Project type: STM32H7 embedded firmware project
- MCU: STM32H750
- Core: ARM Cortex-M7
- Language: C
- IDE / Toolchain: Keil MDK-ARM
- Compiler: ARMCC
- RTOS: FreeRTOS
- RTOS abstraction: CMSIS-RTOS2
- HAL: STM32 HAL Library

## Technology Stack

- STM32H750 + Cortex-M7
- Keil MDK-ARM / uVision
- ARMCC compiler
- FreeRTOS
- CMSIS / CMSIS-RTOS2
- STM32H7 HAL Driver
- W5500 Ethernet
- Modbus
- MQTT
- SNMP
- Easylogger

## Repository Structure

- `Core/`
  - STM32Cube-generated core source and startup-related application files
  - Includes `Src/` and `Inc/`
- `Drivers/`
  - STM32 HAL drivers
  - CMSIS core/device files
  - Third-party CMSIS DSP / CMSIS NN content
- `Middlewares/`
  - Middleware components such as FreeRTOS
- `MDK-ARM/`
  - Keil project files
  - Build output directory
  - RTE auto-generated files
- `Hardware/`
  - Hardware drivers and peripheral abstraction
  - Place board-level device drivers here
- `Host/`
  - Application logic
  - Control flow
  - Device coordination
  - Business logic
- `Internet/`
  - Network protocol implementations and protocol-level modules
  - Includes MQTT, SNMP, HTTP, FTP, DHCP, DNS, TFTP, SNTP, etc.
- `ModbusLib/`
  - Modbus protocol stack and related headers
- `PID/`
  - PID control related source files
- `Easylogger/`
  - Easylogger component and porting files
- `Ethenet/`
  - Ethernet / W5500 related code
  - Note: actual repository directory name is `Ethenet`, not `Ethernet`
- `ADRC/`, `CS5530/`, `Regression/`
  - Functional modules retained as project-specific components

## Build System

- Primary build environment: Keil MDK-ARM
- Primary project file: `MDK-ARM/shiyanji_H7.uvprojx`
- Toolchain: ARMCC (Keil)
- Output directory: `MDK-ARM/shiyanji_H7/`
- Common outputs:
  - `*.axf`
  - `*.hex`
  - `*.bin`
  - `*.map`
  - `*.o`
  - `*.d`
  - `*.crf`

## Architecture Rules

- Keep hardware drivers in `Hardware/`
- Keep business logic, orchestration, control logic, and application behavior in `Host/`
- Keep network protocols and network services in `Internet/`
- Keep Modbus stack changes inside `ModbusLib/` unless integration glue is required elsewhere
- Keep RTOS kernel and vendor middleware under `Middlewares/` unless a porting layer requires local integration
- Keep HAL / CMSIS vendor code under `Drivers/` and avoid unnecessary edits unless fixing vendor integration issues
- Keep Keil project metadata and build outputs inside `MDK-ARM/`

## File Ownership Guidance

- Prefer editing user application files before modifying vendor-generated or vendor-supplied files
- Treat these as application-owned areas:
  - `Hardware/`
  - `Host/`
  - `Internet/`
  - `PID/`
  - `ModbusLib/`
  - `Easylogger/port/`
  - `Core/Src/`
  - `Core/Inc/`
- Treat these as vendor / generated areas and change only when necessary:
  - `Drivers/STM32H7xx_HAL_Driver/`
  - `Drivers/CMSIS/`
  - `Middlewares/Third_Party/FreeRTOS/`
  - `MDK-ARM/RTE/`

## Coding Conventions

- Use C for all embedded application and driver development
- Do not replace embedded C modules with C++ unless explicitly requested
- Preserve existing `.c`, `.h`, `.cpp`, `.s` source files
- Use clear module boundaries:
  - driver-facing code in `Hardware/`
  - system / business behavior in `Host/`
  - network-facing code in `Internet/`
- Prefer small, cohesive modules
- Keep ISR-related code lightweight and deterministic
- Avoid dynamic allocation unless the existing module already relies on it and the RTOS design permits it
- Keep hardware register access encapsulated in driver modules
- Prefer explicit initialization functions per module

## Naming Guidance

- Follow existing project naming first
- Keep filenames consistent with current module naming style
- Use `.c` / `.h` pairs for modules
- Use meaningful prefixes for hardware modules where helpful, such as device or peripheral names
- Avoid renaming existing public headers or source files unless explicitly required
- Preserve existing exported API names to avoid breaking MDK project integration

## RTOS Rules

- Use FreeRTOS through CMSIS-RTOS2 interfaces where the project already follows that model
- Do not block inside interrupts
- Keep task, queue, semaphore, and timer logic centralized and readable
- Review stack use and timing implications when adding new tasks
- Avoid long critical sections

## Networking Rules

- W5500-related Ethernet code belongs in `Ethenet/`
- Protocol-specific logic belongs in `Internet/`
- Modbus-related communication belongs in `ModbusLib/` or integration glue in `Host/`
- Keep transport abstraction separate from business logic where possible

## Logging Rules

- Use Easylogger for runtime logs where logging is already part of the module design
- Keep log statements concise and meaningful
- Avoid excessive logging in hard real-time or interrupt-sensitive paths

## Generated and Vendor Files

- Do not manually edit generated RTE content unless absolutely necessary
- Do not rely on generated `_ARMCMx` example content as application code
- Avoid modifying CMSIS NN tests, IAR examples, or unrelated vendor demos
- If CubeMX-regenerated files are edited, keep changes minimal and easy to reapply

## Ignore Rules

AI assistants should treat the following as generated, temporary, backup, or non-source artifacts and avoid relying on them as primary editable sources unless explicitly requested.

- MDK build outputs:
  - `*.o`
  - `*.d`
  - `*.crf`
  - `*.lst`
  - `*.axf`
  - `*.elf`
  - `*.hex`
  - `*.bin`
  - `*.map`
  - `*.dep`
  - `*.lnp`
  - `*.lib`
  - `*.iex`
- MDK user / temp files:
  - `*.uvoptx`
  - `*.uvguix*`
  - `*.uvgui.*`
  - `*.log`
  - `*.log.lock`
- Auto-generated directories:
  - `RTE/`
  - `_ARMCM*/`
- Backup / temp files:
  - `*.bak`
  - `*.tmp`
  - `*.TMP`
  - `*~`
- NN / example content to ignore:
  - `Drivers/CMSIS/NN/NN_Lib_Tests/`
  - `Drivers/CMSIS/NN/Examples/IAR/`
- Documentation / help artifacts:
  - `*.pdf`
  - `*.chm`

## Source File Protection

- Do not ignore or remove source files with these extensions:
  - `*.c`
  - `*.h`
  - `*.cpp`
  - `*.s`
- Treat source files as authoritative project assets

## AI Assistant Working Rules

- Read the existing module before editing
- Prefer minimal, targeted changes
- Preserve project structure and current naming
- Avoid broad refactors unless explicitly requested
- Avoid changing Keil project configuration unless the task requires it
- Avoid editing generated, vendor, test, or example directories unless the request is specifically about them
- When adding new functionality:
  - hardware-facing additions go to `Hardware/`
  - business logic goes to `Host/`
  - network protocol logic goes to `Internet/`
- When uncertain, prefer integration through application-owned files instead of modifying vendor code

## Build Validation Expectations

- Ensure new code remains compatible with MDK-ARM and ARMCC
- Avoid introducing dependencies on GCC-only extensions, CMake, or desktop-only tooling
- Keep include paths and file references compatible with the Keil project layout
- Assume final validation is performed by building the `MDK-ARM/shiyanji_H7.uvprojx` project in Keil uVision
