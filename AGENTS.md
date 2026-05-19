# AGENTS.md

## 1. 项目简介

本项目是基于 `STM32H750` 的嵌入式固件工程，主要用于执行机构/试验设备控制。结合源码可推断，系统包含以下能力：

- 位移、载荷、引伸计、大变形等测量数据采集与标定
- 伺服输出控制，支持位置/载荷/引伸计等控制模式
- 基于 `W5500` 的以太网通信，主通信链路为自定义上位机协议
- `RS485` / `Modbus` 通信
- 参数持久化（`MRAM`、外接传感器 `EEPROM`）
- `FreeRTOS` 多任务调度与日志输出

说明：

- 工程主体语言为 `C`，当前未发现项目级 `README`，以上简介基于目录结构、Keil 工程文件和主要源码推断。
- RTOS 实际使用方式为 `FreeRTOS + CMSIS-RTOS v1` 风格接口（`cmsis_os.h`、`osThreadDef` 等），不是 CMSIS-RTOS2。

## 2. 目录结构

- `Core/`
  - `Src/`：启动入口、外设初始化、中断、`freertos.c`
  - `Inc/`：对应头文件
- `Host/`
  - 业务核心层，优先阅读
  - 关键文件：
  - `control.c`：闭环控制与运动命令执行
  - `communicate.c`：上位机协议收发与命令分发
  - `in_out.c`：采样、量纲转换、输出映射
  - `gParameter.c/.h`：全局参数、状态机、持久化参数映射
  - `sensor.c`：传感器标定、零点、连接检测
- `Hardware/`
  - 板级驱动与硬件抽象，如 `DI`、`DO`、`RS485`、`MRAM`、编码器等
- `Ethenet/`
  - W5500 相关代码
  - 注意目录名就是 `Ethenet`，不是 `Ethernet`
- `Internet/`
  - 网络协议模块，包含 `MQTT`、`SNMP`、`HTTP`、`FTP`、`DHCP`、`DNS`、`SNTP`、`TFTP`
  - 这些目录多为协议资源；当前主业务通信仍以 `Host/communicate.c + Ethenet/ETHw5500.c` 为主
- `ModbusLib/`
  - Modbus 协议栈与接入说明
  - `MODBUS.md` 为移植说明文档
- `Easylogger/`
  - 日志库与 FreeRTOS 适配
- `PID/`、`ADRC/`、`Regression/`、`CS5530/`
  - 控制算法、识别算法、采集芯片相关模块
- `Drivers/`
  - HAL、CMSIS、DSP/NN 等第三方或厂商代码
- `Middlewares/`
  - `FreeRTOS` 等中间件
- `MDK-ARM/`
  - Keil 工程文件与构建输出
  - 关键文件：`MDK-ARM/shiyanji_H7.uvprojx`

## 3. 构建命令

当前仓库中未发现项目自带的构建脚本（如 `build.bat`、`build.ps1`、`Makefile`、`CMakeLists.txt`）。已确认的构建入口如下：

- 在 `Keil uVision / MDK-ARM` 中打开：`MDK-ARM/shiyanji_H7.uvprojx`
- 目标名：`shiyanji_H7`
- 在 IDE 中执行：`Project -> Rebuild all target files`

已从 `uvprojx` 确认的信息：

- 工具链：`ARM-ADS / ARMCC`
- 芯片：`STM32H750VBTx`
- 输出目录：`MDK-ARM/shiyanji_H7/`
- 已启用 `hex` 输出
- 工程配置了 `fromelf` 后处理生成 `bin` 文件

命令行构建：

- `UV4.exe`/`uVision` 的命令行调用方式在仓库内未提供，具体命令待确认
- 不要自行编造本地安装路径

## 4. 测试命令

当前仓库中未发现项目自建的单元测试或集成测试脚本，也未发现明确可直接运行的测试入口。

已发现但不应当视为本项目测试命令的目录：

- `Drivers/CMSIS/DSP/DSP_Lib_TestSuite/`
- `Drivers/CMSIS/NN/NN_Lib_Tests/`

这些目录属于第三方/CMSIS 自带测试资源，不是本项目业务测试。

当前可执行的最小验证方式：

- 重新编译 `MDK-ARM/shiyanji_H7.uvprojx`
- 如改动涉及控制、通信、采集或存储，进行板级联调/冒烟验证：待确认

自动化测试命令：

- 未发现，待确认

## 5. 代码风格

- 语言：以 `C` 为主，不要将现有 C 模块改写为 C++
- C 标准：Keil 工程开启了 `C99` 相关选项，新增代码应保持 `ARMCC/C99` 兼容
- C++ 标准：项目主体未使用 C++；若必须新增 `.cpp`，标准待确认
- 命名：遵循现有风格，不统一强改
  - 模块文件通常使用 `.c/.h` 配对
  - 全局结构体、状态量、缩写命名较多，修改时优先保持兼容
  - 不要随意重命名对外接口、全局变量、协议字段、寄存器地址宏
- 缩进与排版：
  - 跟随所在文件现有风格
  - 不要只为“格式统一”而重排整个文件
- 注释：
  - 以中文或中英混合短注释为主
  - 注释重点写“为什么/约束/单位/时序”，不要写显而易见的废话
- 格式化：
  - 仓库中未发现统一格式化脚本或 `.clang-format`
  - 不要擅自引入格式化工具并批量改动

## 6. 开发约束

- 不要随意重构公共接口、协议结构、全局参数布局
- 不要删除现有测试资源或示例文件，除非任务明确要求
- 不要引入不必要的新依赖、构建系统或桌面端工具链
- 新增功能优先放在应用自有目录：
  - 硬件相关放 `Hardware/`
  - 业务控制放 `Host/`
  - 协议相关放 `Internet/` 或 `ModbusLib/`
- 尽量避免修改以下目录，除非确有必要：
  - `Drivers/STM32H7xx_HAL_Driver/`
  - `Drivers/CMSIS/`
  - `Middlewares/Third_Party/FreeRTOS/`
  - `MDK-ARM/RTE/`（当前若存在）
- 保持对 `ARMCC` 的兼容性
  - 不要依赖 GCC 专有扩展
  - 不要假设存在 `CMake`、`ninja`、`gcc`、`clang`
- 中断处理保持轻量、确定性，不在 ISR 中做阻塞操作

## 7. Git 工作流

- 修改前先阅读目标模块及其上下游调用
- 提交前至少完成以下检查：
  - 重新编译 `MDK-ARM/shiyanji_H7.uvprojx`
  - 若有可执行测试则运行测试；当前自动化测试命令未发现
  - 若改动影响硬件行为，补充联调结果或注明“待硬件验证”
- 不要提交无关的大规模格式化改动
- 不要把构建产物、日志、临时文件当成源码提交
  - 如 `*.axf`、`*.hex`、`*.bin`、`*.map`、`*.o`、`*.crf`、`*.uvoptx`、`*.log`

## 8. 常见任务指南

### 新增功能

- 先判断功能归属目录，不要跨层随意堆代码
- 尽量复用现有状态机、参数结构和任务框架
- 若新增源文件：
  - 添加 `.c/.h`
  - 更新 `MDK-ARM/shiyanji_H7.uvprojx` 工程分组，确保 Keil 会编译该文件
- 若新增参数需要掉电保存：
  - 同步检查 `gParameter.*`、`mram_manage.*`、`Eeprom_manage.*`

### 修 bug

- 先定位问题属于：
  - 采集/标定
  - 控制算法
  - 协议解析
  - 持久化
  - 任务时序
- 优先做最小修复，不要顺手扩大重构范围
- 涉及协议或存储布局时，先确认兼容性影响

### 添加测试或验证

- 当前未发现项目级自动化测试框架
- 如需补验证，优先选择以下方式：
  - 增加可控的模块自检/调试日志
  - 增加明确的人工验证步骤
  - 在提交说明中写清复现条件、验证方法和结果
- 不要把业务测试代码塞进厂商测试目录

## 9. 注意事项

- 开发环境：`Windows + PowerShell`
- 当前工作目录路径包含空格和括号：
  - `d:\project\shiyanji\shiyanji_H7_250228_V1.19.8 (6)`
  - 命令中请始终对路径加引号
- 换行符：
  - Windows 工程通常优先使用 `CRLF`
  - 如文件已有既定换行风格，保持一致，不要整文件无关改写
- 编码：
  - 仓库中部分 `.md` 文档疑似不是 UTF-8，查看时可能乱码
  - 新增或修改说明文档建议使用 `UTF-8`
- 路径大小写与目录名：
  - 目录名 `Ethenet` 必须按仓库实际名称使用
- Keil 工程注意点：
  - 仅新增文件到磁盘还不够，通常还需要把文件加入 `uvprojx`
  - 若改动启动、内存布局、编译选项，必须明确说明影响

## 附：建议优先阅读的文件

- `Core/Src/main.c`
- `Core/Src/freertos.c`
- `Host/control.c`
- `Host/communicate.c`
- `Host/in_out.c`
- `Host/gParameter.h`
- `Host/gParameter.c`
- `Host/sensor.c`
- `Host/mram_manage.c`
- `Ethenet/ETHw5500.c`
