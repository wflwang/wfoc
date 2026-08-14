# WFOC - 开源低成本高性能FOC电调

[![Software License](https://img.shields.io/badge/software-MIT-blue.svg)](LICENSE)
[![Hardware License](https://img.shields.io/badge/hardware-CERN--OHL--S%20v2-blue.svg)](LICENSE-HARDWARE)
[![FW Version](https://img.shields.io/badge/firmware-v1.0.0-green.svg)](software/config/fw_version.h)

WFOC (WFL FOC) 是一款面向开源社区的低成本高性能无刷直流电机 FOC 控制器。项目参考 STM32 FOC 驱动、VESC 开源代码和 SimpleFOC，在国产 CIU32F003x5 (ARM Cortex-M0+) 上实现了完整的电机控制固件。

- 开源地址: https://github.com/wflwang/wfoc.git
- 邮箱: B5106D@Outlook.com

## 项目特点

- **低成本**: 采用国产 MCU (CIU32F003x5) 和 FD6288 栅极驱动，BOM 成本低
- **高性能**: 中断内全定点 Q16.16 运算（无浮点），20kHz PWM 电流环
- **双模式**: 同时支持 FOC 矢量控制和 BLDC 六步换相，可通过 `comm_mode` 配置切换
- **多观测器**: 滑模观测器 (SMO)、MXLEMMA 全阶观测器、MXV_Lambda 补偿、高频注入 (HFI)
- **VESC 风格协议**: 二进制 UART 协议，支持参数读写、实时遥测、高速流式打印
- **开源**: 硬件 (CERN-OHL-S v2) 和软件 (MIT) 全部开源

## 硬件架构

### 核心器件

| 器件 | 型号 | 封装 | 功能 |
|------|------|------|------|
| MCU | CIU32F003x5 | QFN-20 | 32位 ARM Cortex-M0+, 24MHz, 32KB Flash |
| 栅极驱动 | FD6288 | TSSOP-20 | 三相半桥驱动，内置自举二极管 |
| 运算放大器 | XOPA2333 | - | 2通道电流采样放大 |
| MOSFET | N-MOS | TO-252-2 | 3上 + 3下功率开关管 |

### 硬件特性

- **电源输入**: 10-48V DC
- **持续电流**: 20A（峰值 60A）
- **PWM 频率**: 8kHz-20kHz 可调（默认 20kHz）
- **电流采样**: 2路下桥臂电阻采样 (0.005R + 20x 运放)
- **通信接口**: UART (115200~921600) / PPM
- **保护功能**: 过流、过压、欠压、过温保护
- **调试接口**: SWD

### 引脚映射

引脚定义见 [board_config.h](software/config/board_config.h)，摘要：

| 功能 | 引脚 | 说明 |
|------|------|------|
| UART TX/RX | PC0 / PB5 | 串口通信 |
| PWM A 高/低 | PB2 / PB3 | A相上下管 |
| PWM B 高/低 | PC1 / PB7 | B相上下管 |
| PWM C 高/低 | PA1 / PA0 | C相上下管 |
| 电流采样 A/B | PB0 / PB1 | ADC CH0/CH1 |
| 相电压 U/V/W | PA3 / PA4 / PA5 | ADC CH3/CH4/CH5 |
| 母线电压 | PA7 | ADC CH7 |
| 温度 | PA6 | ADC CH6 |
| PPM 输入 | PB4 | TIM2 输入捕获 |

## 软件架构

### 目录结构

```
software/
├── main.c                  # 主程序入口（初始化与主循环）
├── interrupts.c            # 中断服务（ADC1 FOC环 / UART / PPM / SysTick）
├── Makefile                # 编译脚本
│
├── app/                    # 应用层
│   ├── app.c               # 状态机、安全监督、斜坡控制
│   ├── app_uart.c          # UART控制接口（VESC风格协议、实时遥测）
│   ├── app_ppm.c           # PPM脉冲解码
│   └── app_sound.c         # 电机发声
│
├── motor/                  # 电机控制核心
│   ├── mcpwm.c             # PWM后端桥接（duty/current/speed/position）
│   ├── observer.c          # 观测器（SMO / MXLEMMA / HFI + PLL）
│   ├── bldc.c              # BLDC六步换相（多种反电动势检测）
│   └── encoder.c           # 编码器接口
│
├── driver/mcu/             # MCU外设驱动
│   ├── adc.c               # ADC驱动（DMA多通道，偏置校准）
│   ├── pwm.c               # PWM驱动（TIM1三相中心对齐）
│   ├── uart.c              # UART驱动（环形缓冲收发）
│   ├── timer.c             # 定时器（PPM捕获/看门狗）
│   ├── gpio.c              # GPIO配置
│   ├── mcu_init.c          # 系统初始化（时钟/SysTick）
│   └── ciu32f003x.h        # MCU寄存器定义
│
├── conf/                   # 配置系统
│   ├── datatypes.h         # 核心数据类型（motor_state_t / mc_configuration_t）
│   ├── conf_default.c      # 默认参数
│   └── conf_general.c      # 配置管理（持久化接口）
│
├── config/                 # 编译时配置
│   ├── board_config.h      # 板级引脚映射与硬件参数
│   ├── hwconf.h            # 硬件目标选择（make HW=WFoc_V1）
│   └── fw_version.h        # 固件版本与功能开关
│
├── packet/                 # 通信协议
│   └── packet.c            # VESC风格帧封装（CRC16/CCITT）
│
├── startup/                # 启动文件
│   ├── linker.ld           # 链接脚本
│   └── startup_ciu32f003x.s# 启动汇编
│
└── util/                   # 工具库
    └── fixedpoint.c        # 定点数数学库（Q16.16, 三角函数, PI/PID, 陷波滤波）
```

### 数据流

```
ADC1 ISR (20kHz) ──► 电流采样 ──► Clarke ──► 观测器(角度/转速)
                                       │              │
                                       ▼              ▼
                                     Park ──► PI电流环 ──► 逆Park
                                                          │
                                                       SVPWM ──► TIM1 PWM
                                                          │
应用层 (main loop)                                        │
  app_process() ──► mcpwm_set_*() ──► foc_state.iq_set ───┘
  app_uart_process() ──► 遥测发送 ──► UART
```

### 核心功能实现状态

- [x] FOC 矢量控制（Clarke/Park/SVPWM/电流环PI）
- [x] BLDC 六步换相（ADC/比较器/过零检测 + 参数自学习）
- [x] 观测器（SMO / MXLEMMA / HFI + PLL 锁相环）
- [x] 定点数数学库（Q16.16，中断内无浮点）
- [x] VESC 风格 UART 协议（参数读写、固件版本、终端命令）
- [x] 实时遥测（3相电流、3相电压、转速、d/q轴等）
- [x] 高速流式打印（COMM_GET_VALUES_SELECTIVE + streaming）
- [x] ADC 偏置自校准（上电自动测量零电流偏移）
- [x] 安全监督（过流/过压/欠压/过温保护 + 故障状态机）
- [x] PPM 输入控制
- [x] 电机发声功能
- [ ] 速度闭环 PI（当前为前馈，参数已预留）
- [ ] 位置闭环控制
- [ ] 弱磁控制 / 过调制 / 陷波滤波（框架已建，待接入电流环）
- [ ] Flash 参数持久化（接口已定义，存储待实现）

## 编译环境

### 工具链安装

固件目标为 ARM Cortex-M0+，需要 `arm-none-eabi-gcc` 交叉编译工具链。

**Windows:**
1. 下载 [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
2. 解压并将 `bin` 目录加入系统 `PATH`
3. 安装 [Make for Windows](http://gnuwin32.sourceforge.net/packages/make.htm) 或使用 MSYS2

**Linux (Ubuntu/Debian):**
```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi openocd
```

**macOS (Homebrew):**
```bash
brew install arm-gcc-bin openocd
```

### 验证工具链

```bash
arm-none-eabi-gcc --version    # 需要 10.x 或以上
arm-none-eabi-objcopy --version
openocd --version              # 烧录用
```

### 仿真环境（可选）

仿真在 PC 上运行，验证 FOC 定点算法，只需系统自带 GCC：

```bash
gcc --version    # Windows 可用 MinGW / MSYS2 的 gcc
```

## 编译步骤

### 编译固件

```bash
cd software
make              # 默认硬件目标 WFoc_V1
```

指定硬件目标与优化：
```bash
make HW=WFoc_V1   # 选择硬件版本（当前支持 V1）
make clean         # 清理构建产物
make size          # 查看 Flash/RAM 占用
make info          # 打印源文件列表和编译选项
```

编译成功后输出：
- `build/wfoc.elf` - ELF 可执行文件
- `build/wfoc.hex` - Intel HEX（烧录用）
- `build/wfoc.bin` - 二进制文件
- `build/wfoc.map` - 链接映射表

### 烧录固件

通过 ST-Link / J-Link + OpenOCD 烧录：
```bash
make flash        # 调用 openocd 烧录 build/wfoc.hex
```

也可手动烧录：
```bash
openocd -f interface/stlink.cfg -f target/ciu32f003x.cfg \
    -c "program build/wfoc.hex verify reset exit 0x08000000"
```

### 运行仿真

```bash
# Windows / Linux / macOS 均可
gcc -O2 -o simulation/sim_foc.exe simulation/sim_main.c software/util/fixedpoint.c -lm
./simulation/sim_foc.exe        # Linux/macOS
.\simulation\sim_foc.exe        # Windows PowerShell
```

仿真说明：
- 仿真使用**真实的定点数库** (`fixedpoint.c`)，与固件 ISR 完全相同的 Q16.16 运算
- `sim_main.c` 顶部 `SIM_USE_PERFECT_ANGLE` 开关：
  - `1` = 完美角度（验证 FOC 电流环本身），电机应加速到 400+ RPM
  - `0` = 观测器模式（验证 FOC + 观测器），验证角度估算

## 通信协议

### UART 接口

- 默认波特率: 115200（可通过参数调整为 921600 以支持高速打印）
- 格式: 8N1
- 协议: VESC 风格二进制帧

### 帧格式

```
短帧 (<256字节):  [0x02] [len] [payload...] [crc_hi] [crc_lo] [0x03]
长帧 (>=256字节): [0x03] [len_hi] [len_lo] [payload...] [crc_hi] [crc_lo] [0x03]
```
- `payload` = `[command] [data...]`
- CRC: CRC-16/CCITT (poly 0x1021, init 0xFFFF)

### 支持的命令

| 命令 | 名称 | 说明 |
|------|------|------|
| 0 | COMM_FW_VERSION | 获取固件版本与硬件名称 |
| 2 | COMM_GET_VALUES | 获取完整实时数据（含3相电流/电压/转速） |
| 4 | COMM_SET_DUTY | 设置占空比 (x1e5) |
| 5 | COMM_SET_CURRENT | 设置电流 (x1e3) |
| 6 | COMM_SET_CURRENT_BRAKE | 制动电流 (x1e3) |
| 7 | COMM_SET_RPM | 设置转速 ERPM |
| 8 | COMM_SET_POS | 设置位置 (x1e6) |
| 11 | COMM_GET_MCCONF | 读取电机配置 |
| 12 | COMM_SET_MCCONF | 写入电机配置 |
| 13/14 | COMM_GET/SET_APPCONF | 读取/写入应用配置 |
| 15 | COMM_DETECT_MOTOR_PARAM | 电机参数检测 |
| 17 | COMM_TERMINAL_CMD | 终端命令 (help/stop/restart/alive) |
| 18 | COMM_GET_VALUES_SELECTIVE | 高速选择性遥测（见下） |

### 实时遥测与高速打印

**完整遥测 (COMM_GET_VALUES)**：请求一次返回一帧，包含温度、3相电流、3相电压、d/q轴、占空比、转速、位置、母线电压、功率、故障码等。

**高速选择性遥测 (COMM_GET_VALUES_SELECTIVE)**：

上位机发送 `[mask:u32][rate_ms:u16]`，固件按位掩码只打包需要的字段，大幅减小数据包。`rate_ms > 0` 时开启**主动流式发送**（streaming），固件在主循环按设定间隔自动发送，无需反复请求。

字段掩码位定义（见 [datatypes.h](software/conf/datatypes.h)）：
```
bit 0-2:   IA / IB / IC     (3相电流)
bit 3-5:   VA / VB / VC     (3相电压)
bit 6-9:   ID / IQ / VD / VQ (d/q轴电流电压)
bit 10:    DUTY
bit 11:    SPEED (ERPM)
bit 12:    POSITION (rad)
bit 13:    VBUS
bit 14-15: TEMP_MOS / TEMP_MOT
bit 16:    POWER
```

示例：以 1ms 速率只打印 3相电流 + 3相电压 + 转速：
```
mask = IA|IB|IC|VA|VB|VC|SPEED = 0x208F
发送: COMM_GET_VALUES_SELECTIVE + [0x8F,0x20,0x00,0x00] + [0x01,0x00]
```

**实现原理**（高速打印关键）：
1. ADC 中断 (20kHz) 只采集数据存入 `motor_state` 变量，**不在中断中发串口**
2. 主循环中一次性组装数据包通过 UART 发送
3. streaming 模式避免请求-响应往返延迟
4. 提高波特率 (921600) 可进一步缩短发送时间

## 配置与调参

所有电机参数可通过 UART 在线调整（COMM_GET/SET_MCCONF），参数定义见 [datatypes.h](software/conf/datatypes.h) 的 `mc_configuration_t`，默认值见 [conf_default.c](software/conf/conf_default.c)。

关键参数：
- `comm_mode`: 0=FOC, 1=BLDC
- `observer_type`: 0=无, 1=SMO, 2=MXLEMMA, 3=MXV_Lambda, 4=HFI
- `motor_r / motor_l / motor_flux_linkage / motor_poles`: 电机参数
- `foc_current_kp / ki`: 电流环 PI 增益
- `foc_speed_kp / ki`: 速度环 PI 增益
- `l_max_current / l_max_speed / l_max_duty`: 限幅

## 贡献指南

欢迎任何形式的贡献！请遵循以下步骤：

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/your-feature`)
3. 提交代码 (`git commit -am 'Add some feature'`)
4. 推送到分支 (`git push origin feature/your-feature`)
5. 创建 Pull Request

## 许可证

本项目采用双重许可证：

| 部分 | 许可证 | 文件 |
|------|--------|------|
| 软件代码 | MIT License | [LICENSE](LICENSE) |
| 硬件设计文件 | CERN-OHL-S v2 | [LICENSE-HARDWARE](LICENSE-HARDWARE) |

## 致谢

感谢以下开源项目的启发和参考：

- [VESC](https://github.com/vedderb/bldc) - BLDC 开源电调
- [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) - 简易 FOC 库

## 联系方式

- 邮箱: B5106D@Outlook.com
- GitHub Issues: [提交 Issue](https://github.com/wflwang/wfoc/issues)

---

**免责声明**: 本项目仅供学习和研究使用。使用本项目造成的任何损失，作者不承担责任。请在使用前充分测试并确保安全。
