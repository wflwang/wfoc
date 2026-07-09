# WFOC - 开源低成本高性能FOC电调

[![Software License](https://img.shields.io/badge/software-MIT-blue.svg)](LICENSE)
[![Hardware License](https://img.shields.io/badge/hardware-CERN--OHL--S%20v2-blue.svg)](LICENSE-HARDWARE)

WFOC (WFL FOC) 是一款面向开源社区的低成本高性能无刷直流电机FOC控制器。项目致力于提供简洁高效的硬件设计和软件实现，让开发者能够快速上手并定制自己的电机控制方案。

## 项目特点

- **低成本**: 采用国产MCU和驱动芯片，BOM成本低
- **高性能**: 支持无感/有感FOC算法，最高PWM频率可达20kHz
- **开源**: 硬件和软件全部开源，便于二次开发
- **简洁**: 硬件设计简洁，软件架构清晰，易于理解和调试

## 硬件架构

### 核心器件

| 器件 | 型号 | 封装 | 功能 |
|------|------|------|------|
| MCU | CIU32F003x | QFN-20 | 主控芯片，32位ARM Cortex-M0+ |
| 栅极驱动 | FD6288 | TSSOP20 | 三相半桥驱动，内置自举二极管 |
| 低压差稳压器 | LD6209 | SOT-23 | 3.3V/5V电源转换 |
| 运算放大器 | BL358 | SOP8 | 电流采样信号放大 |
| MOSFET | N-MOS | TO-252-2 | 功率开关管（3个上管+3个下管） |

### 硬件特性

- **电源输入**: 10-48V DC
- **持续电流**: 20A
- **PWM频率**: 8kHz-20kHz可调
- **电流采样**: 开尔文接法（具体阻值待确认）
- **通信接口**: UART / I2C / CAN（预留）
- **保护功能**: 过流保护、过温保护、欠压保护
- **尺寸**: 50mm x 50mm

### 原理图与PCB

硬件设计文件位于 `hardware/wfoc/` 目录，使用 KiCad 10.0 设计：

- `wfoc.kicad_sch` - 原理图
- `wfoc.kicad_pcb` - PCB布局
- `jlcpcb/` - JLCPCB生产文件（Gerber、BOM、CPL）

## 软件架构

> **注意**: 软件部分正在开发中，欢迎贡献！

### 目录结构

```
software/
├── firmware/          # 固件代码
│   ├── src/           # 源代码
│   │   ├── main.c     # 主程序入口
│   │   ├── foc/       # FOC控制算法
│   │   │   ├── foc.c  # FOC核心算法
│   │   │   ├── pwm.c  # PWM生成
│   │   │   └── svpwm.c# SVPWM实现
│   │   ├── driver/    # 硬件驱动
│   │   │   ├── fd6288.c# 栅极驱动
│   │   │   ├── adc.c  # ADC采样
│   │   │   └── uart.c # 串口通信
│   │   └── utils/     # 工具函数
│   ├── include/       # 头文件
│   ├── config/        # 配置文件
│   └── Makefile       # 编译脚本
└── tools/             # 上位机工具
```

### 核心功能

- [ ] 无感FOC控制（滑模观测器/反电动势法）
- [ ] 有感FOC控制（霍尔传感器/编码器）
- [ ] SVPWM空间矢量调制
- [ ] 电流环PI控制
- [ ] 速度环PI控制
- [ ] 位置环控制
- [ ] 通信协议（UART/I2C/CAN）
- [ ] 保护机制
- [ ] 参数配置与保存

## 开发环境

### 硬件工具

- ST-Link / J-Link 调试器
- USB-TTL 串口模块
- 示波器（推荐）

### 软件工具

- Keil MDK / GCC ARM Embedded
- KiCad 10.0+（硬件设计）
- Python（上位机工具）

## 快速开始

### 硬件组装

1. 使用 `hardware/wfoc/jlcpcb/` 目录下的生产文件打样
2. 焊接所有元器件
3. 连接电机和电源

### 软件编译

```bash
cd software/firmware
make
```

### 烧录固件

```bash
make flash
```

## 通信协议

### UART接口

- 波特率: 115200
- 数据位: 8
- 校验位: None
- 停止位: 1

### 命令格式

```
<SOH><CMD><DATA><CHK><EOT>
```

详细协议文档请参考 `docs/communication_protocol.md`（开发中）。

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

- **MIT License**: 适用于 `software/` 目录下的所有软件代码，允许自由使用、修改和分发
- **CERN-OHL-S v2**: 适用于 `hardware/` 目录下的所有硬件设计文件（原理图、PCB、BOM等），要求衍生作品也必须以相同许可证开源

完整许可证文本请参考上述链接文件。

## 致谢

感谢以下开源项目的启发和参考：

- [SimpleFOC](https://github.com/simplefoc/Arduino-FOC)
- [BLDC-Library](https://github.com/vedderb/bldc)

## 联系方式

如有问题或建议，欢迎通过以下方式联系：

- 邮箱: B5106D@Outlook.com
- GitHub Issues: [提交Issue](https://github.com/wflwang/wfoc/issues)

---

**免责声明**: 本项目仅供学习和研究使用。使用本项目造成的任何损失，作者不承担责任。请在使用前充分测试并确保安全。