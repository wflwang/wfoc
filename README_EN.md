
# WFOC - Open-source Low-cost High-performance FOC ESC

[![Software License](https://img.shields.io/badge/software-MIT-blue.svg)](LICENSE)
[![Hardware License](https://img.shields.io/badge/hardware-CERN--OHL--S%20v2-blue.svg)](LICENSE-HARDWARE)

WFOC (WFL FOC) is an open-source, low-cost, high-performance brushless DC motor FOC (Field Oriented Control) controller. The project aims to provide a concise and efficient hardware design and software implementation so developers can quickly get started and customize their motor control solutions.

## Project Highlights

- **Low Cost**: Uses domestically available MCU and driver chips to keep BOM costs low
- **High Performance**: Supports sensorless and sensored FOC algorithms, with PWM frequencies up to 20 kHz
- **Open Source**: Both hardware and software are fully open-source for easy modification
- **Simple Design**: Clean hardware design and clear software architecture for easy understanding and debugging

## Hardware Architecture

### Key Components

| Component | Part | Package | Function |
|-----------|------|---------|---------|
| MCU | CIU32F003x | QFN-20 | Main controller, 32-bit ARM Cortex-M0+ |
| Gate Driver | FD6288 | TSSOP20 | Three-phase half-bridge driver, internal bootstrap diodes |
| LDO | LD6209 | SOT-23 | 3.3V/5V voltage regulation |
| Op Amp | BL358 | SOP8 | Current-sense signal amplification |
| Power IC | EG1192 | ESOP8 | DC-DC converter |
| MOSFET | N-MOS | TO-252-2 | Power switching transistors (3 high + 3 low) |

### Hardware Specs

- **Input Voltage**: 10–48 V DC
- **Continuous Current**: 20 A
- **PWM Frequency**: Adjustable 8 kHz–20 kHz
- **Current Sensing**: Kelvin connection (resistor values TBD)
- **Interfaces**: UART / I2C / CAN (reserved)
- **Protections**: Over-current, over-temperature, under-voltage
- **Dimensions**: 50 mm × 50 mm

### Schematics & PCB

Hardware design files are in the `hardware/wfoc/` directory and were created with KiCad 10.0:

- `wfoc.kicad_sch` — schematic
- `wfoc.kicad_pcb` — PCB layout
- `jlcpcb/` — JLCPCB production files (Gerber, BOM, CPL)

## Software Architecture

> **Note**: Software is under development — contributions are welcome!

### Directory Layout

```
software/
├── firmware/          # Firmware source
│   ├── src/           # Source code
│   │   ├── main.c     # Main entry
│   │   ├── foc/       # FOC control algorithms
│   │   │   ├── foc.c  # FOC core
│   │   │   ├── pwm.c  # PWM generation
│   │   │   └── svpwm.c# SVPWM implementation
│   │   ├── driver/    # Hardware drivers
│   │   │   ├── fd6288.c# Gate driver
│   │   │   ├── adc.c  # ADC sampling
│   │   │   └── uart.c # UART communications
│   │   └── utils/     # Utility functions
│   ├── include/       # Header files
│   ├── config/        # Config files
│   └── Makefile       # Build script
└── tools/             # Host-side tools
```

### Core Features (Planned)

- [ ] Sensorless FOC (sliding-mode observer / back-EMF methods)
- [ ] Sensored FOC (Hall sensors / encoder)
- [ ] SVPWM (Space Vector PWM)
- [ ] Current loop (PI)
- [ ] Speed loop (PI)
- [ ] Position control
- [ ] Communication protocols (UART / I2C / CAN)
- [ ] Protection mechanisms
- [ ] Parameter configuration and persistence

## Development Environment

### Hardware Tools

- ST-Link / J-Link debugger
- USB-TTL serial adapter
- Oscilloscope (recommended)

### Software Tools

- Keil MDK / GCC ARM Embedded
- KiCad 10.0+ (for hardware design)
- Python (for host tools)

## Quick Start

### Hardware Assembly

1. Order boards using production files in `hardware/wfoc/jlcpcb/`
2. Solder all components
3. Connect the motor and power supply

### Build Firmware

```bash
cd software/firmware
make
```

### Flash Firmware

```bash
make flash
```

## Communication Protocol

### UART Settings

- Baudrate: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1

### Frame Format

```
<SOH><CMD><DATA><CHK><EOT>
```

See `docs/communication_protocol.md` for protocol details (in progress).

## Contributing

Contributions in any form are welcome. Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit your changes (`git commit -am 'Add some feature'`)
4. Push the branch (`git push origin feature/your-feature`)
5. Open a Pull Request

## License

This project uses a dual-license scheme:

| Part | License | File |
|------|--------|------|
| Software | MIT License | [LICENSE](LICENSE) |
| Hardware | CERN-OHL-S v2 | [LICENSE-HARDWARE](LICENSE-HARDWARE) |

- **MIT License**: Applies to the software under `software/` and permits free use, modification, and distribution.
- **CERN-OHL-S v2**: Applies to hardware design files under `hardware/` (schematics, PCB, BOM, etc.) and requires derived works to be open under the same license.

Please refer to the linked license files for the full text.

## Acknowledgements

Thanks to these open-source projects for inspiration and reference:

- [SimpleFOC](https://github.com/simplefoc/Arduino-FOC)
- [BLDC-Library](https://github.com/vedderb/bldc)

## Contact

If you have questions or suggestions, please contact:

- Email: B5106D@Outlook.com
- GitHub Issues: [Submit an Issue](https://github.com/wflwang/wfoc/issues)

---

**Disclaimer**: This project is for learning and research purposes only. The authors are not responsible for any damages resulting from its use. Please test thoroughly and ensure safe operation before use.
