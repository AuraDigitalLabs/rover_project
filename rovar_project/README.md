# Environmental Monitoring Rover - PlatformIO Project

This folder contains the PlatformIO firmware for the Environmental Monitoring Rover System.

## Quick Start

```bash
# Build main robot firmware
pio run -e main_robot

# Upload to device
pio run -e main_robot -t upload

# Monitor serial output
pio device monitor -e main_robot
```

## Build Environments

| Environment | Description |
|-------------|-------------|
| `main_robot` | Main robot with sensors & motors |
| `sub_robot` | Sub robot relay & SD logging |
| `main_robot_control` | Main robot joystick controller |
| `sub_robot_control` | Sub robot joystick controller |

## Documentation

See the main [README.md](../README.md) in the parent folder for complete documentation including:
- System architecture
- Hardware requirements
- Pin configurations
- Communication protocol
- Usage instructions

- Environmental Monitoring Rover Team
