# Environmental Monitoring Rover

## Project Structure

```
env_rover_firmware/
├── platformio.ini          # PlatformIO configuration for both robots
├── include/                # Shared header files
│   ├── protocol.h         # Communication protocol definitions
│   ├── lora_link.h        # LoRa communication interface
│   └── app_config.h       # Pin definitions and constants
├── src/
│   ├── common/            # Shared code between robots
│   │   └── lora_link.cpp  # LoRa communication implementation
│   ├── main_robot/        # Main robot firmware
│   │   └── main.cpp       # Main robot entry point
│   └── sub_robot/         # Sub robot firmware
│       └── main.cpp       # Sub robot entry point
└── README.md

```

## Building the Project

### For Main Robot:
```bash
pio run -e main_robot
pio run -e main_robot -t upload
pio device monitor -e main_robot
```

### For Sub Robot:
```bash
pio run -e sub_robot
pio run -e sub_robot -t upload
pio device monitor -e sub_robot
```

## Features

### Main Robot:
- Environmental sensors (water, soil, air quality)
- GPS navigation and obstacle avoidance
- WiFi/MQTT cloud communication
- LoRa communication with sub-robot
- Battery monitoring and auto-return home

### Sub Robot:
- LoRa relay for extended range
- SD card data buffering (offline mode)
- WiFi Access Point (optional)
- Data synchronization with main robot

## Communication Protocol

Both robots use a shared LoRa protocol defined in `include/protocol.h`:
- Packet-based communication
- CRC error checking
- Automatic packet numbering
- RSSI/SNR monitoring

## Next Steps

1. Add sensor implementations to `main_robot/`
2. Add navigation code to `main_robot/`
3. Add SD card storage to `sub_robot/`
4. Add WiFi relay mode to `sub_robot/`
