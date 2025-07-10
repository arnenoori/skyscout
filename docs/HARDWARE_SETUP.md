# Hardware Setup Guide

Basic setup instructions for SkyScout hardware components.

## Required Components

- **Raspberry Pi 5** (8GB recommended)
- **Pixhawk 6C** flight controller
- **USB/CSI Camera** for object detection
- **PX4-compatible drone frame**
- **Power distribution board**
- **GPS module** (required for autonomous flight)
- **RC transmitter** (for safety override)

## Quick Setup

### 1. Raspberry Pi Setup

```bash
# Flash Ubuntu Server 22.04 to SD card
# Enable SSH and configure WiFi

# After boot, install basics:
sudo apt update && sudo apt upgrade -y
sudo apt install git python3-pip
```

### 2. Connect Pixhawk

- Connect Pixhawk TELEM2 to Pi USB (via FTDI)
- Default baudrate: 921600
- Connection string: `/dev/ttyUSB0`

### 3. Camera Configuration

**USB Camera**:
```bash
# Verify detection
ls /dev/video*
# Should show /dev/video0
```

**CSI Camera** (Pi Camera):
```bash
# Enable in /boot/firmware/config.txt
camera_auto_detect=1
```

### 4. Basic Wiring

```
Pixhawk 6C <--TELEM2--> USB-to-Serial <---> Raspberry Pi 5
     |
     ├── GPS Module (GPS1 port)
     ├── RC Receiver (RC IN)
     └── ESCs/Motors (MAIN OUT)

Raspberry Pi 5 <---> Camera (USB/CSI)
```

## Network Setup

Configure Pi as access point or connect to existing network:

```bash
# Install network manager
sudo apt install network-manager
sudo nmtui  # Configure network
```

## Power Considerations

- Pi 5 needs 5V/5A (25W) under load
- Use quality BEC or separate battery
- Monitor voltage to prevent brownouts

## Pre-flight Checklist

1. ✅ All connections secure
2. ✅ GPS has clear sky view
3. ✅ Camera detected by system
4. ✅ RC transmitter bound and tested
5. ✅ Battery voltage checked
6. ✅ Props installed correctly

## Troubleshooting

- **No MAVLink connection**: Check USB permissions `sudo chmod 666 /dev/ttyUSB0`
- **Camera not found**: Try different USB port or check CSI cable
- **GPS not locking**: Ensure outdoor location with clear sky

For detailed pinouts and advanced configurations, refer to manufacturer documentation.
