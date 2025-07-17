# HPS3D Distance CLI

## Overview
The HPS3D Distance CLI is a command-line tool designed to interact with the HPS3D sensor. It allows users to query the distance measurements from the sensor based on specific pixel range inputs.

## Project Structure
```
hps3d-distance-cli
├── src
│   ├── main.c          # Entry point of the command-line tool
│   └── HPS3DUser_IF.h  # Header file for HPS3D sensor interface
├── Makefile            # Build instructions for the project
└── README.md           # Documentation for the project
```

## Requirements
- C Compiler (e.g., GCC)
- HPS3D SDK

## Building the Project
To compile the project, navigate to the project directory and run the following command:

```
make
```

This will generate the executable for the command-line tool.

## Running the Tool
After building the project, you can run the tool using the following command:

```
./hps3d-distance-cli
```

## Usage
Once the tool is running, you can input pixel range values to query the distance measurements from the HPS3D sensor. Follow the prompts in the command line to enter your inputs.

## Dependencies
Ensure that the HPS3D SDK is properly installed and configured on your system before running the tool.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.

# HPS3D-160 LIDAR Service

## MQTT Troubleshooting Guide

### 1. Check MQTT Service Status
```bash
# Check if mosquitto is running
sudo systemctl status mosquitto

# If not running, start it
sudo systemctl start mosquitto

# Enable auto-start at boot
sudo systemctl enable mosquitto
```

### 2. Test MQTT Communication
```bash
# Install mosquitto client tools if not present
sudo apt-get install mosquitto-clients

# Open a terminal and subscribe to the measurement topic
mosquitto_sub -v -t "hps3d/#"

# In another terminal, try to publish a command
mosquitto_pub -t "hps3d/control" -m "start"
```

### 3. Check MQTT Logs
```bash
# View mosquitto logs
sudo journalctl -u mosquitto -f

# View HPS3D service logs
sudo journalctl -u hps3d-service -f
```

### 4. Common Issues and Solutions

#### MQTT Connection Failed
If the service can't connect to MQTT:
1. Verify mosquitto is running
2. Check configuration:
   ```bash
   # Show mosquitto config
   cat /etc/mosquitto/mosquitto.conf
   
   # Default config should allow local connections
   sudo tee /etc/mosquitto/mosquitto.conf << EOF
   listener 1883
   allow_anonymous true
   EOF
   
   # Restart mosquitto
   sudo systemctl restart mosquitto
   ```

#### Permission Issues
If you get permission errors:
```bash
# Check mosquitto user and group
sudo groups mosquitto

# Add required permissions
sudo usermod -a -G dialout mosquitto
sudo usermod -a -G tty mosquitto

# Restart services
sudo systemctl restart mosquitto
sudo systemctl restart hps3d-service
```

#### Port Already in Use
If port 1883 is already in use:
1. Find what's using the port:
   ```bash
   sudo netstat -tulpn | grep 1883
   ```
2. Either stop the conflicting service or change mosquitto's port:
   ```bash
   # Edit mosquitto config
   sudo nano /etc/mosquitto/mosquitto.conf
   
   # Change port number (e.g., to 1884)
   listener 1884
   
   # Update HPS3D service config
   sudo nano /etc/hps3d/points.conf
   # Add: mqtt_port=1884
   ```

#### Debug Mode
Enable debug logging for more information:
1. Edit configuration:
   ```bash
   sudo nano /etc/hps3d/points.conf
   ```
2. Set debug options:
   ```
   debug=1
   debug_file=/var/log/hps3d/debug_hps3d.log
   ```
3. Restart service:
   ```bash
   sudo systemctl restart hps3d-service
   ```

### 5. Testing MQTT Communication

Test basic connectivity:
```bash
# Subscribe to all HPS3D topics
mosquitto_sub -v -t "hps3d/#"

# In another terminal, send commands:
# Start measurement
mosquitto_pub -t "hps3d/control" -m "start"

# Stop measurement
mosquitto_pub -t "hps3d/control" -m "stop"
```

### 6. Verify MQTT Settings

Check current MQTT broker settings:
```bash
# Show all MQTT connections
sudo netstat -tulpn | grep mosquitto

# Check MQTT broker version
mosquitto -h

# Test MQTT broker configuration
mosquitto -t /etc/mosquitto/mosquitto.conf
```

### 7. Reset MQTT Setup

If nothing else works, completely reset MQTT:
```bash
# Stop services
sudo systemctl stop hps3d-service
sudo systemctl stop mosquitto

# Remove and reinstall mosquitto
sudo apt-get remove --purge mosquitto mosquitto-clients
sudo apt-get install mosquitto mosquitto-clients

# Reset HPS3D service
sudo systemctl restart hps3d-service
```

### Support

If problems persist:
1. Collect logs:
   ```bash
   # Collect all relevant logs
   sudo journalctl -u mosquitto > mosquitto.log
   sudo journalctl -u hps3d-service > hps3d.log
   ```
2. Check system resources:
   ```bash
   # Check system load
   top
   
   # Check disk space
   df -h
   
   # Check memory
   free -h
   ```

For additional help, please create an issue with:
- Complete error messages
- Service logs
- System information
- Steps to reproduce the problem