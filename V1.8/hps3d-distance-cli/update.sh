#!/bin/bash

# Exit on error
set -e

echo "Starting HPS3D service update..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (sudo)"
    exit 1
fi

# Check dependencies
echo "Checking dependencies..."
if ! dpkg -l | grep -q libmosquitto-dev; then
    echo "Installing libmosquitto-dev..."
    apt-get update
    apt-get install -y libmosquitto-dev
fi

# Check if mosquitto service is running
if ! systemctl is-active --quiet mosquitto; then
    echo "Installing and starting mosquitto service..."
    apt-get install -y mosquitto
    systemctl start mosquitto
    systemctl enable mosquitto
fi

# Create directories
echo "Creating directories..."
mkdir -p /usr/local/bin
mkdir -p /usr/local/lib
mkdir -p /etc/hps3d
mkdir -p /var/log/hps3d

# Compile the service
echo "Compiling service..."
make clean
make

# Install binary and libraries
echo "Installing binary and libraries..."
make install

# Verify installation
echo "Verifying installation..."
if ! [ -f /usr/local/bin/hps3d_service ]; then
    echo "ERROR: Binary installation failed"
    exit 1
fi

if ! [ -f /usr/local/lib/libHPS3D.so ]; then
    echo "ERROR: Library installation failed"
    exit 1
fi

if ! [ -f /etc/hps3d/points.conf ]; then
    echo "ERROR: Configuration installation failed"
    exit 1
fi

if ! [ -f /etc/systemd/system/hps3d-service.service ]; then
    echo "ERROR: Service installation failed"
    exit 1
fi

# Test configuration syntax
echo "Testing configuration syntax..."
if ! /usr/local/bin/hps3d_service -t; then
    echo "WARNING: Configuration test failed. Please check /etc/hps3d/points.conf"
    echo "The service may not work correctly until configuration is fixed."
fi

# Restart service if it was running
if systemctl is-active --quiet hps3d-service; then
    echo "Restarting service..."
    systemctl restart hps3d-service
else
    echo "Service is not running"
    echo "To start the service, run: sudo systemctl start hps3d-service"
fi

echo "Installation complete!"
echo ""
echo "Usage:"
echo "1. Edit configuration if needed: sudo nano /etc/hps3d/points.conf"
echo "2. Start service: sudo systemctl start hps3d-service"
echo "3. Enable service at boot: sudo systemctl enable hps3d-service"
echo "4. View logs: sudo journalctl -u hps3d-service -f"
echo ""
echo "MQTT Control:"
echo "Start measurement: mosquitto_pub -t \"hps3d/control\" -m \"start\""
echo "Stop measurement: mosquitto_pub -t \"hps3d/control\" -m \"stop\""
echo ""
echo "Debug log: /var/log/hps3d/debug_hps3d.log" 