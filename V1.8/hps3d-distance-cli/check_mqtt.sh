#!/bin/bash

echo "MQTT Port Diagnostic Tool"
echo "------------------------"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (sudo)"
    exit 1
fi

echo "1. Checking MQTT port (1883)..."
if netstat -tuln | grep :1883 > /dev/null; then
    echo "Port 1883 is in use:"
    netstat -tuln | grep :1883
    
    echo -e "\nProcesses using port 1883:"
    lsof -i :1883
    
    # Check mosquitto status
    echo -e "\nMosquitto service status:"
    systemctl status mosquitto
else
    echo "Port 1883 is free"
fi

echo -e "\n2. Checking for multiple mosquitto instances..."
ps aux | grep mosquitto | grep -v grep

echo -e "\n3. Checking mosquitto configuration..."
if [ -f /etc/mosquitto/mosquitto.conf ]; then
    echo "Mosquitto config exists:"
    cat /etc/mosquitto/mosquitto.conf
else
    echo "No mosquitto config found"
fi

echo -e "\n4. Attempting to fix..."

# Stop all mosquitto processes
echo "Stopping mosquitto service..."
systemctl stop mosquitto

# Kill any remaining mosquitto processes
echo "Killing remaining mosquitto processes..."
pkill mosquitto

# Wait a moment
sleep 2

# Check if port is now free
if netstat -tuln | grep :1883 > /dev/null; then
    echo "WARNING: Port 1883 is still in use!"
    echo "You may need to:"
    echo "1. Reboot the system, or"
    echo "2. Change the MQTT port in the configuration"
else
    echo "Port 1883 is now free"
    
    # Start mosquitto service
    echo "Starting mosquitto service..."
    systemctl start mosquitto
    
    # Check status
    echo -e "\nFinal status:"
    systemctl status mosquitto
fi

echo -e "\nTo test MQTT after fixing:"
echo "1. mosquitto_sub -v -t \"test/#\""
echo "2. In another terminal: mosquitto_pub -t \"test\" -m \"hello\"" 