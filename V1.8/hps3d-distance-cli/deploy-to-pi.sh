#!/bin/bash

# HPS3D LIDAR Service - Raspberry Pi Deployment Script
# This script deploys the service to a Raspberry Pi and configures it

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PI_USER="pi"
DEFAULT_MQTT_BROKER="localhost"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 HPS3D LIDAR Service - Raspberry Pi Deployment${NC}"
echo "=================================================="

# Check if we have the required files
if [ ! -f "$SCRIPT_DIR/lib/libHPS3D.so" ]; then
    echo -e "${RED}❌ ERROR: libHPS3D.so not found in lib/ directory${NC}"
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/hps3d.service" ]; then
    echo -e "${RED}❌ ERROR: hps3d.service file not found${NC}"
    exit 1
fi

# Get Raspberry Pi connection details
read -p "Enter Raspberry Pi IP address: " PI_IP
read -p "Enter Pi username (default: pi): " PI_USER
PI_USER=${PI_USER:-$DEFAULT_PI_USER}

read -p "Enter MQTT broker address (default: localhost): " MQTT_BROKER
MQTT_BROKER=${MQTT_BROKER:-$DEFAULT_MQTT_BROKER}

echo -e "${YELLOW}📋 Deployment Configuration:${NC}"
echo "  Pi IP: $PI_IP"
echo "  Pi User: $PI_USER"
echo "  MQTT Broker: $MQTT_BROKER"
echo ""

# Test connectivity
echo -e "${BLUE}🔌 Testing connection to Raspberry Pi...${NC}"
if ! ssh -o ConnectTimeout=10 $PI_USER@$PI_IP 'echo "Connection successful"' 2>/dev/null; then
    echo -e "${RED}❌ ERROR: Cannot connect to $PI_USER@$PI_IP${NC}"
    echo "Please check:"
    echo "  - IP address is correct"
    echo "  - SSH is enabled on the Pi"
    echo "  - Network connectivity"
    exit 1
fi

echo -e "${GREEN}✅ Connection successful${NC}"

# Build the service
echo -e "${BLUE}🔨 Building service...${NC}"
make clean
make

# Check architecture compatibility
echo -e "${BLUE}🔍 Checking library architecture...${NC}"
LIB_ARCH=$(file lib/libHPS3D.so | grep -o "aarch64\|ARM")
if [ -z "$LIB_ARCH" ]; then
    echo -e "${YELLOW}⚠️  WARNING: Cannot determine library architecture${NC}"
else
    echo -e "${GREEN}✅ Library is compiled for ARM/aarch64 (Raspberry Pi compatible)${NC}"
fi

# Create temporary deployment directory
TEMP_DIR="/tmp/hps3d_deploy_$$"
echo -e "${BLUE}📦 Preparing deployment package...${NC}"

ssh $PI_USER@$PI_IP "mkdir -p $TEMP_DIR"

# Copy files to Pi
echo -e "${BLUE}📤 Copying files to Raspberry Pi...${NC}"
scp hps3d_service lib/libHPS3D.so points.conf.example hps3d.service $PI_USER@$PI_IP:$TEMP_DIR/

# Create installation script on Pi
cat << 'EOF' > /tmp/pi_install.sh
#!/bin/bash
set -e

TEMP_DIR="$1"
MQTT_BROKER="$2"

echo "🔧 Installing HPS3D LIDAR Service on Raspberry Pi..."

# Install dependencies
echo "📦 Installing dependencies..."
sudo apt update
sudo apt install -y mosquitto mosquitto-clients build-essential

# Stop existing service if running
sudo systemctl stop hps3d 2>/dev/null || true

# Install binaries
echo "📁 Installing binaries..."
sudo cp $TEMP_DIR/hps3d_service /usr/local/bin/
sudo chmod +x /usr/local/bin/hps3d_service

# Install library
sudo cp $TEMP_DIR/libHPS3D.so /usr/local/lib/
sudo ldconfig

# Create configuration directory
sudo mkdir -p /etc/hps3d
sudo mkdir -p /var/log/hps3d

# Install configuration
if [ ! -f /etc/hps3d/points.conf ]; then
    sudo cp $TEMP_DIR/points.conf.example /etc/hps3d/points.conf
    echo "✅ Configuration installed to /etc/hps3d/points.conf"
else
    echo "⚠️  Configuration exists, keeping current /etc/hps3d/points.conf"
fi

# Update MQTT broker in service if not localhost
if [ "$MQTT_BROKER" != "localhost" ]; then
    echo "🔧 Updating MQTT broker configuration..."
    # Note: This would require modifying the service code to read MQTT_BROKER from environment
    echo "MQTT_BROKER=$MQTT_BROKER" | sudo tee /etc/hps3d/mqtt.conf
fi

# Install systemd service
sudo cp $TEMP_DIR/hps3d.service /etc/systemd/system/
sudo systemctl daemon-reload

# Set permissions for USB device
sudo usermod -a -G dialout,tty $(whoami)

# Create udev rule for HPS3D device
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-hps3d.rules
sudo udevadm control --reload-rules

echo "✅ Installation complete!"
echo ""
echo "🚀 To start the service:"
echo "   sudo systemctl enable --now hps3d"
echo ""
echo "📊 To check status:"
echo "   sudo systemctl status hps3d"
echo ""
echo "📝 To view logs:"
echo "   sudo journalctl -u hps3d -f"
echo ""
echo "🧪 To test MQTT:"
echo "   mosquitto_pub -h localhost -t hps3d/control -m start"
echo "   mosquitto_sub -h localhost -t hps3d/measurements"

# Cleanup
rm -rf $TEMP_DIR
EOF

# Copy and execute installation script
scp /tmp/pi_install.sh $PI_USER@$PI_IP:$TEMP_DIR/
ssh $PI_USER@$PI_IP "chmod +x $TEMP_DIR/pi_install.sh && $TEMP_DIR/pi_install.sh $TEMP_DIR $MQTT_BROKER"

# Clean up
rm /tmp/pi_install.sh

echo ""
echo -e "${GREEN}🎉 Deployment Complete!${NC}"
echo "=================================================="
echo -e "${BLUE}Next steps:${NC}"
echo "1. SSH to your Pi: ssh $PI_USER@$PI_IP"
echo "2. Start the service: sudo systemctl enable --now hps3d"
echo "3. Check status: sudo systemctl status hps3d"
echo ""
echo -e "${BLUE}🧪 Test MQTT Integration:${NC}"
echo "# Start measurements:"
echo "mosquitto_pub -h $MQTT_BROKER -t hps3d/control -m start"
echo ""
echo "# Get measurements:"
echo "mosquitto_sub -h $MQTT_BROKER -t hps3d/measurements"
echo ""
echo "# Get pointcloud:"
echo "mosquitto_pub -h $MQTT_BROKER -t hps3d/control -m get_pointcloud"
echo "mosquitto_sub -h $MQTT_BROKER -t hps3d/pointcloud"
echo ""
echo -e "${GREEN}🔧 Service is ready for use!${NC}"