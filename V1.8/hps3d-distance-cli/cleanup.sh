#!/bin/bash

# Exit on error
set -e

echo "Starting cleanup of HPS3D service..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (sudo)"
    exit 1
fi

# Stop all related services
echo "Stopping services..."
systemctl stop hps3d.service 2>/dev/null || true
systemctl stop hps3d-service.service 2>/dev/null || true

# Disable services
echo "Disabling services..."
systemctl disable hps3d.service 2>/dev/null || true
systemctl disable hps3d-service.service 2>/dev/null || true

# Remove service files
echo "Removing service files..."
rm -f /etc/systemd/system/hps3d.service
rm -f /etc/systemd/system/hps3d-service.service
rm -f /etc/systemd/system/multi-user.target.wants/hps3d.service
rm -f /etc/systemd/system/multi-user.target.wants/hps3d-service.service

# Remove old binaries
echo "Removing old binaries..."
rm -f /usr/local/bin/hps3d_service
rm -f /usr/local/bin/hps3d

# Remove old library files
echo "Removing old libraries..."
rm -f /usr/local/lib/libHPS3D.so
rm -f /usr/local/lib/libHPS3DSDK.so

# Remove old config files
echo "Removing old config files..."
rm -rf /etc/hps3d

# Remove old log files
echo "Removing old log files..."
rm -f /var/log/hps3d_service.log
rm -f /var/log/hps3d.log
rm -rf /var/log/hps3d

# Remove old PID files
echo "Removing old PID files..."
rm -f /var/run/hps3d_service.pid
rm -f /var/run/hps3d.pid

# Reload systemd
echo "Reloading systemd..."
systemctl daemon-reload
systemctl reset-failed

echo "Cleanup complete!"
echo ""
echo "To reinstall the service, run:"
echo "sudo ./update.sh" 