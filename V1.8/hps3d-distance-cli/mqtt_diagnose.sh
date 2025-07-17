#!/bin/bash

echo "MQTT Connection Diagnostic Tool"
echo "-----------------------------"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (sudo)"
    exit 1
fi

# Function to check port
check_port() {
    if netstat -tuln | grep ":1883 " > /dev/null; then
        echo "[OK] Port 1883 is open"
        netstat -tuln | grep ":1883 "
    else
        echo "[ERROR] Port 1883 is not open!"
        echo "Checking what might be blocking..."
        
        # Check firewall
        if command -v ufw >/dev/null 2>&1; then
            echo "UFW status:"
            ufw status | grep 1883
        fi
        
        # Check if something else uses the port
        echo "Checking port usage:"
        lsof -i :1883
    fi
}

# Function to check mosquitto status
check_mosquitto() {
    echo "Checking Mosquitto service..."
    if systemctl is-active --quiet mosquitto; then
        echo "[OK] Mosquitto is running"
    else
        echo "[ERROR] Mosquitto is not running!"
        echo "Mosquitto status:"
        systemctl status mosquitto
    fi
}

# Function to check configuration
check_config() {
    echo "Checking Mosquitto configuration..."
    
    if [ -f /etc/mosquitto/mosquitto.conf ]; then
        echo "[OK] Configuration file exists"
        
        # Check permissions
        ls -l /etc/mosquitto/mosquitto.conf
        
        # Check key settings
        echo "Configuration settings:"
        grep -E "listener|allow_anonymous|password_file" /etc/mosquitto/mosquitto.conf
        
        # Validate config
        mosquitto -t /etc/mosquitto/mosquitto.conf
    else
        echo "[ERROR] No configuration file found!"
        
        # Create basic config
        echo "Creating basic configuration..."
        cp mosquitto.conf /etc/mosquitto/mosquitto.conf
        chown mosquitto:mosquitto /etc/mosquitto/mosquitto.conf
        chmod 644 /etc/mosquitto/mosquitto.conf
        
        echo "Restarting Mosquitto..."
        systemctl restart mosquitto
    fi
}

# Function to check logs
check_logs() {
    echo "Checking Mosquitto logs..."
    
    if [ -f /var/log/mosquitto/mosquitto.log ]; then
        echo "Last 10 log entries:"
        tail -n 10 /var/log/mosquitto/mosquitto.log
    else
        echo "[WARNING] No log file found"
    fi
}

# Function to test MQTT connection
test_connection() {
    echo "Testing MQTT connection..."
    
    # Install mosquitto-clients if not present
    if ! command -v mosquitto_pub >/dev/null 2>&1; then
        echo "Installing mosquitto-clients..."
        apt-get install -y mosquitto-clients
    fi
    
    # Test publish
    echo "Testing publish..."
    if mosquitto_pub -h localhost -t test -m "test" -W 1; then
        echo "[OK] Publish test successful"
    else
        echo "[ERROR] Publish test failed"
    fi
}

# Function to check permissions
check_permissions() {
    echo "Checking permissions..."
    
    # Check mosquitto user
    if id mosquitto >/dev/null 2>&1; then
        echo "[OK] Mosquitto user exists"
        echo "User groups:"
        groups mosquitto
    else
        echo "[ERROR] Mosquitto user not found!"
    fi
    
    # Check directory permissions
    echo "Directory permissions:"
    ls -ld /var/lib/mosquitto
    ls -ld /var/log/mosquitto
}

# Main diagnostic flow
echo "Starting diagnostics..."
check_mosquitto
echo "-----------------------------"
check_port
echo "-----------------------------"
check_config
echo "-----------------------------"
check_permissions
echo "-----------------------------"
check_logs
echo "-----------------------------"
test_connection
echo "-----------------------------"

echo "Diagnostic complete. If problems persist, try these steps:"
echo "1. Restart Mosquitto: sudo systemctl restart mosquitto"
echo "2. Reset configuration: sudo cp mosquitto.conf /etc/mosquitto/mosquitto.conf"
echo "3. Check firewall: sudo ufw status"
echo "4. Verify no other service uses port 1883"
echo "5. Check system resources: top"
echo ""
echo "For detailed logs:"
echo "sudo journalctl -u mosquitto -n 50" 