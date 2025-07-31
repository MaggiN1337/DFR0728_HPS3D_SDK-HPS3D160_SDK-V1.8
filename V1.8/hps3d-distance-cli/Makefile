# Compiler selection based on target architecture
ARCH ?= $(shell uname -m)
ifeq ($(ARCH),aarch64)
    CC=gcc
else ifeq ($(ARCH),armv7l)
    CC=gcc
else
    # Cross-compilation for Raspberry Pi from x86_64
    CC=aarch64-linux-gnu-gcc
    CROSS_COMPILE=1
endif

CFLAGS=-I./lib -Wall -Wextra -DTARGET_ARCH=\"$(ARCH)\"
LDFLAGS=-L./lib -lHPS3D -lpthread -lm -lmosquitto

SRCS=src/main.c src/HPS3DUser_IF.c
OBJS=$(SRCS:.c=.o)
TARGET=hps3d_service

.PHONY: all clean install check-arch pi-deploy test-mqtt

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Architecture compatibility check
check-arch:
	@echo "Current system: $(shell uname -m)"
	@echo "Library architecture: $(shell file lib/libHPS3D.so | cut -d',' -f2)"
	@if [ "$(shell uname -m)" != "aarch64" ] && [ "$(shell uname -m)" != "armv7l" ]; then \
		echo "WARNING: Architecture mismatch detected!"; \
		echo "This system is $(shell uname -m) but library is for ARM."; \
		echo "Use 'make pi-deploy' to deploy to Raspberry Pi."; \
	fi

# Test MQTT functionality (mock mode for x86_64)
test-mqtt:
	@echo "Testing MQTT integration..."
	@echo "Checking MQTT broker availability..."
	@mosquitto_pub -h localhost -t hps3d/control -m "start" || echo "MQTT broker not available"
	@echo "Use 'sudo apt install mosquitto mosquitto-clients' to install MQTT broker"

# Raspberry Pi deployment
pi-deploy: $(TARGET)
	@echo "Deploying to Raspberry Pi..."
	@read -p "Enter Pi IP address: " PI_IP; \
	scp $(TARGET) lib/libHPS3D.so points.conf.example hps3d.service pi@$$PI_IP:/tmp/; \
	ssh pi@$$PI_IP 'sudo cp /tmp/$(TARGET) /usr/local/bin/ && \
	                sudo cp /tmp/libHPS3D.so /usr/local/lib/ && \
	                sudo ldconfig && \
	                sudo mkdir -p /etc/hps3d && \
	                sudo cp /tmp/points.conf.example /etc/hps3d/points.conf && \
	                sudo cp /tmp/hps3d.service /etc/systemd/system/ && \
	                sudo systemctl daemon-reload'
	@echo "Deployment complete. Enable with: ssh pi@$$PI_IP 'sudo systemctl enable --now hps3d'"

install: check-arch
	# Binaries und Bibliotheken
	install -d /usr/local/bin
	install -m 755 $(TARGET) /usr/local/bin/
	install -m 644 lib/libHPS3D.so /usr/local/lib/
	ldconfig

	# Konfigurationsverzeichnis
	install -d /etc/hps3d
	test -f /etc/hps3d/points.conf || install -m 644 points.conf.example /etc/hps3d/points.conf

	# Systemd Service
	install -m 644 hps3d.service /etc/systemd/system/
	systemctl daemon-reload
	@echo "Installation abgeschlossen. Starten Sie den Service mit:"
	@echo "systemctl start hps3d"

clean:
	rm -f $(OBJS) $(TARGET)