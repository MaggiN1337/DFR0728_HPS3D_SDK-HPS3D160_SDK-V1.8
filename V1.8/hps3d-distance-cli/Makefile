# Build system configuration
ARCH ?= $(shell uname -m)
CROSS_COMPILE ?= 0
TARGET_ARCH ?= $(ARCH)

# Compiler and build mode selection
ifeq ($(CROSS_COMPILE),1)
    # Cross-compilation mode
    CC=aarch64-linux-gnu-gcc
    TARGET_ARCH=aarch64
    BUILD_MODE=cross
else ifeq ($(ARCH),aarch64)
    # Native ARM64 compilation
    CC=gcc
    TARGET_ARCH=aarch64
    BUILD_MODE=native-arm64
else ifeq ($(ARCH),armv7l)
    # Native ARM32 compilation
    CC=gcc
    TARGET_ARCH=armv7l
    BUILD_MODE=native-arm32
else
    # Native x86_64 compilation (testing/development)
    CC=gcc
    TARGET_ARCH=x86_64
    BUILD_MODE=native-x86_64
    MOCK_MODE=1
endif

# Build flags based on mode
CFLAGS=-I./lib -Wall -Wextra -std=c11 -DTARGET_ARCH=\"$(TARGET_ARCH)\"

# Add mock mode for x86_64 builds
ifdef MOCK_MODE
    CFLAGS += -DMOCK_LIDAR=1 -DDEBUG_BUILD=1
    $(info Building in MOCK mode for development/testing on $(ARCH))
endif

# Library dependencies with fallback handling
LIBS_BASE=-lpthread -lm
LIBS_MOSQUITTO=-lmosquitto
LIBS_HPS3D=-lHPS3D

# Check for cross-compilation library path
ifeq ($(BUILD_MODE),cross)
    LDFLAGS=-L./lib $(LIBS_HPS3D) $(LIBS_BASE) $(LIBS_MOSQUITTO)
else ifeq ($(BUILD_MODE),native-arm64)
    LDFLAGS=-L./lib $(LIBS_HPS3D) $(LIBS_BASE) $(LIBS_MOSQUITTO)
else ifeq ($(BUILD_MODE),native-arm32)
    LDFLAGS=-L./lib $(LIBS_HPS3D) $(LIBS_BASE) $(LIBS_MOSQUITTO)
else
    # Native x86_64 - use system libraries where possible, skip mosquitto if not available
    LDFLAGS=$(LIBS_BASE)
    # Try to link mosquitto if available, otherwise skip for mock build
    ifneq ($(shell pkg-config --exists libmosquitto 2>/dev/null; echo $$?),0)
        $(warning MQTT functionality will be mocked - libmosquitto not found)
        CFLAGS += -DMOCK_MQTT=1
    else
        LDFLAGS += $(LIBS_MOSQUITTO)
    endif
    # HPS3D library will be mocked/stubbed
endif

# Source files - exclude HPS3DUser_IF.c in mock mode since it calls real library
ifdef MOCK_MODE
    SRCS=src/main.c
else
    SRCS=src/main.c src/HPS3DUser_IF.c
endif

OBJS=$(SRCS:.c=.o)
TARGET=hps3d_service

.PHONY: all clean install check-arch check-deps install-deps pi-deploy test-mqtt mock-build info help

all: check-deps $(TARGET)

# Display build information
info:
	@echo "=== HPS3D Build System Information ==="
	@echo "Host Architecture: $(ARCH)"
	@echo "Target Architecture: $(TARGET_ARCH)"
	@echo "Build Mode: $(BUILD_MODE)"
	@echo "Compiler: $(CC)"
	@echo "CFLAGS: $(CFLAGS)"
	@echo "LDFLAGS: $(LDFLAGS)"
	@echo ""
	@echo "Available targets:"
	@echo "  all          - Build with dependency checks"
	@echo "  mock-build   - Build with mock HPS3D library for testing"
	@echo "  check-deps   - Check build dependencies"
	@echo "  install-deps - Install missing dependencies"
	@echo "  check-arch   - Check architecture compatibility"
	@echo "  pi-deploy    - Deploy to Raspberry Pi"
	@echo "  test-mqtt    - Test MQTT functionality"
	@echo "  clean        - Remove build artifacts"
	@echo ""

# Help target
help: info

# Check build dependencies
check-deps:
	@echo "=== Checking Build Dependencies ==="
	@echo -n "Checking compiler $(CC)... "
	@which $(CC) > /dev/null 2>&1 && echo "✓ Found" || (echo "✗ Not found" && exit 1)
	
	@echo -n "Checking pthread library... "
	@$(CC) -lpthread -xc -E - < /dev/null > /dev/null 2>&1 && echo "✓ Available" || echo "⚠ Missing"
	
	@echo -n "Checking math library... "
	@$(CC) -lm -xc -E - < /dev/null > /dev/null 2>&1 && echo "✓ Available" || echo "⚠ Missing"
	
	@echo -n "Checking mosquitto headers... "
	@echo '#include <mosquitto.h>' | $(CC) -E - > /dev/null 2>&1 && echo "✓ Available" || echo "⚠ Missing (install libmosquitto-dev)"
	
	@echo -n "Checking mosquitto library... "
	@$(CC) -lmosquitto -xc /dev/null -o /dev/null > /dev/null 2>&1 && echo "✓ Available" || echo "⚠ Missing (install libmosquitto-dev)"
	
ifneq ($(BUILD_MODE),native-x86_64)
	@echo -n "Checking HPS3D library... "
	@test -f ./lib/libHPS3D.so && echo "✓ Found" || echo "✗ Missing lib/libHPS3D.so"
endif
	
ifeq ($(BUILD_MODE),cross)
	@echo -n "Checking cross-compiler... "
	@which aarch64-linux-gnu-gcc > /dev/null 2>&1 && echo "✓ Found" || (echo "✗ Install gcc-aarch64-linux-gnu" && exit 1)
endif
	@echo ""

# Install missing dependencies
install-deps:
	@echo "=== Installing Build Dependencies ==="
	
	@echo "Updating package list..."
	@sudo apt update
	
	@echo -n "Checking mosquitto development package... "
	@if ! dpkg -l | grep -q libmosquitto-dev; then \
		echo "Installing libmosquitto-dev..."; \
		sudo apt install -y libmosquitto-dev; \
	else \
		echo "✓ Already installed"; \
	fi
	
	@echo -n "Checking build essentials... "
	@if ! dpkg -l | grep -q build-essential; then \
		echo "Installing build-essential..."; \
		sudo apt install -y build-essential; \
	else \
		echo "✓ Already installed"; \
	fi

ifeq ($(CROSS_COMPILE),1)
	@echo -n "Checking cross-compiler... "
	@if ! dpkg -l | grep -q gcc-aarch64-linux-gnu; then \
		echo "Installing gcc-aarch64-linux-gnu..."; \
		sudo apt install -y gcc-aarch64-linux-gnu; \
	else \
		echo "✓ Already installed"; \
	fi
endif

	@echo ""
	@echo "✅ Dependencies installation complete!"
	@echo "Run 'make check-deps' to verify."
	@echo ""

# Mock build for development/testing on x86_64
mock-build: MOCK_MODE=1
mock-build: CFLAGS += -DMOCK_LIDAR=1 -DDEBUG_BUILD=1
mock-build: LDFLAGS = $(LIBS_BASE) $(LIBS_MOSQUITTO)
mock-build: check-deps $(TARGET)
	@echo "Mock build completed - HPS3D library calls will be stubbed"

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Architecture compatibility check
check-arch:
	@echo "=== Architecture Compatibility Check ==="
	@echo "Host system: $(ARCH)"
	@echo "Target system: $(TARGET_ARCH)"
	@echo "Build mode: $(BUILD_MODE)"
	
	@if [ -f ./lib/libHPS3D.so ]; then \
		echo "Library architecture: $$(file lib/libHPS3D.so | cut -d',' -f2 | xargs)"; \
	else \
		echo "HPS3D library: Not found (will use mock in development)"; \
	fi
	
	@echo ""
	@case "$(BUILD_MODE)" in \
		native-x86_64) \
			echo "ℹ️  Development/Testing Mode:"; \
			echo "   Building on x86_64 with mock HPS3D library"; \
			echo "   Use 'make pi-deploy' to deploy to Raspberry Pi"; \
			echo "   Use 'make CROSS_COMPILE=1' for cross-compilation"; \
			;; \
		native-arm64) \
			echo "✅ Native ARM64 Build:"; \
			echo "   Building natively on ARM64 (Raspberry Pi 4/5)"; \
			;; \
		native-arm32) \
			echo "✅ Native ARM32 Build:"; \
			echo "   Building natively on ARM32 (older Raspberry Pi)"; \
			;; \
		cross) \
			echo "🔄 Cross-compilation Mode:"; \
			echo "   Cross-compiling for ARM64 from x86_64"; \
			;; \
	esac
	@echo ""

# Test MQTT functionality (mock mode for x86_64)
test-mqtt:
	@echo "Testing MQTT integration..."
	@echo "Checking MQTT broker availability..."
	@mosquitto_pub -h localhost -t hps3d/control -m "start" || echo "MQTT broker not available"
	@echo "Use 'sudo apt install mosquitto mosquitto-clients' to install MQTT broker"

# Raspberry Pi deployment
pi-deploy: $(TARGET)
	@echo "=== Deploying to Raspberry Pi ==="
	@if [ "$(BUILD_MODE)" = "native-x86_64" ]; then \
		echo "⚠️  Warning: You built in mock mode on x86_64"; \
		echo "   This build will not work on Raspberry Pi!"; \
		echo "   Use 'make CROSS_COMPILE=1' to cross-compile first"; \
		echo ""; \
		read -p "Continue anyway? (y/N): " confirm; \
		if [ "$$confirm" != "y" ] && [ "$$confirm" != "Y" ]; then \
			echo "Deployment cancelled."; \
			exit 1; \
		fi; \
	fi
	
	@echo "Available Raspberry Pi deployment methods:"
	@echo "1. Interactive (enter IP manually)"
	@echo "2. Pre-configured (set PI_IP environment variable)"
	@echo ""
	
	@if [ -n "$(PI_IP)" ]; then \
		echo "Using pre-configured PI_IP: $(PI_IP)"; \
		PI_TARGET="$(PI_IP)"; \
	else \
		read -p "Enter Raspberry Pi IP address: " PI_TARGET; \
	fi; \
	\
	echo "Copying files to Raspberry Pi..."; \
	scp $(TARGET) lib/libHPS3D.so points.conf.example hps3d.service pi@$$PI_TARGET:/tmp/ || exit 1; \
	\
	echo "Installing on Raspberry Pi..."; \
	ssh pi@$$PI_TARGET 'sudo cp /tmp/$(TARGET) /usr/local/bin/ && \
	                     sudo cp /tmp/libHPS3D.so /usr/local/lib/ && \
	                     sudo ldconfig && \
	                     sudo mkdir -p /etc/hps3d && \
	                     if [ ! -f /etc/hps3d/points.conf ]; then \
	                         sudo cp /tmp/points.conf.example /etc/hps3d/points.conf; \
	                     fi && \
	                     sudo cp /tmp/hps3d.service /etc/systemd/system/ && \
	                     sudo systemctl daemon-reload' || exit 1; \
	\
	echo ""; \
	echo "✅ Deployment successful!"; \
	echo ""; \
	echo "To start the service:"; \
	echo "  ssh pi@$$PI_TARGET 'sudo systemctl enable --now hps3d'"; \
	echo ""; \
	echo "To check status:"; \
	echo "  ssh pi@$$PI_TARGET 'sudo systemctl status hps3d'"; \
	echo ""

# Install on current system (with architecture check)
install: check-arch check-deps
	@echo "=== Installing HPS3D Service ==="
	
	@if [ "$(BUILD_MODE)" = "native-x86_64" ]; then \
		echo "⚠️  Warning: Installing mock build on x86_64"; \
		echo "   This is for development/testing only!"; \
		echo ""; \
	fi
	
	# Create directories
	@echo "Creating directories..."
	@install -d /usr/local/bin
	@install -d /usr/local/lib
	@install -d /etc/hps3d
	
	# Install binary
	@echo "Installing binary..."
	@install -m 755 $(TARGET) /usr/local/bin/
	
	# Install library (only for ARM builds)
	@if [ -f lib/libHPS3D.so ] && [ "$(BUILD_MODE)" != "native-x86_64" ]; then \
		echo "Installing HPS3D library..."; \
		install -m 644 lib/libHPS3D.so /usr/local/lib/; \
		ldconfig; \
	else \
		echo "Skipping HPS3D library (mock build or not found)"; \
	fi
	
	# Install configuration
	@echo "Installing configuration..."
	@if [ ! -f /etc/hps3d/points.conf ]; then \
		install -m 644 points.conf.example /etc/hps3d/points.conf; \
		echo "Created default configuration at /etc/hps3d/points.conf"; \
	else \
		echo "Configuration already exists at /etc/hps3d/points.conf"; \
	fi
	
	# Install systemd service
	@echo "Installing systemd service..."
	@install -m 644 hps3d.service /etc/systemd/system/
	@systemctl daemon-reload
	
	@echo ""
	@echo "✅ Installation complete!"
	@echo ""
	@echo "To start the service:"
	@echo "  sudo systemctl enable --now hps3d"
	@echo ""
	@echo "To check status:"
	@echo "  sudo systemctl status hps3d"
	@echo ""

clean:
	rm -f $(OBJS) $(TARGET)