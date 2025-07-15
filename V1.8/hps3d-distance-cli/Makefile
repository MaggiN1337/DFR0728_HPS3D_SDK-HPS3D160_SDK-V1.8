CC=gcc
CFLAGS=-I./lib -Wall -Wextra -Werror -pedantic -fPIC -D_GNU_SOURCE
LDFLAGS=-L./lib -lHPS3D -lpthread -lm -lmosquitto

SRCS=src/main.c src/HPS3DUser_IF.c
OBJS=$(SRCS:.c=.o)
TARGET=hps3d_service

.PHONY: all clean install check-deps

all: check-deps $(TARGET)

check-deps:
	@echo "Checking dependencies..."
	@which pkg-config > /dev/null || (echo "pkg-config not found. Please install pkg-config" && exit 1)
	@pkg-config --exists libmosquitto || (echo "libmosquitto not found. Please install libmosquitto-dev" && exit 1)
	@ldconfig -p | grep libHPS3D > /dev/null || (echo "libHPS3D not found. Please install HPS3D SDK first" && exit 1)
	@echo "All dependencies found."

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

install: check-deps
	# Create directories
	install -d /usr/local/bin
	install -d /usr/local/lib
	install -d /etc/hps3d
	install -d /var/log/hps3d

	# Install binaries and libraries
	install -m 755 $(TARGET) /usr/local/bin/
	install -m 644 lib/libHPS3D.so /usr/local/lib/
	ldconfig

	# Install configuration
	test -f /etc/hps3d/points.conf || install -m 644 points.conf.example /etc/hps3d/points.conf

	# Install systemd service
	install -m 644 hps3d-service.service /etc/systemd/system/
	systemctl daemon-reload

	@echo "Installation complete. To start the service:"
	@echo "1. Edit configuration if needed: sudo nano /etc/hps3d/points.conf"
	@echo "2. Start service: sudo systemctl start hps3d-service"
	@echo "3. Enable at boot: sudo systemctl enable hps3d-service"

clean:
	rm -f $(OBJS) $(TARGET)