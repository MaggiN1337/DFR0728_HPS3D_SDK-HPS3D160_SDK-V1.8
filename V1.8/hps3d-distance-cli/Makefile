CC=gcc
CFLAGS=-I./lib -Wall -Wextra
LDFLAGS=-L./lib -lHPS3D -lpthread -lm -lmosquitto

SRCS=src/main.c src/HPS3DUser_IF.c
OBJS=$(SRCS:.c=.o)
TARGET=hps3d_service

.PHONY: all clean install

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

install:
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