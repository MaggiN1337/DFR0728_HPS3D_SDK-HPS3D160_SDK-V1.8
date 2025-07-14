CC = gcc
CFLAGS = -Wall -I./src -I/usr/local/include
LDFLAGS = -L./lib -lHPS3D -lpthread -lm

TARGET = hps3d_service
SRCS = src/main.c
OBJS = $(SRCS:.c=.o)

.PHONY: all clean install uninstall

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

install: $(TARGET)
	# Binärdatei installieren
	install -m 755 $(TARGET) /usr/local/bin/
	# Konfigurationsverzeichnis erstellen
	mkdir -p /etc/hps3d
	# Beispielkonfiguration kopieren, wenn noch keine existiert
	[ -f /etc/hps3d/points.conf ] || install -m 644 points.conf.example /etc/hps3d/points.conf
	# Systemd Service installieren
	install -m 644 hps3d-service.service /etc/systemd/system/
	# Bibliothek installieren
	install -m 644 lib/libHPS3D.so /usr/local/lib/
	# Header installieren
	mkdir -p /usr/local/include/hps3d
	install -m 644 src/HPS3D*.h /usr/local/include/hps3d/
	# Bibliotheken aktualisieren
	ldconfig
	# Systemd Service aktivieren
	systemctl enable hps3d-service
	systemctl daemon-reload

uninstall:
	systemctl stop hps3d-service || true
	systemctl disable hps3d-service || true
	rm -f /etc/systemd/system/hps3d-service.service
	rm -f /usr/local/bin/$(TARGET)
	rm -f /usr/local/lib/libHPS3D.so
	rm -rf /usr/local/include/hps3d
	systemctl daemon-reload

clean:
	rm -f $(TARGET) $(OBJS)