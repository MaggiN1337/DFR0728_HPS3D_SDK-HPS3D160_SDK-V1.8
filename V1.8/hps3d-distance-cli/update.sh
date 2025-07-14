#!/bin/bash

# Fehlgeschlagene Befehle stoppen das Skript
set -e

echo "Starte HPS3D-160 Service Update..."

# In das Verzeichnis des Skripts wechseln
cd "$(dirname "$0")"

# Git Repository aktualisieren
echo "Aktualisiere Code von Git..."
git pull

# Kompilieren
echo "Kompiliere Service..."
make clean
make

# Installation mit Root-Rechten
echo "Installiere Service..."
sudo make install

# Konfigurationsverzeichnis erstellen falls nicht vorhanden
echo "Prüfe Konfigurationsverzeichnis..."
sudo mkdir -p /etc/hps3d

# Konfigurationsdatei überprüfen und aktualisieren
CONFIG_FILE="/etc/hps3d/points.conf"
EXAMPLE_CONFIG="points.conf.example"
if [ -f "$CONFIG_FILE" ]; then
    # MD5 Checksummen vergleichen
    CONFIG_MD5=$(md5sum "$CONFIG_FILE" | cut -d' ' -f1)
    EXAMPLE_MD5=$(md5sum "$EXAMPLE_CONFIG" | cut -d' ' -f1)
    
    if [ "$CONFIG_MD5" != "$EXAMPLE_MD5" ]; then
        echo "WARNUNG: Konfigurationsdatei unterscheidet sich von der Beispielkonfiguration"
        echo "         Dies könnte auf neue Optionen oder Änderungen hinweisen"
        echo "         Ihre aktuelle Konfiguration wird gesichert"
        
        # Backup mit Zeitstempel erstellen
        BACKUP_FILE="${CONFIG_FILE}.$(date +%Y%m%d_%H%M%S).bak"
        sudo cp "$CONFIG_FILE" "$BACKUP_FILE"
        echo "         Backup erstellt: $BACKUP_FILE"
        
        # Neue Konfiguration kopieren
        echo "         Aktualisiere Konfiguration mit neuer Version..."
        sudo cp "$EXAMPLE_CONFIG" "$CONFIG_FILE"
        echo "         Bitte überprüfen Sie die neue Konfiguration und passen Sie sie bei Bedarf an"
        echo "         Ihre alte Konfiguration wurde als Backup gespeichert"
    else
        echo "Konfigurationsdatei ist aktuell"
    fi
else
    echo "Kopiere Beispielkonfiguration..."
    sudo cp "$EXAMPLE_CONFIG" "$CONFIG_FILE"
fi

# Service neustarten
echo "Starte Service neu..."
if systemctl is-active --quiet hps3d-service; then
    sudo systemctl restart hps3d-service
else
    sudo systemctl start hps3d-service
fi

# Status anzeigen
echo "Service Status:"
systemctl status hps3d-service

echo "Update abgeschlossen!" 