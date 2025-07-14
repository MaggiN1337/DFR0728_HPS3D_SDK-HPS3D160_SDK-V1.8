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

# Beispielkonfiguration kopieren falls nicht vorhanden
if [ ! -f /etc/hps3d/points.conf ]; then
    echo "Kopiere Beispielkonfiguration..."
    sudo cp points.conf.example /etc/hps3d/points.conf
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