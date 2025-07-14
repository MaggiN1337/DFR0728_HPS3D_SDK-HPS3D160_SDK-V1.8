/*
 * HPS3D-160 LIDAR 4-Punkt Messservice für NodeRed Integration
 * 
 * Kompilierung: gcc -o hps3d_service hps3d_service.c -lhps3d -lpthread -lm
 * 
 * Funktionen:
 * - Kontinuierliche Messung von 4 definierten Punkten
 * - JSON Output für NodeRed
 * - Konfigurierbare Messpunkte
 * - Fehlerbehandlung und Reconnect
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <sys/stat.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h> // Für va_list, va_start, va_end

// HPS3D SDK Headers
#include "HPS3DUser_IF.h"

// Konfiguration
#define MAX_POINTS 4
#define AREA_SIZE 5        // 5x5 Pixel Messbereich
#define AREA_OFFSET 2      // (5-1)/2 für zentrierten Bereich
#define MEASURE_INTERVAL_MS 1000  // 10 Hz Messrate
#define OUTPUT_INTERVAL_MS 2000  // 1 Hz Output für NodeRed
#define CONFIG_FILE "/etc/hps3d/points.conf"
#define PID_FILE "/var/run/hps3d_service.pid"
#define DEBUG_FILE "hps3d_debug.log"  // Debug-Ausgaben in diese Datei
#define USB_PORT "/dev/ttyACM0"  // Standard USB Port für HPS3D-160

// Globale Variablen
static int running = 1;
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_handle = -1;
static HPS3D_MeasureData_t g_measureData;
static bool reconnect_needed = false;
static FILE* debug_file = NULL;  // File handle für Debug-Ausgaben

// Debug-Ausgabe Funktion
void debug_print(const char* format, ...) {
    if (debug_file) {
        va_list args;
        va_start(args, format);
        vfprintf(debug_file, format, args);
        fflush(debug_file);  // Sofort in Datei schreiben
        va_end(args);
    }
}

// Messpunkt Definition
typedef struct {
    int x, y;           // Pixel-Koordinaten im 160x60 Array (Zentrum des 5x5 Bereichs)
    float distance;     // Gemessene Durchschnittsdistanz in mm
    float min_distance; // Minimale Distanz im Messbereich
    float max_distance; // Maximale Distanz im Messbereich
    int valid_pixels;   // Anzahl gültiger Pixel im Messbereich
    int valid;          // Messung gültig (mind. 50% der Pixel gültig)
    time_t timestamp;   // Zeitstempel der letzten Messung
    char name[32];      // Name des Messpunkts
} MeasurePoint;

// Globale Messpunkte
static MeasurePoint points[MAX_POINTS] = {
    {40, 30, 0.0, 0.0, 0.0, 0, 0, 0, "point_1"},   // Links-Oben
    {120, 30, 0.0, 0.0, 0.0, 0, 0, 0, "point_2"},  // Rechts-Oben
    {40, 45, 0.0, 0.0, 0.0, 0, 0, 0, "point_3"},   // Links-Unten
    {120, 45, 0.0, 0.0, 0.0, 0, 0, 0, "point_4"}   // Rechts-Unten
};

// Event Callback für HPS3D
static void EventCallBackFunc(int handle, int eventType, uint8_t *data, int dataLen, void *userPara) {
    switch ((HPS3D_EventType_t)eventType) {
        case HPS3D_DISCONNECT_EVEN:
            printf("WARNUNG: HPS3D-160 getrennt, versuche Wiederverbindung...\n");
            reconnect_needed = true;
            break;
        case HPS3D_SYS_EXCEPTION_EVEN:
            printf("WARNUNG: System Exception: %s\n", data);
            break;
        default:
            break;
    }
}

// Signal Handler
void signal_handler(int sig) {
    printf("Signal %d empfangen, beende Service...\n", sig);
    running = 0;
}

// Konfigurationsdatei laden
int load_config() {
    FILE *fp = fopen(CONFIG_FILE, "r");
    if (!fp) {
        printf("Verwende Standard-Konfiguration\n");
        return 0;
    }
    
    char line[256];
    int point_idx = 0;
    
    while (fgets(line, sizeof(line), fp) && point_idx < MAX_POINTS) {
        if (line[0] == '#' || line[0] == '\n') continue;
        
        int x, y;
        char name[32];
        if (sscanf(line, "%d,%d,%s", &x, &y, name) == 3) {
            // Prüfe ob der 5x5 Bereich innerhalb des Sensors liegt
            if (x >= AREA_OFFSET && x < (160 - AREA_OFFSET) && 
                y >= AREA_OFFSET && y < (60 - AREA_OFFSET)) {
                points[point_idx].x = x;
                points[point_idx].y = y;
                strncpy(points[point_idx].name, name, sizeof(points[point_idx].name)-1);
                point_idx++;
            } else {
                printf("WARNUNG: Koordinaten (%d,%d) ungültig - 5x5 Bereich außerhalb des Sensors\n", x, y);
            }
        }
    }
    
    fclose(fp);
    printf("Konfiguration geladen: %d Punkte\n", point_idx);
    return point_idx;
}

// LIDAR initialisieren
int init_lidar() {
    HPS3D_StatusTypeDef ret;

    // Messdatenstruktur initialisieren
    ret = HPS3D_MeasureDataInit(&g_measureData);
    if (ret != HPS3D_RET_OK) {
        printf("FEHLER: Messdatenstruktur konnte nicht initialisiert werden\n");
        return -1;
    }

    // Callback registrieren
    ret = HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);
    if (ret != HPS3D_RET_OK) {
        printf("FEHLER: Callback-Registrierung fehlgeschlagen\n");
        return -1;
    }

    // USB Verbindung aufbauen
    ret = HPS3D_USBConnectDevice(USB_PORT, &g_handle);
    if (ret != HPS3D_RET_OK) {
        printf("FEHLER: Verbindung zu HPS3D-160 fehlgeschlagen (%d)\n", ret);
        return -1;
    }

    printf("HPS3D-160 verbunden: %s\n", HPS3D_GetDeviceVersion(g_handle));

    // Weniger aggressive Filtereinstellungen
    HPS3D_SetDistanceFilterConf(g_handle, false, 0.1f);  // Distanzfilter deaktiviert
    HPS3D_SetSmoothFilterConf(g_handle, HPS3D_SMOOTH_FILTER_DISABLE, 0);  // Glättungsfilter deaktiviert
    HPS3D_SetEdgeFilterEnable(g_handle, false);  // Kantenfilter deaktiviert
    
    // Optische Wegkorrektur aktivieren für genauere Messungen
    HPS3D_SetOpticalPathCalibration(g_handle, true);

    // Messung starten
    ret = HPS3D_StartCapture(g_handle);
    if (ret != HPS3D_RET_OK) {
        printf("FEHLER: Messung konnte nicht gestartet werden\n");
        return -1;
    }

    printf("LIDAR initialisiert und gestartet\n");
    return 0;
}

// Einzelne Messung durchführen
int measure_points() {
    if (!HPS3D_IsConnect(g_handle)) {
        return -1;
    }

    HPS3D_EventType_t event_type;
    HPS3D_StatusTypeDef ret = HPS3D_SingleCapture(g_handle, &event_type, &g_measureData);
    
    if (ret != HPS3D_RET_OK) {
        printf("WARNUNG: Messung fehlgeschlagen (%d)\n", ret);
        return -1;
    }

    pthread_mutex_lock(&data_mutex);
    
    // Alle 4 Punkte messen
    if (event_type == HPS3D_FULL_DEPTH_EVEN) {
        for (int i = 0; i < MAX_POINTS; i++) {
            int center_x = points[i].x;
            int center_y = points[i].y;
            float sum_distance = 0;
            int valid_count = 0;
            float min_distance = 65000;
            float max_distance = 0;
            
            // Debug: Array für Rohdaten
            uint16_t raw_values[AREA_SIZE * AREA_SIZE];
            int raw_idx = 0;
            
            // 5x5 Bereich um den Punkt messen
            for (int dy = -AREA_OFFSET; dy <= AREA_OFFSET; dy++) {
                for (int dx = -AREA_OFFSET; dx <= AREA_OFFSET; dx++) {
                    int x = center_x + dx;
                    int y = center_y + dy;
                    int pixel_index = y * 160 + x;
                    
                    uint16_t distance_raw = g_measureData.full_depth_data.distance[pixel_index];
                    raw_values[raw_idx++] = distance_raw;
                    
                    // Erweiterte Gültigkeitsprüfung
                    if (distance_raw > 0 && distance_raw < 65000 && 
                        distance_raw != HPS3D_LOW_AMPLITUDE && 
                        distance_raw != HPS3D_SATURATION && 
                        distance_raw != HPS3D_ADC_OVERFLOW && 
                        distance_raw != HPS3D_INVALID_DATA) {
                        
                        sum_distance += distance_raw;
                        valid_count++;
                        
                        if (distance_raw < min_distance) min_distance = distance_raw;
                        if (distance_raw > max_distance) max_distance = distance_raw;
                    }
                }
            }
            
            // Debug: Ausgabe der Rohdaten für jeden Punkt
            debug_print("\n----------------------------------------\n");
            debug_print("DEBUG Point %s Raw Values (Timestamp: %ld):\n", 
                       points[i].name, time(NULL));
            for (int y = 0; y < AREA_SIZE; y++) {
                debug_print("  ");
                for (int x = 0; x < AREA_SIZE; x++) {
                    debug_print("%5d ", raw_values[y * AREA_SIZE + x]);
                }
                debug_print("\n");
            }
            debug_print("Valid pixels: %d/%d\n", valid_count, AREA_SIZE * AREA_SIZE);
            debug_print("Min distance: %.1f mm\n", min_distance);
            debug_print("Max distance: %.1f mm\n", max_distance);
            if (valid_count > 0) {
                debug_print("Average distance: %.1f mm\n", sum_distance / valid_count);
            }
            debug_print("----------------------------------------\n");
            
            // Messung ist gültig wenn mindestens 25% der Pixel gültig sind (weniger streng)
            int required_valid_pixels = (AREA_SIZE * AREA_SIZE) / 4;
            if (valid_count >= required_valid_pixels) {
                points[i].distance = sum_distance / valid_count;
                points[i].min_distance = min_distance;
                points[i].max_distance = max_distance;
                points[i].valid_pixels = valid_count;
                points[i].valid = 1;
                points[i].timestamp = time(NULL);
            } else {
                points[i].valid = 0;
                points[i].valid_pixels = valid_count;
            }
        }
    }
    
    pthread_mutex_unlock(&data_mutex);
    return 0;
}

// JSON Output für NodeRed
void output_json() {
    pthread_mutex_lock(&data_mutex);
    
    printf("{\n");
    printf("  \"timestamp\": %ld,\n", time(NULL));
    printf("  \"measurements\": {\n");
    
    for (int i = 0; i < MAX_POINTS; i++) {
        printf("    \"%s\": {\n", points[i].name);
        printf("      \"distance_mm\": %.1f,\n", points[i].distance);
        printf("      \"distance_m\": %.3f,\n", points[i].distance / 1000.0);
        printf("      \"min_distance_mm\": %.1f,\n", points[i].min_distance);
        printf("      \"max_distance_mm\": %.1f,\n", points[i].max_distance);
        printf("      \"valid_pixels\": %d,\n", points[i].valid_pixels);
        printf("      \"valid\": %s,\n", points[i].valid ? "true" : "false");
        printf("      \"age_seconds\": %ld,\n", time(NULL) - points[i].timestamp);
        printf("      \"coordinates\": {\"x\": %d, \"y\": %d}\n", points[i].x, points[i].y);
        printf("    }%s\n", (i < MAX_POINTS-1) ? "," : "");
    }
    
    printf("  }\n");
    printf("}\n");
    fflush(stdout);
    
    pthread_mutex_unlock(&data_mutex);
}

// Mess-Thread
void* measure_thread(void* arg) {
    while (running) {
        if (reconnect_needed) {
            HPS3D_CloseDevice(g_handle);
            sleep(1);  // Kurz warten vor Reconnect
            if (init_lidar() == 0) {
                reconnect_needed = false;
                printf("HPS3D-160 erfolgreich neu verbunden\n");
            }
            continue;
        }

        if (measure_points() != 0) {
            usleep(500000); // 500ms Pause bei Fehler
        } else {
            usleep(MEASURE_INTERVAL_MS * 1000);
        }
    }
    return NULL;
}

// Output-Thread
void* output_thread(void* arg) {
    while (running) {
        output_json();
        usleep(OUTPUT_INTERVAL_MS * 1000);
    }
    return NULL;
}

// PID-Datei erstellen
int create_pid_file() {
    FILE *fp = fopen(PID_FILE, "w");
    if (!fp) {
        printf("WARNUNG: PID-Datei konnte nicht erstellt werden\n");
        return -1;
    }
    
    fprintf(fp, "%d\n", getpid());
    fclose(fp);
    return 0;
}

// Cleanup
void cleanup() {
    debug_print("Cleanup...\n");
    
    if (g_handle >= 0) {
        HPS3D_StopCapture(g_handle);
        HPS3D_CloseDevice(g_handle);
    }
    
    HPS3D_MeasureDataFree(&g_measureData);
    HPS3D_UnregisterEventCallback();
    
    unlink(PID_FILE);
    
    debug_print("Service beendet\n");
}

// Hauptprogramm
int main(int argc, char *argv[]) {
    printf("HPS3D-160 LIDAR Service gestartet\n");
    
    // Debug-Datei öffnen
    debug_file = fopen(DEBUG_FILE, "w");
    if (!debug_file) {
        printf("WARNUNG: Debug-Datei konnte nicht geöffnet werden\n");
    }
    
    // Signal Handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Daemon Mode
    if (argc > 1 && strcmp(argv[1], "-d") == 0) {
        daemon(0, 0);
    }
    
    // PID-Datei erstellen
    create_pid_file();
    
    // Konfiguration laden
    load_config();
    
    // LIDAR initialisieren
    if (init_lidar() != 0) {
        cleanup();
        return 1;
    }
    
    // Threads starten
    pthread_t measure_tid, output_tid;
    
    if (pthread_create(&measure_tid, NULL, measure_thread, NULL) != 0) {
        printf("FEHLER: Mess-Thread konnte nicht erstellt werden\n");
        cleanup();
        return 1;
    }
    
    if (pthread_create(&output_tid, NULL, output_thread, NULL) != 0) {
        printf("FEHLER: Output-Thread konnte nicht erstellt werden\n");
        cleanup();
        return 1;
    }
    
    // Auf Threads warten
    pthread_join(measure_tid, NULL);
    pthread_join(output_tid, NULL);
    
    cleanup();
    
    // Debug-Datei schließen
    if (debug_file) {
        fclose(debug_file);
    }
    
    return 0;
}