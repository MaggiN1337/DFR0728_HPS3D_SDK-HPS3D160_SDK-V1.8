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
#include <stdarg.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <mosquitto.h>

// HPS3D SDK Headers
#include "HPS3DUser_IF.h"

// Forward declarations
static int init_lidar(void);
static int init_mqtt(void);
static int init_http_server(void);
static int measure_points(void);
static char* create_json_output(void);
static void cleanup(void);
static int load_config(void);
static int create_pid_file(void);

void* measure_thread(void* arg);
void* output_thread(void* arg);
void* http_server_thread(void* arg);
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message);

// Konfiguration
#define MAX_POINTS 4
#define AREA_SIZE 5        // 5x5 Pixel Messbereich
#define AREA_OFFSET 2      // (5-1)/2 für zentrierten Bereich
#define DEFAULT_MIN_VALID_PIXELS 6  // Standard: 25% der Pixel (6 von 25)
#define MEASURE_INTERVAL_MS 500  // 50 Hz Messrate
#define OUTPUT_INTERVAL_MS 1000  // 1 Hz Output für NodeRed
#define CONFIG_FILE "/etc/hps3d/points.conf"
#define PID_FILE "/var/run/hps3d_service.pid"
#define DEFAULT_DEBUG_FILE "debug_hps3d.log"
#define USB_PORT "/dev/ttyACM0"

// HTTP Server Konfiguration
#define HTTP_PORT 8080
#define HTTP_RESPONSE "HTTP/1.1 200 OK\r\nContent-Length: %zu\r\nContent-Type: application/json\r\n\r\n%s"

// MQTT Konfiguration
#define MQTT_HOST "localhost"
#define MQTT_PORT 1883
#define MQTT_TOPIC "hps3d/measurements"
#define MQTT_CONTROL_TOPIC "hps3d/control"
#define MQTT_POINTCLOUD_TOPIC "hps3d/pointcloud"  // Neues Topic für Punktwolke
#define MQTT_RECONNECT_DELAY 5  // Sekunden zwischen Reconnect-Versuchen

// Globale Variablen
static volatile int running = 1;
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_handle = -1;
static HPS3D_MeasureData_t g_measureData;
static volatile bool reconnect_needed = false;
static FILE* debug_file = NULL;
static int debug_enabled = 0;
static int min_valid_pixels = DEFAULT_MIN_VALID_PIXELS;
static struct mosquitto *mosq = NULL;
static int http_socket = -1;
static volatile int measurement_active = 0;  // Start deaktiviert
static volatile int mqtt_connected = 0;
static volatile int pointcloud_requested = 0;  // Neue Variable für Punktwolken-Request

// Debug-Ausgabe Funktion
void debug_print(const char* format, ...) {
    static FILE* debug_file = NULL;
    static int first_call = 1;
    
    // Beim ersten Aufruf Debug-File öffnen
    if (first_call) {
        debug_file = fopen("debug_hps3d.log", "w");
        if (!debug_file) {
            fprintf(stderr, "FEHLER: Debug-Datei konnte nicht geöffnet werden\n");
            return;
        }
        first_call = 0;
    }

    va_list args;
    va_start(args, format);
    
    // In Debug-Datei schreiben
    time_t now = time(NULL);
    char timestamp[26];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));
    fprintf(debug_file, "[%s] ", timestamp);
    vfprintf(debug_file, format, args);
    fflush(debug_file);
    
    va_end(args);
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
    (void)handle;    // Ungenutzte Parameter markieren
    (void)dataLen;
    (void)userPara;
    
    HPS3D_EventType_t event = (HPS3D_EventType_t)eventType;
    switch (event) {
        case HPS3D_DISCONNECT_EVEN:
            printf("WARNUNG: HPS3D-160 getrennt, versuche Wiederverbindung...\n");
            reconnect_needed = true;
            break;
        case HPS3D_SYS_EXCEPTION_EVEN:
            if (data) {
                printf("WARNUNG: System Exception: %s\n", (char*)data);
            } else {
                printf("WARNUNG: System Exception (keine Details verfügbar)\n");
            }
            break;
        default:
            printf("WARNUNG: Unbekanntes Event: %d\n", eventType);
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
    char debug_file_path[256] = DEFAULT_DEBUG_FILE;
    
    while (fgets(line, sizeof(line), fp)) {
        // Kommentare und leere Zeilen überspringen
        if (line[0] == '#' || line[0] == '\n') continue;
        
        // Debug-Einstellungen verarbeiten
        if (strncmp(line, "debug=", 6) == 0) {
            debug_enabled = atoi(line + 6);
            continue;
        }
        
        if (strncmp(line, "debug_file=", 11) == 0) {
            char* path = line + 11;
            // Newline am Ende entfernen
            char* nl = strchr(path, '\n');
            if (nl) *nl = '\0';
            // Wenn ein Pfad angegeben ist, diesen verwenden
            if (strlen(path) > 0) {
                strncpy(debug_file_path, path, sizeof(debug_file_path)-1);
            }
            continue;
        }

        // Minimale gültige Pixel-Einstellung verarbeiten
        if (strncmp(line, "min_valid_pixels=", 17) == 0) {
            min_valid_pixels = atoi(line + 17);
            continue;
        }
        
        // Messpunkte verarbeiten
        int x, y;
        char name[32];
        if (point_idx < MAX_POINTS && sscanf(line, "%d,%d,%s", &x, &y, name) == 3) {
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
    
    // Debug-Datei öffnen wenn aktiviert
    if (debug_enabled) {
        debug_file = fopen(debug_file_path, "w");
        if (!debug_file) {
            printf("WARNUNG: Debug-Datei %s konnte nicht geöffnet werden\n", debug_file_path);
        } else {
            printf("Debug-Ausgaben werden in %s geschrieben\n", debug_file_path);
        }
    }
    
    printf("Konfiguration geladen: %d Punkte, Debug %s, min_valid_pixels %d\n", 
           point_idx, debug_enabled ? "aktiviert" : "deaktiviert", min_valid_pixels);
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
            
            // Messung ist gültig wenn mindestens die konfigurierte Anzahl Pixel gültig sind
            if (valid_count >= min_valid_pixels) {
                points[i].distance = sum_distance / valid_count;
                points[i].min_distance = min_distance;
                points[i].max_distance = max_distance;
                points[i].valid_pixels = valid_count;
                points[i].valid = 1;
                points[i].timestamp = time(NULL);
                
                debug_print("Messung gültig: %d/%d Pixel (min: %d)\n", 
                          valid_count, AREA_SIZE * AREA_SIZE, min_valid_pixels);
            } else {
                points[i].valid = 0;
                points[i].valid_pixels = valid_count;
                
                debug_print("Messung ungültig: %d/%d Pixel (min: %d)\n", 
                          valid_count, AREA_SIZE * AREA_SIZE, min_valid_pixels);
            }
        }
    }
    
    pthread_mutex_unlock(&data_mutex);
    return 0;
}

// JSON String für Output erstellen
char* create_json_output() {
    static char json_buffer[4096];
    pthread_mutex_lock(&data_mutex);
    
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"timestamp\": %ld,"
        "\"active\": %s,"
        "\"measurements\": {",
        time(NULL),
        measurement_active ? "true" : "false"
    );
    
    for (int i = 0; i < MAX_POINTS; i++) {
        char point_json[512];
        snprintf(point_json, sizeof(point_json),
            "\"%s\": {"
            "\"distance_mm\": %.1f,"
            "\"distance_m\": %.3f,"
            "\"min_distance_mm\": %.1f,"
            "\"max_distance_mm\": %.1f,"
            "\"valid_pixels\": %d,"
            "\"valid\": %s,"
            "\"age_seconds\": %ld,"
            "\"coordinates\": {\"x\": %d, \"y\": %d}"
            "}%s",
            points[i].name,
            points[i].distance,
            points[i].distance / 1000.0,
            points[i].min_distance,
            points[i].max_distance,
            points[i].valid_pixels,
            points[i].valid ? "true" : "false",
            time(NULL) - points[i].timestamp,
            points[i].x, points[i].y,
            (i < MAX_POINTS-1) ? "," : ""
        );
        strcat(json_buffer, point_json);
    }
    
    strcat(json_buffer, "}}");
    pthread_mutex_unlock(&data_mutex);
    
    return json_buffer;
}

// JSON String für Punktwolke erstellen
char* create_pointcloud_json() {
    static char json_buffer[160*60*50];  // Mehr Speicher für JSON
    int buffer_pos = 0;
    int remaining = sizeof(json_buffer);
    
    pthread_mutex_lock(&data_mutex);
    
    debug_print("Erstelle Punktwolken-JSON...\n");
    
    // Prüfe Messdaten
    if (!g_measureData.full_depth_data.distance) {
        debug_print("FEHLER: Keine Messdaten verfügbar\n");
        pthread_mutex_unlock(&data_mutex);
        return NULL;
    }
    
    // Header schreiben
    buffer_pos += snprintf(json_buffer + buffer_pos, remaining,
        "{\"timestamp\":%ld,\"width\":%d,\"height\":%d,\"data\":[",
        time(NULL), 160, 60);
    remaining = sizeof(json_buffer) - buffer_pos;
    
    int valid_points = 0;
    // Alle Pixel durchgehen
    for (int y = 0; y < 60 && remaining > 0; y++) {
        for (int x = 0; x < 160 && remaining > 0; x++) {
            int pixel_index = y * 160 + x;
            uint16_t distance = g_measureData.full_depth_data.distance[pixel_index];
            
            // Nur gültige Werte senden
            if (distance > 0 && distance < 65000 && 
                distance != HPS3D_LOW_AMPLITUDE && 
                distance != HPS3D_SATURATION && 
                distance != HPS3D_ADC_OVERFLOW && 
                distance != HPS3D_INVALID_DATA) {
                
                // Komma hinzufügen wenn nicht erster Punkt
                if (valid_points > 0) {
                    buffer_pos += snprintf(json_buffer + buffer_pos, remaining, ",");
                    remaining = sizeof(json_buffer) - buffer_pos;
                }
                
                // Punkt hinzufügen
                buffer_pos += snprintf(json_buffer + buffer_pos, remaining,
                    "{\"x\":%d,\"y\":%d,\"d\":%d}",
                    x, y, distance);
                remaining = sizeof(json_buffer) - buffer_pos;
                valid_points++;
            }
        }
    }
    
    // JSON abschließen
    buffer_pos += snprintf(json_buffer + buffer_pos, remaining, "]}");
    
    debug_print("Punktwolken-JSON erstellt mit %d gültigen Punkten\n", valid_points);
    
    pthread_mutex_unlock(&data_mutex);
    return json_buffer;
}

// Output-Thread
void* output_thread(void* arg) {
    (void)arg;  // Ungenutzte Parameter markieren
    
    while (running) {
        // Normale Messpunkte ausgeben wenn aktiv
        if (measurement_active) {
            char* json_output = create_json_output();
            
            // Ausgabe auf stdout
            printf("%s\n", json_output);
            fflush(stdout);
            
            // MQTT Publish wenn verbunden
            if (mosq && mqtt_connected) {
                int rc = mosquitto_publish(mosq, NULL, MQTT_TOPIC, strlen(json_output), json_output, 0, false);
                if (rc != MOSQ_ERR_SUCCESS) {
                    debug_print("MQTT Publish fehlgeschlagen: %d\n", rc);
                }
            }
        }
        
        // Punktwolke bei Anforderung senden
        if (pointcloud_requested) {
            debug_print("Pointcloud requested, capturing data...\n");
            
            // Stelle sicher, dass wir aktuelle Daten haben
            if (measure_points() == 0) {
                char* cloud_json = create_pointcloud_json();
                if (mosq && mqtt_connected) {
                    debug_print("Publishing pointcloud data...\n");
                    int rc = mosquitto_publish(mosq, NULL, MQTT_POINTCLOUD_TOPIC, 
                                    strlen(cloud_json), cloud_json, 0, false);
                    if (rc != MOSQ_ERR_SUCCESS) {
                        debug_print("Failed to publish pointcloud: %d\n", rc);
                    } else {
                        debug_print("Pointcloud published successfully\n");
                    }
                }
            } else {
                debug_print("Failed to capture pointcloud data\n");
            }
            pointcloud_requested = 0;  // Request zurücksetzen
        }
        
        usleep(OUTPUT_INTERVAL_MS * 1000);
    }
    return NULL;
}

// MQTT Callback für Control Messages
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    (void)mosq;
    (void)userdata;
    
    if (!message || !message->payload) {
        debug_print("MQTT: Ungültige Nachricht empfangen\n");
        return;
    }
    
    debug_print("MQTT Nachricht empfangen: Topic=%s, Payload=%.*s\n", 
                message->topic, (int)message->payloadlen, (char*)message->payload);
    
    if (strcmp(message->topic, MQTT_CONTROL_TOPIC) == 0) {
        if (strncmp(message->payload, "start", message->payloadlen) == 0) {
            measurement_active = 1;
            debug_print("Messung aktiviert via MQTT\n");
        } else if (strncmp(message->payload, "stop", message->payloadlen) == 0) {
            measurement_active = 0;
            debug_print("Messung deaktiviert via MQTT\n");
        } else if (strncmp(message->payload, "get_pointcloud", message->payloadlen) == 0) {
            debug_print("Punktwolke angefordert via MQTT\n");
            
            // Prüfe ob LIDAR verbunden
            if (!HPS3D_IsConnect(g_handle)) {
                debug_print("FEHLER: LIDAR nicht verbunden\n");
                return;
            }
            
            // Prüfe ob Messdaten initialisiert
            if (!g_measureData.full_depth_data.distance) {
                debug_print("FEHLER: Messdaten nicht initialisiert\n");
                return;
            }
            
            // Führe Messung durch
            if (measure_points() != 0) {
                debug_print("FEHLER: Punktwolken-Messung fehlgeschlagen\n");
                return;
            }
            
            // Erstelle JSON
            char* cloud_json = create_pointcloud_json();
            if (!cloud_json) {
                debug_print("FEHLER: JSON-Erstellung fehlgeschlagen\n");
                return;
            }
            
            // Sende Daten
            if (mosq && mqtt_connected) {
                int rc = mosquitto_publish(mosq, NULL, MQTT_POINTCLOUD_TOPIC, 
                                strlen(cloud_json), cloud_json, 0, false);
                if (rc != MOSQ_ERR_SUCCESS) {
                    debug_print("FEHLER: MQTT Publish fehlgeschlagen (%d)\n", rc);
                } else {
                    debug_print("Punktwolke erfolgreich gesendet\n");
                }
            } else {
                debug_print("FEHLER: MQTT nicht verbunden\n");
            }
        }
    }
}

// MQTT Verbindungs-Callback
void mqtt_connect_callback(struct mosquitto *mosq, void *userdata, int result) {
    (void)mosq;
    (void)userdata;
    
    if (!result) {
        mqtt_connected = 1;
        debug_print("MQTT: Verbindung hergestellt\n");
        
        // Resubscribe nach Reconnect
        if (mosquitto_subscribe(mosq, NULL, MQTT_CONTROL_TOPIC, 0) != MOSQ_ERR_SUCCESS) {
            debug_print("MQTT: Subscribe nach Reconnect fehlgeschlagen\n");
        }
        
        // Status nach Verbindung senden
        char status[100];
        snprintf(status, sizeof(status), "{\"status\": \"connected\", \"active\": %s}", 
                measurement_active ? "true" : "false");
        mosquitto_publish(mosq, NULL, MQTT_TOPIC "/status", strlen(status), status, 0, false);
    } else {
        mqtt_connected = 0;
        debug_print("MQTT: Verbindung fehlgeschlagen (%d)\n", result);
    }
}

// MQTT Disconnect-Callback
void mqtt_disconnect_callback(struct mosquitto *mosq, void *userdata, int rc) {
    (void)mosq;
    (void)userdata;
    
    mqtt_connected = 0;
    debug_print("MQTT: Verbindung getrennt (%d)\n", rc);
}

// MQTT Initialisierung aktualisiert
int init_mqtt(void) {
    mosquitto_lib_init();
    mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) {
        printf("FEHLER: MQTT Client konnte nicht erstellt werden\n");
        return -1;
    }

    // Callbacks setzen
    mosquitto_connect_callback_set(mosq, mqtt_connect_callback);
    mosquitto_disconnect_callback_set(mosq, mqtt_disconnect_callback);
    mosquitto_message_callback_set(mosq, mqtt_message_callback);

    // Verbindungsversuch
    if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        printf("WARNUNG: MQTT Verbindung fehlgeschlagen - verwende nur HTTP\n");
        return -1;
    }

    // Subscribe to control topic
    if (mosquitto_subscribe(mosq, NULL, MQTT_CONTROL_TOPIC, 0) != MOSQ_ERR_SUCCESS) {
        printf("WARNUNG: MQTT Subscribe fehlgeschlagen\n");
        return -1;
    }

    // Start MQTT loop in background
    if (mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS) {
        printf("WARNUNG: MQTT Loop konnte nicht gestartet werden\n");
        return -1;
    }

    printf("MQTT Client verbunden mit %s:%d\n", MQTT_HOST, MQTT_PORT);
    return 0;
}

// HTTP Server initialisieren
int init_http_server() {
    struct sockaddr_in server_addr;
    
    http_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (http_socket < 0) {
        printf("FEHLER: HTTP Socket konnte nicht erstellt werden\n");
        return -1;
    }
    
    int opt = 1;
    setsockopt(http_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(HTTP_PORT);
    
    if (bind(http_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("FEHLER: HTTP Socket konnte nicht gebunden werden\n");
        return -1;
    }
    
    if (listen(http_socket, 3) < 0) {
        printf("FEHLER: HTTP Server konnte nicht gestartet werden\n");
        return -1;
    }
    
    printf("HTTP Server läuft auf Port %d\n", HTTP_PORT);
    return 0;
}

// HTTP Request Handler
void* http_server_thread(void* arg) {
    (void)arg;  // Ungenutzte Parameter markieren
    char buffer[1024];
    char response[1024];
    
    while (running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        int client_socket = accept(http_socket, (struct sockaddr *)&client_addr, &client_len);
        if (client_socket < 0) continue;
        
        ssize_t bytes_read = read(client_socket, buffer, sizeof(buffer)-1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            
            // Parse HTTP request
            if (strstr(buffer, "GET /status") != NULL) {
                // Status Abfrage
                snprintf(response, sizeof(response), 
                        "{\"active\": %s, \"connected\": %s}", 
                        measurement_active ? "true" : "false",
                        HPS3D_IsConnect(g_handle) ? "true" : "false");
            }
            else if (strstr(buffer, "POST /start") != NULL) {
                // Messung starten
                measurement_active = 1;
                snprintf(response, sizeof(response), "{\"status\": \"started\"}");
                debug_print("Messung aktiviert via HTTP\n");
            }
            else if (strstr(buffer, "POST /stop") != NULL) {
                // Messung stoppen
                measurement_active = 0;
                snprintf(response, sizeof(response), "{\"status\": \"stopped\"}");
                debug_print("Messung deaktiviert via HTTP\n");
            }
            else {
                // Unbekannter Befehl
                snprintf(response, sizeof(response), "{\"error\": \"unknown command\"}");
            }
            
            // HTTP Response senden
            char http_response[2048];
            snprintf(http_response, sizeof(http_response), HTTP_RESPONSE, 
                    strlen(response), response);
            write(client_socket, http_response, strlen(http_response));
        }
        
        close(client_socket);
    }
    
    return NULL;
}

// Mess-Thread
void* measure_thread(void* arg) {
    (void)arg;  // Ungenutzte Parameter markieren
    
    while (running) {
        if (!measurement_active) {
            usleep(100000);  // 100ms Pause wenn inaktiv
            continue;
        }

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

// Cleanup aktualisiert
void cleanup(void) {
    debug_print("Cleanup...\n");
    
    running = 0;  // Threads stoppen
    
    if (g_handle >= 0) {
        HPS3D_StopCapture(g_handle);
        HPS3D_CloseDevice(g_handle);
    }
    
    if (mosq) {
        if (mqtt_connected) {
            mosquitto_disconnect(mosq);
        }
        mosquitto_loop_stop(mosq, true);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
    }
    
    if (http_socket >= 0) {
        close(http_socket);
    }
    
    HPS3D_MeasureDataFree(&g_measureData);
    HPS3D_UnregisterEventCallback();
    
    if (debug_file) {
        fclose(debug_file);
    }
    
    unlink(PID_FILE);
    
    debug_print("Service beendet\n");
}

// Hauptprogramm
int main(int argc, char *argv[]) {
    printf("HPS3D-160 LIDAR Service gestartet (Messung initial deaktiviert)\n");
    
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
    
    // MQTT initialisieren
    init_mqtt();
    
    // HTTP Server starten
    if (init_http_server() != 0) {
        cleanup();
        return 1;
    }
    
    // LIDAR initialisieren aber noch nicht messen
    if (init_lidar() != 0) {
        cleanup();
        return 1;
    }
    
    debug_print("Service gestartet, warte auf Aktivierung via MQTT/HTTP...\n");
    
    // Threads starten
    pthread_t measure_tid, output_tid, http_tid;
    
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
    
    if (pthread_create(&http_tid, NULL, http_server_thread, NULL) != 0) {
        printf("FEHLER: HTTP-Thread konnte nicht erstellt werden\n");
        cleanup();
        return 1;
    }
    
    // Auf Threads warten
    pthread_join(measure_tid, NULL);
    pthread_join(output_tid, NULL);
    pthread_join(http_tid, NULL);
    
    cleanup();
    return 0;
}