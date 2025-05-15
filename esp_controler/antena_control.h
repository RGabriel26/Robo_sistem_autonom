#ifndef ANTENA_CONTROL_H
#define ANTENA_CONTROL_H

#include <WiFi.h>
#include <ESP32Servo.h>

// definire pini
#define pinServoAntena 4 

// setari de functionare
#define DELAY_UNGHI 50
#define PAS_UNGHI 10
#define LIM_INF 20
#define LIM_SUP 160

// variabile externe - definite în .ino
extern const char* ssid_statie[];
extern volatile int interval[2];
extern Servo servoAntena;
extern WiFiUDP udpReceiver;
extern const unsigned int localPort;

void connect_statie(const char* ssid, const char* password) {
    WiFi.disconnect();
    udpReceiver.stop();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println(".");
    }
    if (WiFi.SSID() == ssid_statie[1]) udpReceiver.begin(localPort);
    Serial.println("");
    Serial.print("WiFi connected ");
    Serial.println(ssid);
}

// utilizata doar local
void trunchiere_interval(int unghi_max_rssi, int val_restrangere) {
    if (unghi_max_rssi == interval[0] || unghi_max_rssi == interval[1]) {
        int nou_lim_inf = unghi_max_rssi - (val_restrangere + 20);
        interval[0] = (nou_lim_inf <= LIM_INF) ? LIM_INF : nou_lim_inf;

        int nou_lim_sup = unghi_max_rssi + (val_restrangere + 20);
        interval[1] = (nou_lim_sup >= LIM_SUP) ? LIM_SUP : nou_lim_sup;
    } else {
        int nou_lim_inf = unghi_max_rssi - val_restrangere;
        interval[0] = (nou_lim_inf <= LIM_INF) ? LIM_INF : nou_lim_inf;

        int nou_lim_sup = unghi_max_rssi + val_restrangere;
        interval[1] = (nou_lim_sup >= LIM_SUP) ? LIM_SUP : nou_lim_sup;
    }
}

// la folosirea acestei functii este nevoie de integrarea intr-un block de interogare astfel
// pentru a se efita cazurile in care se pierde conectiunea cu statia in timpul rularii
int det_unghi_orientare() {
        int unghi_max_rssi = 0;
        int max_rssi = -200;

        for (int unghi = interval[0]; unghi <= interval[1]; unghi += PAS_UNGHI) {
            servoAntena.write(unghi);
            delay(DELAY_UNGHI);
            int rssi = 0;
            for (int i = 0; i < 5; i++) {
                rssi += WiFi.RSSI();
                delay(5);
            }
            rssi /= 5;

            if (rssi > max_rssi) {
                max_rssi = rssi;
                unghi_max_rssi = unghi;
            }
        }

        trunchiere_interval(unghi_max_rssi, 20);

        for (int unghi = interval[1]; unghi >= interval[0]; unghi -= PAS_UNGHI) {
            servoAntena.write(unghi);
            delay(DELAY_UNGHI);
            int rssi = 0;
            for (int i = 0; i < 5; i++) {
                rssi += WiFi.RSSI();
                delay(5);
            }

            if (rssi > max_rssi) {
                max_rssi = rssi;
                unghi_max_rssi = unghi;
            }
        }

        trunchiere_interval(unghi_max_rssi, 20);

        return unghi_max_rssi;
}

#endif
