#ifndef ANTENA_CONTROL_H
#define ANTENA_CONTROL_H

#include <WiFi.h>
#include <ESP32Servo.h>
#include "motoare.h" // functii destinate deplasarii 

// ========================
// DEFINIRE PINI
// ========================

#define pinServoAntena 4 ///< Pinul de control al servomotorului antenei

// ========================
// SETĂRI FUNCȚIONARE
// ========================

#define DELAY_UNGHI 50       ///< Timp de așteptare între poziționări succesive ale antenei [ms]
#define PAS_UNGHI 10         ///< Incrementul unghiului de scanare
#define LIM_INF 20           ///< Limită inferioară pentru unghiul de scanare
#define LIM_SUP 160          ///< Limită superioară pentru unghiul de scanare

// ========================
// VARIABILE EXTERNE
// ========================

/// Lista SSID-urilor disponibile pentru stații (definită în .ino)
extern const char* ssid_statie[];

/// Intervalul curent de unghiuri de scanare (modificabil dinamic)
extern volatile int interval[2];

/// Instanță globală a servomotorului antenei
extern Servo servoAntena;

/// Instanță UDP pentru recepția de pachete de date
extern WiFiUDP udpReceiver;

/// Portul local pentru recepția UDP
extern const unsigned int localPort;

/// Indexul curent al stației la care se face conectarea (0 = dezactivat)
extern volatile int conectareStatie;

/**
 * @brief Se conectează la o rețea Wi-Fi specificată.
 * 
 * Dezactivează PWM, resetează conexiunile și pornește modul STA.
 * Activează și receptorul UDP dacă este conectat la Statia_A.
 * 
 * @param ssid SSID-ul rețelei
 * @param password Parola rețelei
 */
void connect_statie(const char* ssid, const char* password) {
    digitalWrite(pinPWM, LOW);
    WiFi.disconnect();
    udpReceiver.stop();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500);
        Serial.print(".");
    }
    if (WiFi.SSID() == ssid_statie[1]) udpReceiver.begin(localPort);
    Serial.println("");
    Serial.print("WiFi connected ");
    Serial.println(ssid);
}

/**
 * @brief Trunchiază intervalul de căutare a semnalului RSSI în jurul unghiului maxim.
 * 
 * Această funcție este utilizată intern pentru a restrânge dinamic intervalul
 * în care se face căutarea unghiului optim.
 * 
 * @param unghi_max_rssi Unghiul unde s-a găsit semnalul maxim
 * @param val_restrangere Valoarea cu care se restrânge din fiecare parte
 */
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

/**
 * @brief Determină unghiul de orientare cu cel mai bun semnal Wi-Fi.
 * 
 * Antena se rotește în intervalul [interval[0], interval[1]] în pași de `PAS_UNGHI`,
 * măsoară semnalul RSSI și determină unghiul unde semnalul este maxim.
 * Intervalul este apoi restrâns dinamic.
 * 
 * @param[out] rssi_max_out Valoarea RSSI maximă găsită (prin referință)
 * @return int Unghiul la care s-a găsit cel mai bun semnal
 */
int det_unghi_orientare(int &rssi_max_out) {
        int unghi_max_rssi = 0;
        int max_rssi = -200;

        for (int unghi = interval[0]; unghi <= interval[1]; unghi += PAS_UNGHI) {
            servoAntena.write(unghi);
            vTaskDelay(DELAY_UNGHI);
            int rssi = 0;
            for (int i = 0; i < 5; i++) {
                rssi += WiFi.RSSI();
                vTaskDelay(5);
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
            vTaskDelay(DELAY_UNGHI);
            int rssi = 0;
            for (int i = 0; i < 5; i++) {
                rssi += WiFi.RSSI();
                vTaskDelay(5);
            }

            if (rssi > max_rssi) {
                max_rssi = rssi;          // actualizare valoare maxima rssi  
                unghi_max_rssi = unghi;   // actualizare unghi cu valoarea maxim rssi
            }
        }

        trunchiere_interval(unghi_max_rssi, 20);
        rssi_max_out = max_rssi; // valoarea transmisa prin referinta

        return unghi_max_rssi;
}

#endif
