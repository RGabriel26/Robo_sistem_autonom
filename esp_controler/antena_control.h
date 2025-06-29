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

#define DELAY_UNGHI 100       ///< Timp de așteptare între poziționări succesive ale antenei [ms]
#define PAS_UNGHI 30         ///< Incrementul unghiului de scanare

#define LIM_INF 20           ///< Limită inferioară pentru unghiul de scanare
#define LIM_SUP 160          ///< Limită superioară pentru unghiul de scanare

// ========================
// VARIABILE EXTERNE
// ========================

/// Lista SSID-urilor disponibile pentru stații (definită în .ino)
extern const char* ssid_statie[];

/// Instanță globală a servomotorului antenei
extern Servo servoAntena;

/// Instanță UDP pentru recepția de pachete de date
extern WiFiUDP udpReceiver;

/// Portul local pentru recepția UDP
extern const unsigned int localPort;

/// Indexul curent al stației la care se face conectarea (0 = dezactivat)
extern volatile int conectareStatie;

// ========================
// VARIABILE INTERNE
// ========================
static int interval[2] = {LIM_INF, LIM_SUP}; ///< Intervalul de scanare a unghiului pentru RSSI

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
    dacWrite(pinPWM, 0);
    WiFi.disconnect();
    udpReceiver.stop();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("\nDEBUG - connect_statie - Asteptare pentru conexiunea la SSID: "); Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500);
        Serial.print(".");
    }
    if (WiFi.SSID() == ssid_statie[1]) udpReceiver.begin(localPort);
    Serial.println("");
    Serial.print("WiFi conectat la: ");
    Serial.println(ssid);
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
int det_unghi_orientare(int &rssi_max_out, int &ok_deplasare) {
        int unghi_max_rssi = 0;
        // int max_rssi = -200;
        int max_rssi = 0; // valoare maxima rssi pentru zona de proximitate
        int rssi = 0;

        for (int unghi = interval[0]; unghi <= interval[1]; unghi += PAS_UNGHI) {
            servoAntena.write(unghi);
            vTaskDelay(DELAY_UNGHI);
            rssi = WiFi.RSSI();

            if (rssi < max_rssi) {
                max_rssi = rssi;
                unghi_max_rssi = unghi;
            }
        }
        rssi = 0;

        for (int unghi = interval[1]; unghi >= interval[0]; unghi -= PAS_UNGHI) {
            servoAntena.write(unghi);
            vTaskDelay(DELAY_UNGHI);
            rssi = WiFi.RSSI();

            if (rssi < max_rssi) {
                max_rssi = rssi;          // actualizare valoare maxima rssi  
                unghi_max_rssi = unghi;   // actualizare unghi cu valoarea maxim rssi
            }
        }
        rssi_max_out = max_rssi; // valoarea transmisa prin referinta
        ok_deplasare = 1; // setare variabila pentru controlul deplasarii
        return unghi_max_rssi;
}

#endif