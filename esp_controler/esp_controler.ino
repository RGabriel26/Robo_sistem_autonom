
#include "motoare.h" // functii destinate deplasarii 
#include "antena_control.h" // funtii destinate orientarii antenei
#include "brat_control.h" // functii destinate controlului bratului apucator
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Arduino.h>
#include <WiFiUdp.h>

/*
================================================================================
 FUNCTII DISPONIBILE IN FISIERELE HEADER IMPORTATE
================================================================================

motoare.h:
  - motoare_deplasareFata()           - deplasare inainte
  - motoare_deplasareSpate()          - deplasare inapoi
  - motoare_rotireStanga()            - rotire in stanga
  - motoare_rotireDreapta()           - rotire in dreapta
  - motoare_verticalStanga()          - deplasare laterala stanga (roti magnum)
  - motoare_verticalDreapta()         - deplasare laterala dreapta (roti magnum)
  - motoare_stop()                    - oprire completa a motoarelor
  - motoare_executieRetragere()       - pentru executia manevrei de retragere din statie dupa prindere/eliberare obiect

antena_control.h:
  - connect_statie(const char* ssid, const char* password)          - conectare la o retea Wi-Fi specificata
  - det_unghi_orientare()                                           - returneaza unghiul cu cel mai bun semnal RSSI detectat

brat_control.h:
  - init_brat()         - pozitionare initiala a servomotoarelor bratului apucator
  - brat_prindere()     - secventa de prindere: capul apuca, bratul se ridica
  - brat_eliberare()    - secventa de eliberare: bratul coboara, capul elibereaza

================================================================================
*/

const char versiune[] = "1.1.1";                                    // versiunea programului - modificare logica de deplasare urmarire

// date de conectare pentru statii
const char* ssid_statie[] = {"", "Zona_A", "Zona_B", "Zona_C"};
const char* password_statie[] = {"", "12345678", "12345678", "12345678"}; 

// instante servo
Servo servoAntena; // pentru antena
Servo servoBase;   // pentru brat - servo de la baza    - miscare verticala
Servo servoHead;   // pentru brat - servo de la efector - miscare orizontala

// instanda UDP
WiFiUDP udpReceiver;
const unsigned int localPort = 4210; // port pentru comunicarea prin protocolul UDP cu statia A
 
// parametrii
#define PROX_RSSI_MAX -55           // parametru de prag pentru determinarea zonei de proximitate fata de o statie
const int pwmMaxVal = 130;                 // valoare de PWM pentru deplasare
const int unghiCentrareAntena = 87;       // unghiul de centrare al antenei in pozitia initiala

// variabile globale
enum StareComportament{              // enum folosit pentru determinarea executiei comenzilor algoritmului comportamental
  STARE_VERIFICARE_PREZENTA_OBIECT,
  STARE_ZONA_A,
  STARE_ZONA_B, 
  STARE_ZONA_C, 
  STARE_REPAUS
};

enum TipDeplasare{                   // enum folosit pentru controlul deplasarii robotului
  DEPLASARE_URMARIRE, 
  DEPLASARE_CAUTARE_STATIONARA, 
  DEPLASARE_POZITIONARE_OBIECT,
  DEPLASARE_STOP
};

volatile int conectareStatie = 0;                   // sttari posibilie {0,1,2,3} = {STOP, ZONA_A, ZONA_B, ZONA_C}
volatile int obiect_prezent_zonaA = 0;              // pentru stocarea prezentei obiectului din ZONA_A
volatile int unghiOrientare = 0;                    // unghiul de comanda al servo unde s-a depistat valoare maxima RSSI din intervalul de unghiuri in care s-au facut cautarile
volatile int proximitateRSSI = 0;                   // valoarea rssi masurata corespunzatoare unghiului de orientare
volatile int obiect_detectat = 0;                   // inregistreaza prezenta obiectului din zona A
volatile int obiect_x = 0;                          // coordonate x al obiectului detectat
volatile int obiect_y = 0;                          // coordonate y al obiectului detectat
volatile int obiect_w = 0;                          // coordonate w al obiectului detectat
volatile int obiect_h = 0;                          // coordonate h al obiectului detectat

portMUX_TYPE muxUART = portMUX_INITIALIZER_UNLOCKED;    // mutex acces variabile prin uart
portMUX_TYPE muxRSSI = portMUX_INITIALIZER_UNLOCKED;    // mutex acces variabila RSSI
portMUX_TYPE muxVarG = portMUX_INITIALIZER_UNLOCKED;    // mutex acces variabile globale
portMUX_TYPE muxUNGHI = portMUX_INITIALIZER_UNLOCKED;   // mutex acces variabila unghi de orientare al antenei

// instanta a structurii de date enum 
StareComportament stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT;
TipDeplasare tipDeplasare_Robot = DEPLASARE_STOP;

// handle-uri pentru task-uri
TaskHandle_t handleTaskControl_servoAntena = NULL;
TaskHandle_t handleTaskComportamentRobot = NULL;
TaskHandle_t handleTaskCitireUART = NULL;
TaskHandle_t handleTaskDeplasare = NULL;

//=======================================================================================================================================================
void setup() {
  // setare UART 0 in cazul debugging ului
  Serial.begin(115200);
  // setare UART 1 pentru primirea pachetelor de date de la ESP CAM
  Serial1.begin(115200, SERIAL_8N1, 35, -1);
  // initializare pini control motoare / drivere
  setup_pini();
  // initializare brat
  init_brat();

  // miscari de test ale sistemelor cu servomotoare
  brat_prindere(); // prindere
  delay(500);
  brat_eliberare(); // eliberare
  delay(500);
  init_brat(); // initializare brat

  servoAntena.write(0);
  delay(500);
  servoAntena.write(180);
  delay(500);
  servoAntena.write(unghiCentrareAntena);

  // initializarea conectiune cu o statie care nu exista
  conectareStatie = 0; 

  // initializare tasks
  initTaskuri();
  delay(1000);

}
//=======================================================================================================================================================
/**
 * @brief Task pentru controlul antenei și orientarea spre sursa de semnal Wi-Fi.
 * 
 * Conectează robotul la rețeaua Wi-Fi specificată, determină unghiul de orientare,
 * și citește datele RSSI. Activează task-ul comportamental după conectare.
 */
void taskControl_servoAntena(void *parameter) {
  while (true) {
    portENTER_CRITICAL(&muxVarG);
    int varLocal = conectareStatie;
    portEXIT_CRITICAL(&muxVarG);

    if(varLocal == 0) {
      vTaskDelay(50);
      continue;
    }

    if (WiFi.status() != WL_CONNECTED || WiFi.SSID() != ssid_statie[varLocal]) { // daca nu este conectat, se incearca conectarea la statia curenta din pasul comportamental
      servoAntena.write(unghiCentrareAntena);                                                     // pozitionare antena in pozitie default
      connect_statie(ssid_statie[varLocal], password_statie[varLocal]);          // conectare la statia curenta
      Serial.print("DEBUG - taskControl_servoAntena - Conectare la statia: "); Serial.println(ssid_statie[varLocal]);
      Serial.println("DEBUG - taskControl_servoAntena - activare handleTaskComportamentRobot");
      vTaskResume(handleTaskComportamentRobot);
    } else {
      // activare roti daca robotul este conectat la o statie
      analogWrite(pinPWM, pwmMaxVal);
    
      // determinarea valorii RSSI pentru proximitate
      int val_RSSI_temp = WiFi.RSSI(); // valoare RSSI pentru proximitate

      portENTER_CRITICAL(&muxRSSI);
      proximitateRSSI = val_RSSI_temp; // salvare valoare rssi in variabila globala
      portEXIT_CRITICAL(&muxRSSI);

      // verificare daca obiectul este prezent in zona A
      verificare_obiect_zonaA();
    }
    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
//=======================================================================================================================================================
/**
 * @brief Task pentru citirea datelor de la ESP-CAM prin UART.
 * 
 * Decodifică mesajele primite, determină dacă un obiect este detectat și salvează coordonatele.
 */
void taskCitireUART(void *parameter){
  // - decodificare mesaj uart + salvare
  String buffer = "";
  
  while (true) {
    while (Serial1.available()) {
      char c = Serial1.read();
      buffer += c;
      if (buffer.endsWith("<END>")) {
        int x, y, w, h;
        int start = buffer.indexOf("<START>") + 7;
        int end = buffer.indexOf("<END>");
        String continut = buffer.substring(start, end);

        if (continut == "NO_OBJ") {
          portENTER_CRITICAL(&muxUART);
          obiect_detectat = 0;
          obiect_x = 0;
          obiect_y = 0;
          obiect_w = 0;
          obiect_h = 0;
          portEXIT_CRITICAL(&muxUART);
        } else {
          int index1 = continut.indexOf(",");
          int index2 = continut.indexOf(",", index1 + 1);
          int index3 = continut.indexOf(",", index2 + 1);
          x = continut.substring(0, index1).toInt();
          y = continut.substring(index1 + 1, index2).toInt();
          w = continut.substring(index2 + 1, index3).toInt();
          h = continut.substring(index3 + 1).toInt();

          portENTER_CRITICAL(&muxUART);
          obiect_detectat = 1;
          obiect_x = x;
          obiect_y = y;
          obiect_w = w;
          obiect_h = h;
          portEXIT_CRITICAL(&muxUART);
        }

        buffer = "";
      }
    }
    vTaskDelay(10);
  }
}
//=======================================================================================================================================================
/**
 * @brief Task pentru controlul deplasării robotului în funcție de unghiul de orientare.
 * 
 * Controlează motoarele pentru deplasare în față, rotire stânga/dreapta,
 * sau alte manevre în funcție de unghiul de orientare al antenei.
 */
void taskControl_Deplasare(void *parameter) {
  while (true) {
    portENTER_CRITICAL(&muxVarG);
    TipDeplasare tipDeplasare_Robot_local = ::tipDeplasare_Robot; // actualizare tip deplasare
    portEXIT_CRITICAL(&muxVarG);

    switch (tipDeplasare_Robot_local) {
      case DEPLASARE_STOP:{
        // Logica pentru stationare
        motoare_stop();
        break;
      }

      case DEPLASARE_URMARIRE:{
        motoare_stop(); // oprire motoare inainte de orientare
        
        // Logica pentru urmarire   
        int rssi_temp = 0;
        
        int unghiProvenienta = det_unghi_orientare(rssi_temp);

        portENTER_CRITICAL(&muxUNGHI);
        ::proximitateRSSI = rssi_temp;        // salvare valoare rssi in variabila globala
        ::unghiOrientare = unghiProvenienta;  // salvare valoare unghi in variabila globala pentru debuging
        portEXIT_CRITICAL(&muxUNGHI);

        if (unghiProvenienta < 20) {         // obiectul este in stanga
          motoare_rotireStanga();
          vTaskDelay(300);
          motoare_stop();
        } else if (unghiProvenienta > 160) { // obiectul este in dreapta
          motoare_rotireDreapta();
          vTaskDelay(300);
          motoare_stop();
        } else {                             // obiectul este in fata
          motoare_deplasareFata();
          vTaskDelay(500);
          motoare_stop();
        }
        break;
      }

      case DEPLASARE_CAUTARE_STATIONARA:{
        // Logica pentru cautare stationara
        motoare_rotireDreapta();

        break;
      }

      case DEPLASARE_POZITIONARE_OBIECT:{
       //vTaskSuspend(handleTaskComportamentRobot); // suspendare task comportament pentru a evita conflicte
        // Logica pentru pozitionare obiect
        portENTER_CRITICAL(&muxUART);
        int obiect_detectat_local = ::obiect_detectat;
        int obiect_x_local = ::obiect_x;
        int obiect_y_local = ::obiect_y;
        int obiect_w_local = ::obiect_w;
        int obiect_h_local = ::obiect_h;
        portEXIT_CRITICAL(&muxUART);

        // logica de pozitionarea robotului pe traiectoria obiectului
        if (obiect_detectat_local) {
          Serial.println("DEBUG - DEPLASARE_POZITIONARE_OBIECT - Obiect detectat, pozitionare robot...");
          
          // logica de deplasare in fata
          if (obiect_x_local < 30) { // obiectul este in stanga
            Serial.println("DEBUG - DEPLASARE_POZITIONARE_OBIECT - Obiect in stanga, deplasare laterala stanga...");
            motoare_lateralStanga();
            vTaskDelay(200);
            motoare_stop();
            vTaskDelay(500);
          } else if (obiect_x_local > 55) { // obiectul este in dreapta
            Serial.println("DEBUG - DEPLASARE_POZITIONARE_OBIECT - Obiect in dreapta, deplasare laterala dreapta...");
            motoare_lateralDreapta();
            vTaskDelay(200);
            motoare_stop();
            vTaskDelay(500);
          } else { // obiectul este in fata
            motoare_deplasareFata();
            if(obiect_y_local > 60){
              //vTaskSuspend(handleTaskComportamentRobot); // suspendare task comportament pentru a evita conflicte
              Serial.println("DEBUG - DEPLASARE_POZITIONARE_OBIECT - Obiect in pozitie de prindere, executare prindere...");
              motoare_stop();
              vTaskDelay(1000);
              motoare_deplasareFata();
              vTaskDelay(100);
              motoare_stop();
              vTaskDelay(500);
              brat_prindere(); // executie prindere
              vTaskDelay(1000);
              motoare_executieRetragere(); // executie retragere
              vTaskDelay(1000);
              motoare_stop();
              control_stareComportamentRobot(STARE_ZONA_B);
              //vTaskResume(handleTaskComportamentRobot);
            }else{
              vTaskDelay(500); 
              motoare_stop();
            }
          }
        }
        break;
      }
      default:
        motoare_stop();
        break;
    }
    vTaskDelay(1);
  }
}
//=======================================================================================================================================================
/**
 * @brief Taskul principal de comportament al robotului – controlează starea sistemului.
 * 
 * În funcție de context (RSSI, detecția obiectului, prezența în zonă), comută între stări și activează task-uri.
 */
void taskComportamentRobot(void *parameter) {
  while (true) {
    vTaskResume(handleTaskControl_servoAntena);
    vTaskResume(handleTaskCitireUART);
    vTaskResume(handleTaskDeplasare);

    // actualizare variabile locale sincronizate
    portENTER_CRITICAL(&muxVarG);
    int statieCurenta = ::conectareStatie;
    portEXIT_CRITICAL(&muxVarG);

    portENTER_CRITICAL(&muxRSSI);
    int rssi_local = ::proximitateRSSI;
    int unghiProvenienta_local = ::unghiOrientare;
    portEXIT_CRITICAL(&muxRSSI);

    portENTER_CRITICAL(&muxUART);
    int obiect_prezent_local  = ::obiect_prezent_zonaA;
    int obiect_detectat_local = ::obiect_detectat;
    int obiect_x_local = ::obiect_x; 
    int obiect_y_local = ::obiect_y;    
    int obiect_w_local = ::obiect_w;
    int obiect_h_local = ::obiect_h;
    portEXIT_CRITICAL(&muxUART);

    // debug scurt
    Serial.println("\n=== DEBUG COMPORTAMENT ===");
    Serial.print("Versiune: "); Serial.println(versiune);
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("STARE: "); Serial.println(stareComport_Robot);
    Serial.print("RSSI: "); Serial.println(rssi_local);
    Serial.print("UNGHI: "); Serial.println(unghiProvenienta_local);
    Serial.print("OBIECT PREZENT: "); Serial.println(obiect_prezent_local);
    Serial.print("OBIECT DETECTAT: "); Serial.println(obiect_detectat_local);
    Serial.print("OBIECT DETECTAT - COORDONATE: "); Serial.print(obiect_x_local); Serial.print(", "); Serial.print(obiect_y_local); Serial.print(", "); Serial.print(obiect_w_local); Serial.print(", "); Serial.println(obiect_h_local);
    Serial.println("=============================");

    // executie comportament pe baza de stari
    switch (stareComport_Robot) {
      case STARE_VERIFICARE_PREZENTA_OBIECT:
        handleStare_Verificare(obiect_prezent_local);
        break;

      case STARE_ZONA_A:
        handleStareZona_A(rssi_local, obiect_detectat_local);
        break;

      case STARE_ZONA_B:
        handleStareZona_B(rssi_local);
        break;

      case STARE_ZONA_C:
        handleStareZona_C(rssi_local, obiect_prezent_local);
        break;

      case STARE_REPAUS:
        break;
    }
    vTaskDelay(100);
  }
}
//=======================================================================================================================================================
/**
 * @brief Tratează starea inițială – verifică dacă există obiect în zona A.
 * 
 * Dacă este prezent obiectul, comută către STARE_ZONA_A, altfel către STARE_ZONA_C.
 */
void handleStare_Verificare(int obiect_prezent) {
  Serial.println("DEBUG - STARE_VERIFICARE");

  portENTER_CRITICAL(&muxVarG);
  conectareStatie = 1;
  portEXIT_CRITICAL(&muxVarG);

  if (isConnectedToStation(1)) {
    vTaskDelay(3000);
    if (obiect_prezent) {
      Serial.println("DEBUG - STARE_VERIFICARE - Obiect prezent. Trecere la STARE_ZONA_A.");
      control_stareComportamentRobot(STARE_ZONA_A);
    } else {
      Serial.println("DEBUG - STARE_VERIFICARE - Obiect absent. Trecere la STARE_ZONA_C.");
      control_stareComportamentRobot(STARE_ZONA_C);
    }
  } else {
    asteptareReconectare(1);
    }
}
//=======================================================================================================================================================
/**
 * @brief Tratează comportamentul robotului când este în zona A.
 * 
 * Activează task-urile corespunzătoare în funcție de intensitatea RSSI și prezența obiectului.
 */
void handleStareZona_A(int RSSI, int obiect_detectat) {
  Serial.println("DEBUG - STARE_ZONA_A");

  portENTER_CRITICAL(&muxVarG);
  conectareStatie = 1;
  portEXIT_CRITICAL(&muxVarG);

  if (isConnectedToStation(1)) {
    if (obiect_detectat == 1) {
      Serial.println("DEBUG - STARE_ZONA_A - Obiect detectat vizual. - Activare POZITIONARE.");
      control_taskDeplasare(DEPLASARE_POZITIONARE_OBIECT);
      // dupa detectarea si pozitionarea obiectului, se trece la STARE_ZONA_B
      // comportament tratat in taskControl_Deplasare dupa ce obiectul este prins
    } else if (RSSI > PROX_RSSI_MAX) { // conditie in care se verifica daca s-a ajuns in zona de proximitate
      Serial.println("DEBUG - STARE_ZONA_A - RSSI puternic.          - Activare STATIONARA.");
      // control_taskDeplasare(DEPLASARE_CAUTARE_STATIONARA);
    } else {
      Serial.println("DEBUG - STARE_ZONA_A - RSSI slab.              - Activare URMARIRE.");
      control_taskDeplasare(DEPLASARE_URMARIRE);
    }
  } else {
    control_taskDeplasare(DEPLASARE_STOP);
    asteptareReconectare(1);
  }
}
//=======================================================================================================================================================
/**
 * @brief Tratează comportamentul robotului în zona B (eliberare obiect).
 * 
 * Eliberează obiectul și execută retragerea, apoi revine la verificarea prezenței.
 */
void handleStareZona_B(int RSSI) {
  Serial.println("DEBUG - STARE_ZONA_B");

  portENTER_CRITICAL(&muxVarG);
  conectareStatie = 2;
  portEXIT_CRITICAL(&muxVarG);

  if (isConnectedToStation(2)) {
    if (RSSI > PROX_RSSI_MAX) {
      Serial.println("DEBUG - STARE_ZONA_B - Executare eliberare.");
      motoare_stop(); 
      vTaskDelay(500);
      brat_eliberare();
      vTaskDelay(500);
      motoare_executieRetragere();
      vTaskDelay(1000);
      control_stareComportamentRobot(STARE_VERIFICARE_PREZENTA_OBIECT);
      vTaskDelay(500);
      control_taskDeplasare(DEPLASARE_STOP);
    } else {
      Serial.println("DEBUG - STARE_ZONA_B - RSSI slab.              - Activare URMARIRE.");
      control_taskDeplasare(DEPLASARE_URMARIRE);
    }
  } else {
    control_taskDeplasare(DEPLASARE_STOP);
    asteptareReconectare(2);
  }
}
//=======================================================================================================================================================
/**
 * @brief Tratează comportamentul robotului în zona C (așteptare pentru apariția obiectului).
 * 
 * Verifică RSSI și prezența obiectului în zona A înainte de a comuta înapoi în STARE_ZONA_A.
 */
void handleStareZona_C(int RSSI, int obiect_prezent) {
  Serial.println("DEBUG - STARE_ZONA_C");

  portENTER_CRITICAL(&muxVarG);
  conectareStatie = 3;
  portEXIT_CRITICAL(&muxVarG);

  if (isConnectedToStation(3)) {
    if (RSSI > -50) {
      Serial.println("DEBUG - STARE_ZONA_C - PROXIMITATE - Oprire si asteptare prezenta obiect.");
      // verificare prezenta obiect zona A
      // stabilire conectare statia A
      portENTER_CRITICAL(&muxVarG);
      conectareStatie = 1;
      portEXIT_CRITICAL(&muxVarG);

      control_taskDeplasare(DEPLASARE_STOP);
      asteptareReconectare(1);

      if (isConnectedToStation(1)) {
        int prezenta = 0;
        while (!prezenta) {
          Serial.println("DEBUG - STARE_ZONA_C - PROXIMITATE -  Asteptare prezenta obiect in zona A...");
          portENTER_CRITICAL(&muxUART);
          prezenta = ::obiect_prezent_zonaA;
          portEXIT_CRITICAL(&muxUART);
          vTaskDelay(1000);
        }
      }else{
        Serial.println("DEBUG - STARE_ZONA_C - CONECTARE ZONA_A - ESUATA");
      }
      Serial.println("DEBUG - STARE_ZONA_C - Obiect prezent in zona A. Trecere la STARE_ZONA_A.");
      control_stareComportamentRobot(STARE_ZONA_A);
      control_taskDeplasare(DEPLASARE_STOP);
    } else {
      Serial.println("DEBUG - STARE_ZONA_C - RSSI slab.              - Activare URMARIRE.");
      control_taskDeplasare(DEPLASARE_URMARIRE);
    }
  } else {
    control_taskDeplasare(DEPLASARE_STOP);
    asteptareReconectare(3);
  }
}
//=======================================================================================================================================================
/**
 * @brief Controlează tipul de deplasare al robotului.
 * @param tipDeplasare Tipul de deplasare dorit (enum TipDeplasare).
 * 
 */
void control_taskDeplasare(enum TipDeplasare tipDeplasare) {
  portENTER_CRITICAL(&muxVarG);
  ::tipDeplasare_Robot = tipDeplasare; // actualizare tip deplasare
  portEXIT_CRITICAL(&muxVarG);
}
//=======================================================================================================================================================
/**
 * @brief Controlează starea comportamentală a robotului.
 * 
 * @param stareComport Starea comportamentului dorită.
 */
void control_stareComportamentRobot(enum StareComportament stareComport) {
  portENTER_CRITICAL(&muxVarG);
  ::stareComport_Robot = stareComport; // actualizare stare comportament
  portEXIT_CRITICAL(&muxVarG);
}
//=======================================================================================================================================================
/**
 * @brief Verifică dacă robotul este conectat la o anumită stație Wi-Fi.
 * 
 * @param stationIndex Indexul stației (1 = A, 2 = B, 3 = C)
 * @return true dacă este conectat, false altfel.
 */
bool isConnectedToStation(int stationIndex) {
  return WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[stationIndex];
}
//=======================================================================================================================================================
/**
 * @brief Suspendă temporar execuția și încearcă reconectarea la stația dorită.
 * 
 * @param conectareStare_local Indexul stației către care se dorește reconectarea.
 */
void asteptareReconectare(int conectareStare_local){
  Serial.println("DEBUG - asteptareReconectare");
  control_taskDeplasare(DEPLASARE_STOP);
  //WiFi.disconnect(true); // deconectare de la retea

  portENTER_CRITICAL(&muxVarG);
  ::conectareStatie = conectareStare_local;
  portEXIT_CRITICAL(&muxVarG); 

  servoAntena.write(unghiCentrareAntena);


  vTaskSuspend(handleTaskComportamentRobot);
  vTaskDelay(1);
}
//=======================================================================================================================================================
/**
 * @brief Verifică dacă un obiect este prezent în zona A prin recepția unui mesaj UDP.
 * 
 * Dacă este conectat la SSID-ul corespunzător, citește mesajul UDP și actualizează
 * starea obiectului prezent în zona A.
 */
void verificare_obiect_zonaA() {
  if(WiFi.SSID() == ssid_statie[1]){ // verificare conectiune Zona_A
      int packetSize = udpReceiver.parsePacket();
      if (packetSize) {
        char incomingPacket[255];
        int len = udpReceiver.read(incomingPacket, 255);
        if (len > 0) {
          incomingPacket[len] = '\0';         // terminator de string
          int valoare = atoi(incomingPacket); // conversie la int

          portENTER_CRITICAL(&muxUART);
          obiect_prezent_zonaA = valoare;
          portEXIT_CRITICAL(&muxUART);

          Serial.print("DEBUG - taskControl_servoAntena - Mesaj UDP primit (zona A): ");
          Serial.println(valoare);
        }
      }
    }
}
//=======================================================================================================================================================
/**
 * @brief Inițializează pinii hardware.
 */
void setup_pini() {
  // setare pini pentru motoare
  pinMode(DRIVER_A_IN1, OUTPUT);
  pinMode(DRIVER_A_IN2, OUTPUT);
  pinMode(DRIVER_A_IN3, OUTPUT);
  pinMode(DRIVER_A_IN4, OUTPUT);
  pinMode(DRIVER_B_IN1, OUTPUT);
  pinMode(DRIVER_B_IN2, OUTPUT);
  pinMode(DRIVER_B_IN3, OUTPUT);
  pinMode(DRIVER_B_IN4, OUTPUT);
  pinMode(pinPWM, OUTPUT);

  // setare pini pentru servo
  pinMode(pinServoAntena, OUTPUT);
  servoAntena.attach(pinServoAntena); 

  // setare pini pentru brat
  pinMode(pinServoBase, OUTPUT);
  pinMode(pinServoHead, OUTPUT);
  servoBase.attach(pinServoBase);
  servoHead.attach(pinServoHead);

  delay(100);
  servoAntena.write(unghiCentrareAntena);
}
//=======================================================================================================================================================
/**
 * @brief Inițializează toate task-urile FreeRTOS utilizate de sistem.
 * 
 * Creează și suspendă inițial toate task-urile pentru controlul antenei, deplasare și comunicare.
 */
void initTaskuri(){
    xTaskCreatePinnedToCore(
    taskControl_servoAntena,       // functia
    "taskControl_servoAntena",     // nume task
    4096,             // stack size
    NULL,             // parametru
    1,                // prioritate
    &handleTaskControl_servoAntena,             // handle (optional)
    0                 // core 0
  );
  vTaskSuspend(handleTaskControl_servoAntena); // suspendare initiala a taskului de control al antenei

    xTaskCreatePinnedToCore(
    taskControl_Deplasare,
    "taskControl_Deplasare",
    4096,
    NULL,
    1,
    &handleTaskDeplasare,
    1               // core 1
  );
  vTaskSuspend(handleTaskDeplasare); // suspendare initiala a taskului de deplasare

  xTaskCreatePinnedToCore(
    taskCitireUART,
    "taskCitireUART",
    4096,
    NULL,
    1,
    &handleTaskCitireUART,
    1               // core 1
  );
  vTaskSuspend(handleTaskCitireUART); // suspendare initiala a taskului de citire UART

  xTaskCreatePinnedToCore(
    taskComportamentRobot,
    "taskComportamentRobot",
    4096,
    NULL,
    1,
    &handleTaskComportamentRobot,
    1               // core 1
  );
  //vTaskSuspend(handleTaskComportamentRobot); // suspendare initiala a taskului de comportament al robotului
}
//=======================================================================================================================================================
/**
 * @brief Funcție goală de tip loop() specifică Arduino. Nu este folosită în acest sistem.
 */
void loop(){
  // Nu este necesară o funcție loop() în acest sistem, deoarece toate operațiunile sunt gestionate de task-uri FreeRTOS.
  // Task-urile rulează în fundal și gestionează comportamentul robotului.
  vTaskDelay(1000); // Delay pentru a evita blocarea CPU, dacă este necesar.

  //   // Verifică dacă toate taskurile sunt suspendate
  // if (eTaskGetState(handleTaskComportamentRobot) == eSuspended) {
    

  //   /* aici vreau sa se citeasca o valoare de la tastatura prin uart
  //   iar poentru fiecare caracter primit, care va fi un numar sa se execute una din comenzile destinate controlului motoarelor astfel:
  //   0 - oprire
  //   1 - deplasare fata
  //   2 - deplasare spate
  //   3 - rotire stanga
  //   4 - rotire dreapta
  //   5 - deplasare laterala stanga
  //   6 - deplasare laterala dreapta
  //   */
  //   if (Serial.available()) {
  //     char comanda = Serial.read();
  //       switch (comanda) {
  //         case '0':
  //           delay(2000);
  //           motoare_stop();
  //           break;
  //         case '1':
  //           delay(2000);
  //           motoare_deplasareFata();
  //           break;
  //         case '2':
  //           delay(2000);
  //           motoare_deplasareSpate();
  //           break;
  //         case '3':
  //           delay(2000);
  //           motoare_rotireStanga();
  //           break;
  //         case '4':
  //           delay(2000);
  //           motoare_rotireDreapta();
  //           break;
  //         case '5':
  //           delay(2000);
  //           motoare_lateralStanga();
  //           break;
  //         case '6':
  //           delay(2000);
  //           motoare_lateralDreapta();
  //           break;
  //         case '7':
  //           delay(2000);
  //           brat_prindere();
  //           break;
  //         case '8':
  //           delay(2000);
  //           brat_eliberare();
  //           break;  
  //         default:
  //           Serial.println("Comanda necunoscuta!");
  //       }

  //     delay(2000);
  //     motoare_stop();
  //   }

  // }

}
