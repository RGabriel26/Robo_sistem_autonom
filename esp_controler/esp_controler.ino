#include "motoare.h" // functii destinate deplasarii 
#include "antena_control.h" // funtii destinate orientarii antenei
#include "brat_control.h" // functii destinate controlului bratului apucator
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Arduino.h>
#include <WiFiUdp.h>

/*
================================================================================
 FUNCȚII DISPONIBILE ÎN FIȘIERELE HEADER IMPORTATE
================================================================================

motoare.h:
  - FORWARD()           → deplasare înainte
  - BACK()              → deplasare înapoi
  - LEFT()              → rotire în stânga
  - RIGHT()             → rotire în dreapta
  - VERTICAL_LEFT()     → deplasare laterală stânga (roți magnum)
  - VERTICAL_RIGHT()    → deplasare laterală dreapta (roți magnum)
  - STOP()              → oprire completă a motoarelor

antena_control.h:
  - connect_statie(const char* ssid, const char* password)          → conectare la o rețea Wi-Fi specificată
  - det_unghi_orientare()                                           → returnează unghiul cu cel mai bun semnal RSSI detectat

brat_control.h:
  - init_brat()         → poziționare inițială a servomotoarelor brațului apucător
  - brat_prindere()     → secvență de prindere: capul apucă, brațul se ridică
  - brat_eliberare()    → secvență de eliberare: brațul coboară, capul eliberează

================================================================================
*/

// date de conectare pentru statii
const char* ssid_statie[] = {"", "Statia_A", "Statia_B", "Statia_C"}; // indexul 0 este folosit atunci cand se doreste blocarea task ului taskAntena
const char* password_statie[] = {"", "12345678", "12345678", "12345678"};

// instante servo
Servo servoAntena; // pentru antena
Servo servoBase;   // pentru brat - servo de la baza    - miscare verticala
Servo servoHead;   // pentru brat - servo de la efector - miscare orizontala

// instanda UDP
WiFiUDP udpReceiver;
const unsigned int localPort = 4210; // trebuie să fie același ca pe ESP8266

// parametrii
#define PROX_RSSI_MAX -35

// variabile globale
// enum folosit pentru determinarea executiei comenzilor algoritmului comportamental
enum StareComportament{
  STARE_VERIFICARE_PREZENTA_OBIECT,
  STARE_ZONA_A,
  STARE_ZONA_B, 
  STARE_ZONA_C, 
  STARE_REPAUS
};
volatile int interval[2] = {LIM_INF , LIM_SUP};     // interval cu unghiurile pe care le va explora antena
volatile int conectareStatie = 0;                   // sttari posibilie {0,1,2,3} = {STOP, ZONA_A, ZONA_B, ZONA_C}
volatile int prezentaObiect_zonaA = 0;              // pentru stocarea prezentei obiectului din ZONA_A
volatile int unghiProvenienta = 0;                  // valoare RSSI corezpunzatoare unghiului de orientare obtinuta din det_unghi_orientare si folosita pentru determinarea proximitatii fata de o ZONA
volatile int obiect_detectat = 0;
volatile int obiect_x = 0;
volatile int obiect_y = 0;
volatile int obiect_w = 0; 
volatile int obiect_h = 0;
portMUX_TYPE muxUART = portMUX_INITIALIZER_UNLOCKED;    // mutex pentru blocarea accesului la o structura de cod - folosit pentru scriere/citire variabile globale
portMUX_TYPE muxRSSI = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxVarG = portMUX_INITIALIZER_UNLOCKED;

StareComportament stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT; // instanta a structurii de date enum StareComportament

// handelere pentru controlul activarii taskurilor
TaskHandle_t handleTaskDeplasare_Urmarire = NULL;
TaskHandle_t handleTaskDeplasare_CautareStationara = NULL;
TaskHandle_t handleTaskDeplasare_PozitionareObiect = NULL; 

void setup() {
  // setare UART 0 in cazul debugging ului
  Serial.begin(115200);
  // setare UART 1 pentru primirea pachetelor de date de la ESP CAM
  Serial1.begin(9600, SERIAL_8N1, 35, -1);
  // initializare pini control motoare / drivere
  pinMode(DRIVER_A_IN1, OUTPUT);
  pinMode(DRIVER_A_IN2, OUTPUT);
  pinMode(DRIVER_A_IN3, OUTPUT);
  pinMode(DRIVER_A_IN4, OUTPUT);
  pinMode(DRIVER_B_IN1, OUTPUT);
  pinMode(DRIVER_B_IN2, OUTPUT);
  pinMode(DRIVER_B_IN3, OUTPUT);
  pinMode(DRIVER_B_IN4, OUTPUT);
  pinMode(pinPWM, OUTPUT);

  // initializarea servo antenei 
  pinMode(pinServoAntena, OUTPUT);
  servoAntena.attach(pinServoAntena); 

  // initializare serbo brat
  pinMode(pinServoBase, OUTPUT);
  pinMode(pinServoHead, OUTPUT);
  servoBase.attach(pinServoBase);
  servoHead.attach(pinServoHead);

  delay(100);
  servoAntena.write(90);
  init_brat();
  // Initializare statia A
  conectareStatie = 0; 
  stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT;

  // initializare tasks
  xTaskCreatePinnedToCore(
    taskControl_servoAntena,       // funcția
    "taskControl_servoAntena",     // nume task
    4096,             // stack size
    NULL,             // parametru
    1,                // prioritate
    NULL,             // handle (optional)
    0                 // core 0
  );

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_Urmarire,
    "taskControl_Deplasare_Urmarire",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_Urmarire,
    1               // core 1
  );

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_CautareStationare,
    "taskControl_Deplasare_CautareStationare",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_CautareStationara,
    1               // core 1
  );

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_PozitionareObiect,
    "taskControl_Deplasare_PozitionareObiect",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_PozitionareObiect,
    1               // core 1
  );

  xTaskCreatePinnedToCore(
    taskCitireUART,
    "taskCitireUART",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_PozitionareObiect,
    1               // core 1
  );

  // stare finala
  digitalWrite(pinPWM, LOW); // setat in HIGH pentru tensiune continua de 3.3V, echivalent PWM 100 %

  // delay pentru asigurarea initializarilor si verificarilor
  delay(1000);
}

// Task 1: Verificare conexiune si orientarea antenei spre sursa de semnal wifi
void taskControl_servoAntena(void *parameter) {
  while (true) {

    portENTER_CRITICAL(&muxVarG);
    int varLocal = conectareStatie;
    portEXIT_CRITICAL(&muxVarG);

    if(varLocal == 0) {
      vTaskDelay(50);
      continue;
    }
    
    if (WiFi.status() != WL_CONNECTED || WiFi.SSID() != ssid_statie[varLocal]) {                                              // daca nu este conectat, se incearca conectarea la statia curenta din pasul comportamental
      servoAntena.write(90);                                                          // pozitionare antena in pozitie default
      connect_statie(ssid_statie[varLocal], password_statie[varLocal]); // conectare la statia curenta
    } else {
      // posibila necesitatea unei verificari suplimentare statiei actual conectate
      if(WiFi.SSID() == ssid_statie[1]){ // verificare conectiune statia A
        int packetSize = udpReceiver.parsePacket();
        if (packetSize) {
          char incomingPacket[255];
          int len = udpReceiver.read(incomingPacket, 255);
          if (len > 0) {
            incomingPacket[len] = '\0'; // terminator de string
            int valoare = atoi(incomingPacket); // conversie la int

            portENTER_CRITICAL(&muxUART);
            prezentaObiect_zonaA = valoare;
            portEXIT_CRITICAL(&muxUART);

            // Serial.print("Mesaj UDP primit (zona A): ");
            // Serial.println(prezentaObiect_zonaA);
          }
        }
      }
      // determinarea unghiului de orientare spre sursa de semnal
      int val_temp = det_unghi_orientare();
      portENTER_CRITICAL(&muxRSSI);
      unghiProvenienta = val_temp;
      portEXIT_CRITICAL(&muxRSSI);
      Serial.print("TEST - task cont antena - unghi: ");
      Serial.println(val_temp); // FOLOSIT PENTRU TEST
    }
    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}

// Task 2: Control motoarelor pentru a duce unghiul de orientare spre zona tampon de mers inainte
void taskControl_Deplasare_Urmarire(void *parameter) {
  digitalWrite(pinPWM, HIGH);
  // control dupa unghiul de comanda al servomotorului anatenei
  int unghiProvenienta_local = 0;
  while (true) {
    // protectie la citire
    portENTER_CRITICAL(&muxRSSI);
    unghiProvenienta_local = unghiProvenienta;
    portEXIT_CRITICAL(&muxRSSI);

  if (unghiProvenienta_local >= 70 && unghiProvenienta_local <= 110) {
    motoare_deplasareFata();
  } else if (unghiProvenienta_local < 70) {
    motoare_rotireStanga();
  } else if (unghiProvenienta_local > 110) {
    motoare_rotireDreapta();
  }
    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
void taskControl_Deplasare_CautareStationare(void *parameter) {
  digitalWrite(pinPWM, HIGH);
  // comenzi repetate de stanga - dreapta
  while (true) {
    motoare_rotireDreapta();
    delay(500);
    motoare_rotireStanga();
    delay(1000);
    motoare_rotireDreapta();
    delay(500);
    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
void taskControl_Deplasare_PozitionareObiect(void *parameter) {
  digitalWrite(pinPWM, HIGH);
  // - determinarea pozitiei de prindere
  // - activare brat_prindere()
  // - activare motoare_executieRetragere()
  // - schimbare stare - STARE_STATIA_B
  while (true) {

    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}

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

    vTaskDelay(1);
  }
}

void loop() {
  // ROBOTUL IN TIMPUL INITIALIZARII, INCEARCA CONECTAREA LA ZONA A
  // LA PRIMA EXECUTIEI A FUNCTIEI LOOP AR TREBUI CA VARIABILA prezentaObiect_zonaA SA FIE DEJA POPULATA SAU NU
  // VERIFICARE_OBIECT
      // DA
        // CAUTARE_ZONA_A
      // NU
        // CAUTARE_ZONA_C
  // CAUTARE_ZONA_A
      // - conectare STATIE_A
      // - verificare proximitate
      // - detectie
      // - prindere obiect
      // - rotire
      // - CAUTARE_ZONA_B
  // CAUTARE_ZONA_B
      // - conectare STATIE_B
      // - verificare proximitate
      // - eliberare obiect
      // - rotire
      // - VERIFICARE_OBIECT
  // CAUTARE_ZONA_C
      // - conectare STATIE_C
      // - verificare proximitate
      // - veridicare prezenta obiect din zona A
        // REPAUS
          // - conectare la STATIE_A
          // - asteptare prezentaObiect_zonaA = true
          // CAUTARE_ZONA_A
  
  // Serial.println("TEST:");
  // Serial.print("TEST - pin de pwm: ");
  // Serial.println(digitalRead(pinPWM));
  // Serial.print("TEST - statie conectata: ");
  // Serial.println(WiFi.SSID());
  // Serial.print("TEST - stare: ");
  // Serial.println(stareComport_Robot);
  // Serial.print("TEST - prezenta obiect: ");
  // Serial.println(prezentaObiect_zonaA);

  int unghiProvenienta_local = 0;

  portENTER_CRITICAL(&muxVarG);
  int varStareLocal = conectareStatie;
  portEXIT_CRITICAL(&muxVarG);

  // protectie la citire
  portENTER_CRITICAL(&muxRSSI);
  unghiProvenienta_local = unghiProvenienta;
  portEXIT_CRITICAL(&muxRSSI);
  
  switch (stareComport_Robot) {
    Serial.println("TEST - intrare switch");
    // -----------------------------------------------------------------------------
    case STARE_VERIFICARE_PREZENTA_OBIECT:
      vTaskSuspend(handleTaskDeplasare_Urmarire);
      varStareLocal = 1; // conectare STATIE_A 
      if(WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[varStareLocal]){
        if(prezentaObiect_zonaA)
          stareComport_Robot = STARE_ZONA_A;
        else
          stareComport_Robot = STARE_ZONA_C;
      }else{
        // conectare la statia A
        //connect_statie(ssid_statie[varStareLocal], password_statie[varStareLocal]);
      }
      break;
    // -----------------------------------------------------------------------------
    case STARE_ZONA_A:
        varStareLocal = 1; // conectare STATIE_A
        if(WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[varStareLocal]){ // validare ca robotul este conectat la statia corecta
          // VERIFICARE PROXIMITATE FATA DE STATIA_A SAU DETECTIE OBIECT
          // verificare detectie obiect
          if (obiect_detectat == 1){
            // trecerea pe control al detectiei
            
            vTaskResume(handleTaskDeplasare_PozitionareObiect);
            vTaskSuspend(handleTaskDeplasare_Urmarire);
            vTaskSuspend(handleTaskDeplasare_CautareStationara);
          }
          // verificare proxomitate fata de Statia_A
          if (unghiProvenienta_local < PROX_RSSI_MAX ){   // verificare proximitate 
            
            vTaskResume(handleTaskDeplasare_CautareStationara);
            vTaskSuspend(handleTaskDeplasare_Urmarire);
            vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
          }else{
            // controlul motoarelor va fi executat de taskControl_Deplasare_Urmarire
            // mentinere taskControl_DeplasareUrmarire
            digitalWrite(pinPWM, LOW);
            vTaskResume(handleTaskDeplasare_Urmarire);
            vTaskSuspend(handleTaskDeplasare_CautareStationara);
            vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
          }
        }else{
          // blocare task de deplasare si asigurarea conectarii la statia corespunzatoare
          
          vTaskSuspend(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
          //connect_statie(ssid_statie[varStareLocal], password_statie[varStareLocal]);
        }     

      break;
    // -----------------------------------------------------------------------------
    case STARE_ZONA_B: 
      varStareLocal = 2; // conectare STATIE_B
      if(WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[varStareLocal]){
        if (unghiProvenienta_local < PROX_RSSI_MAX){         // verificare proximitate
          brat_eliberare();             // eliberare obiect
          motoare_executieRetragere();  // executare retragere din zona de depozitare
          stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT;
        }else{
          // activare task de urmarire a statiei
          
          vTaskResume(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        }
      }else{
          // blocarea deplasarii daca robotul nu este conectat la statia corespunzatoare
          
          vTaskSuspend(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
          //connect_statie(ssid_statie[varStareLocal], password_statie[varStareLocal]);
        }

      break;
    // -----------------------------------------------------------------------------
    case STARE_ZONA_C: 
      varStareLocal = 3; // conectare STATIE_C
      if(WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[varStareLocal]){ // veridicare ca robotul sa fie conectat la statia corecta
        if (unghiProvenienta_local < PROX_RSSI_MAX){   // verificare proximitate
          // oprire 
          // conectare la STATIA_A
          digitalWrite(pinPWM, LOW);
          vTaskSuspend(handleTaskDeplasare_Urmarire);
          varStareLocal = 1;    // conectare STATIE_A
          //connect_statie(ssid_statie[varStareLocal], password_statie[varStareLocal]);
          if(prezentaObiect_zonaA)
            stareComport_Robot = STARE_ZONA_A;
        }else{  
          // executie deplasare prin urmarirea statie de interes
          
          vTaskResume(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        }
      }else{
        // conectare la statia corecta
        
        vTaskSuspend(handleTaskDeplasare_Urmarire);
        vTaskSuspend(handleTaskDeplasare_CautareStationara);
        vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        //connect_statie(ssid_statie[varStareLocal], password_statie[varStareLocal]);
      }

      break;
    // -----------------------------------------------------------------------------
    case STARE_REPAUS:
      // inca nu am nevoie
      break;
  }  

  portENTER_CRITICAL(&muxVarG);
  conectareStatie = varStareLocal;
  portEXIT_CRITICAL(&muxVarG); 
   
}

// void loop() {
//   Serial.println("Loop activ");
//   delay(1000); // temporizare pentru debugging
// }
