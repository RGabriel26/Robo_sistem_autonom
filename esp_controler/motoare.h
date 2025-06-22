#ifndef MOTOARE_H
#define MOTOARE_H

#include <Arduino.h>

// ========================
// DEFINIRE PINI MOTOARE
// ========================

// pini roti față
#define DRIVER_A_IN1 27
#define DRIVER_A_IN2 14
#define DRIVER_A_IN3 12
#define DRIVER_A_IN4 13

// pini roti spate
#define DRIVER_B_IN1 32
#define DRIVER_B_IN2 33
#define DRIVER_B_IN3 25
#define DRIVER_B_IN4 26

// pin pentru control PWM general
#define pinPWM 16

/**
 * @brief Deplasare înainte a robotului.
 * 
 * Setează pinii driverelor pentru a roti toate roțile în direcția înainte.
 */
void motoare_deplasareFata() {
  digitalWrite(pinPWM, HIGH);
  digitalWrite(DRIVER_A_IN1, HIGH);
  digitalWrite(DRIVER_A_IN2, LOW);
  digitalWrite(DRIVER_A_IN3, HIGH);
  digitalWrite(DRIVER_A_IN4, LOW);
  digitalWrite(DRIVER_B_IN1, HIGH);
  digitalWrite(DRIVER_B_IN2, LOW);
  digitalWrite(DRIVER_B_IN3, HIGH);
  digitalWrite(DRIVER_B_IN4, LOW);
}

/**
 * @brief Deplasare înapoi a robotului.
 * 
 * Inversează sensul de rotație al roților pentru mers înapoi.
 */
void motoare_deplasareSpate() {
  digitalWrite(pinPWM, HIGH);
  digitalWrite(DRIVER_A_IN1, LOW);
  digitalWrite(DRIVER_A_IN2, HIGH);
  digitalWrite(DRIVER_A_IN3, LOW);
  digitalWrite(DRIVER_A_IN4, HIGH);
  digitalWrite(DRIVER_B_IN1, LOW);
  digitalWrite(DRIVER_B_IN2, HIGH);
  digitalWrite(DRIVER_B_IN3, LOW);
  digitalWrite(DRIVER_B_IN4, HIGH);
}

/**
 * @brief Rotație pe loc spre stânga.
 * 
 * Controlează motoarele pentru a executa o întoarcere pe loc spre stânga.
 */
void motoare_rotireStanga() {
  digitalWrite(pinPWM, HIGH);
  digitalWrite(DRIVER_A_IN1, LOW);
  digitalWrite(DRIVER_A_IN2, HIGH);
  digitalWrite(DRIVER_A_IN3, HIGH);
  digitalWrite(DRIVER_A_IN4, LOW);
  digitalWrite(DRIVER_B_IN1, HIGH);
  digitalWrite(DRIVER_B_IN2, LOW);
  digitalWrite(DRIVER_B_IN3, LOW);
  digitalWrite(DRIVER_B_IN4, HIGH);
}

/**
 * @brief Rotație pe loc spre dreapta.
 * 
 * Controlează motoarele pentru a executa o întoarcere pe loc spre dreapta.
 */
void motoare_rotireDreapta() {
  digitalWrite(pinPWM, HIGH);
  digitalWrite(DRIVER_A_IN1, HIGH);
  digitalWrite(DRIVER_A_IN2, LOW);
  digitalWrite(DRIVER_A_IN3, LOW);
  digitalWrite(DRIVER_A_IN4, HIGH);
  digitalWrite(DRIVER_B_IN1, LOW);
  digitalWrite(DRIVER_B_IN2, HIGH);
  digitalWrite(DRIVER_B_IN3, HIGH);
  digitalWrite(DRIVER_B_IN4, LOW);
}

/**
 * @brief Deplasare laterală spre stânga (cu roți magnum).
 * 
 * Activează toate motoarele pentru a realiza o mișcare laterală stânga.
 */
void motoare_verticalStanga() {
  digitalWrite(pinPWM, HIGH);
  digitalWrite(DRIVER_A_IN1, LOW);
  digitalWrite(DRIVER_A_IN2, HIGH);
  digitalWrite(DRIVER_A_IN3, HIGH);
  digitalWrite(DRIVER_A_IN4, LOW);
  digitalWrite(DRIVER_B_IN1, LOW);
  digitalWrite(DRIVER_B_IN2, HIGH);
  digitalWrite(DRIVER_B_IN3, HIGH);
  digitalWrite(DRIVER_B_IN4, LOW);
}

/**
 * @brief Deplasare laterală spre dreapta (cu roți magnum).
 * 
 * Activează toate motoarele pentru a realiza o mișcare laterală dreapta.
 */
void motoare_verticalDreapta() {
  digitalWrite(pinPWM, HIGH);
  digitalWrite(DRIVER_A_IN1, HIGH);
  digitalWrite(DRIVER_A_IN2, LOW);
  digitalWrite(DRIVER_A_IN3, LOW);
  digitalWrite(DRIVER_A_IN4, HIGH);
  digitalWrite(DRIVER_B_IN1, HIGH);
  digitalWrite(DRIVER_B_IN2, LOW);
  digitalWrite(DRIVER_B_IN3, LOW);
  digitalWrite(DRIVER_B_IN4, HIGH);
}

/**
 * @brief Oprirea completă a tuturor motoarelor.
 * 
 * Setează toți pinii de control în LOW pentru a opri motorul.
 */
void motoare_stop() {
  digitalWrite(pinPWM, LOW);
  digitalWrite(DRIVER_A_IN1, LOW);
  digitalWrite(DRIVER_A_IN2, LOW);
  digitalWrite(DRIVER_A_IN3, LOW);
  digitalWrite(DRIVER_A_IN4, LOW);
  digitalWrite(DRIVER_B_IN1, LOW);
  digitalWrite(DRIVER_B_IN2, LOW);
  digitalWrite(DRIVER_B_IN3, LOW);
  digitalWrite(DRIVER_B_IN4, LOW);
}

/**
 * @brief Execută manevra de retragere după eliberarea sau prinderea obiectului.
 * 
 * Include mers înapoi și rotație pentru a ieși din stație. Se folosește `vTaskDelay` din FreeRTOS.
 */
void motoare_executieRetragere(){
  digitalWrite(pinPWM, HIGH);
  motoare_deplasareSpate();
  vTaskDelay(500);
  motoare_rotireDreapta();
  vTaskDelay(1000);
  motoare_stop();
}

#endif
