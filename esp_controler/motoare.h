#ifndef MOTOARE_H
#define MOTOARE_H

#include <Arduino.h>

// definire pini
// pini roti fata
#define DRIVER_A_IN1 27
#define DRIVER_A_IN2 14
#define DRIVER_A_IN3 12
#define DRIVER_A_IN4 13

// pini roti spate
#define DRIVER_B_IN1 32
#define DRIVER_B_IN2 33
#define DRIVER_B_IN3 25
#define DRIVER_B_IN4 26

// pin pebtru PWM 
#define pinPWM 16 

void motoare_deplasareFata() {
  digitalWrite(DRIVER_A_IN1, HIGH);
  digitalWrite(DRIVER_A_IN2, LOW);
  digitalWrite(DRIVER_A_IN3, HIGH);
  digitalWrite(DRIVER_A_IN4, LOW);
  digitalWrite(DRIVER_B_IN1, HIGH);
  digitalWrite(DRIVER_B_IN2, LOW);
  digitalWrite(DRIVER_B_IN3, HIGH);
  digitalWrite(DRIVER_B_IN4, LOW);
}

void motoare_deplasareSpate() {
  digitalWrite(DRIVER_A_IN1, LOW);
  digitalWrite(DRIVER_A_IN2, HIGH);
  digitalWrite(DRIVER_A_IN3, LOW);
  digitalWrite(DRIVER_A_IN4, HIGH);
  digitalWrite(DRIVER_B_IN1, LOW);
  digitalWrite(DRIVER_B_IN2, HIGH);
  digitalWrite(DRIVER_B_IN3, LOW);
  digitalWrite(DRIVER_B_IN4, HIGH);
}

void motoare_rotireStanga() {
  digitalWrite(DRIVER_A_IN1, LOW);
  digitalWrite(DRIVER_A_IN2, HIGH);
  digitalWrite(DRIVER_A_IN3, HIGH);
  digitalWrite(DRIVER_A_IN4, LOW);
  digitalWrite(DRIVER_B_IN1, HIGH);
  digitalWrite(DRIVER_B_IN2, LOW);
  digitalWrite(DRIVER_B_IN3, LOW);
  digitalWrite(DRIVER_B_IN4, HIGH);
}

void motoare_rotireDreapta() {
  digitalWrite(DRIVER_A_IN1, HIGH);
  digitalWrite(DRIVER_A_IN2, LOW);
  digitalWrite(DRIVER_A_IN3, LOW);
  digitalWrite(DRIVER_A_IN4, HIGH);
  digitalWrite(DRIVER_B_IN1, LOW);
  digitalWrite(DRIVER_B_IN2, HIGH);
  digitalWrite(DRIVER_B_IN3, HIGH);
  digitalWrite(DRIVER_B_IN4, LOW);
}

void motoare_verticalStanga() {
  digitalWrite(DRIVER_A_IN1, LOW);
  digitalWrite(DRIVER_A_IN2, HIGH);
  digitalWrite(DRIVER_A_IN3, HIGH);
  digitalWrite(DRIVER_A_IN4, LOW);
  digitalWrite(DRIVER_B_IN1, LOW);
  digitalWrite(DRIVER_B_IN2, HIGH);
  digitalWrite(DRIVER_B_IN3, HIGH);
  digitalWrite(DRIVER_B_IN4, LOW);
}

void motoare_verticalDreapta() {
  digitalWrite(DRIVER_A_IN1, HIGH);
  digitalWrite(DRIVER_A_IN2, LOW);
  digitalWrite(DRIVER_A_IN3, LOW);
  digitalWrite(DRIVER_A_IN4, HIGH);
  digitalWrite(DRIVER_B_IN1, HIGH);
  digitalWrite(DRIVER_B_IN2, LOW);
  digitalWrite(DRIVER_B_IN3, LOW);
  digitalWrite(DRIVER_B_IN4, HIGH);
}

void motoare_stop() {
  digitalWrite(DRIVER_A_IN1, LOW);
  digitalWrite(DRIVER_A_IN2, LOW);
  digitalWrite(DRIVER_A_IN3, LOW);
  digitalWrite(DRIVER_A_IN4, LOW);
  digitalWrite(DRIVER_B_IN1, LOW);
  digitalWrite(DRIVER_B_IN2, LOW);
  digitalWrite(DRIVER_B_IN3, LOW);
  digitalWrite(DRIVER_B_IN4, LOW);
}

void motoare_executieRetragere(){
  digitalWrite(pinPWM, HIGH);
  motoare_deplasareSpate();
  vTaskDelay(500);
  motoare_rotireDreapta();
  vTaskDelay(1000);
  digitalWrite(pinPWM, LOW);
}

#endif
