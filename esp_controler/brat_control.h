#ifndef BRAT_CONTROL_H
#define BRAT_CONTROL_H

#include <ESP32Servo.h>

// ========================
// DEFINIRE PINI SERVOMOTOARE
// ========================

#define pinServoBase 23  ///< Pin pentru servo de la bază (ridicare/coborâre)
#define pinServoHead 16  ///< Pin pentru servo de la efector (apucare/eliberare)

// ========================
// POZIȚII ȘI TIMPI
// ========================

#define DELAY_MOVE_ARM 200                ///< Delay între mișcările bratului [ms]
#define POZ_SERVO_BASE_DEFAULT 35         ///< Poziție implicită a bazei (coborât)
#define POZ_SERVO_HEAD_DEFAULT 20         ///< Poziție implicită a efectorului (deschis)
#define POZ_SERVO_BASE_RIDICAT 17         ///< Poziție ridicată a bazei
#define POZ_SERVO_HEAD_RIDICAT 60         ///< Poziție închisă a efectorului (pentru prindere)

// ========================
// VARIABILE EXTERNE
// ========================

/// Servo pentru baza bratului - definit în sketch principal (.ino)
extern Servo servoBase;

/// Servo pentru capul bratului - definit în sketch principal (.ino)
extern Servo servoHead;

/**
 * @brief Inițializează brațul în poziția implicită.
 * 
 * Setează servomotoarele în pozițiile de așteptare:
 * - Baza coborâtă
 * - Capul deschis
 */
void init_brat() {
  servoBase.write(POZ_SERVO_BASE_DEFAULT);
  servoHead.write(POZ_SERVO_HEAD_DEFAULT);
}

/**
 * @brief Execută secvența de prindere a obiectului.
 * 
 * Mișcări implicate:
 * - Capul se închide pentru a apuca
 * - Baza se ridică după o întârziere
 */
void brat_prindere() {
  servoHead.write(POZ_SERVO_HEAD_RIDICAT);
  vTaskDelay(DELAY_MOVE_ARM);
  servoBase.write(POZ_SERVO_BASE_RIDICAT);
}

/**
 * @brief Execută secvența de eliberare a obiectului.
 * 
 * Mișcări implicate:
 * - Baza coboară
 * - Capul se deschide după o întârziere
 */
void brat_eliberare() {
  servoBase.write(POZ_SERVO_BASE_DEFAULT);
  vTaskDelay(DELAY_MOVE_ARM);
  servoHead.write(POZ_SERVO_HEAD_DEFAULT);
}

#endif
