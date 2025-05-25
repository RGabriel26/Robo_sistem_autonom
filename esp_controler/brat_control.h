#ifndef BRAT_CONTROL_H
#define BRAT_CONTROL_H

#include <ESP32Servo.h>

// definire pini
#define pinServoBase 23
#define pinServoHead 19

// setari de functionare
#define DELAY_MOVE_ARM 200
#define POZ_SERVO_BASE_DEFAULT 35
#define POZ_SERVO_HEAD_DEFAULT 20
#define POZ_SERVO_BASE_RIDICAT 17
#define POZ_SERVO_HEAD_RIDICAT 60

// variabile externe - definite în .ino
extern Servo servoBase;
extern Servo servoHead;

void init_brat() {
  servoBase.write(POZ_SERVO_BASE_DEFAULT);
  servoHead.write(POZ_SERVO_HEAD_DEFAULT);
}

void brat_prindere() {
  servoHead.write(POZ_SERVO_HEAD_RIDICAT);
  vTaskDelay(DELAY_MOVE_ARM);
  servoBase.write(POZ_SERVO_BASE_RIDICAT);
}

void brat_eliberare() {
  servoBase.write(POZ_SERVO_BASE_DEFAULT);
  vTaskDelay(DELAY_MOVE_ARM);
  servoHead.write(POZ_SERVO_HEAD_DEFAULT);
}

#endif
