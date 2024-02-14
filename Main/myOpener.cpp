#include <Arduino.h>
#include "myOpener.h"

#include <Servo.h>
const pin_size_t SERVO_1 = 21;
const pin_size_t SERVO_2 = 22;
Servo servo1;
Servo servo2;

void MY_OPENER::init() {
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  OPENER::init();
}

void MY_OPENER::open() {
  servo1.write(120);
  servo2.write(120);
}

void MY_OPENER::close() {
  servo1.write(30);
  servo2.write(30);
}
