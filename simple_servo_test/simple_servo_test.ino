#include <Servo.h>

const int SERVO1_PIN = 2;
const int SERVO2_PIN = 3;

const int SERVO_DOWN_DEG = 70;
const int SERVO_NEUTRAL_DEG = 90;
const int SERVO_UP_DEG = 120;

Servo servo1;
Servo servo2;

void writeBoth(int angle) {
  servo1.write(angle);
  servo2.write(angle);
}

void setup() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  writeBoth(SERVO_NEUTRAL_DEG);
  delay(1000);

  writeBoth(SERVO_DOWN_DEG);
  delay(1000);

  writeBoth(SERVO_NEUTRAL_DEG);
  delay(1000);

  writeBoth(SERVO_UP_DEG);
  delay(1000);

  writeBoth(SERVO_NEUTRAL_DEG);
}

void loop() {
}
