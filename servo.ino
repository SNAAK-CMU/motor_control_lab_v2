#include "servo.h"
#include <Servo.h>

Servo servo_1; 

void setup_servo(uint8_t PIN) {
  servo_1.attach(PIN);
}

// ANGLE from 0 to 270
void set_servo_angle(int ANGLE) {
  int servo_angle = map(ANGLE, 0, 270, 0, 255);
  servo_1.write(servo_angle);
  delay(15);  
}