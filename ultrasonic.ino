#include "ultrasonic.h"
void setup_ultrasonic(uint8_t PIN) {
  pinMode(PIN, INPUT);
}

// Returns ultrasonic distance in cm
// Input PW -> PulseWidth Input
// Output -> Distance in cm
float read_ultrasonic(uint8_t PIN) {
  float voltage = analogRead(PIN) * 5.0 / 1023.0; // returns between 0 and 1023 (5 V / 1024 = 4.9 mV resolution)
  //Serial.println(analogRead(PIN));
  float distance = voltage / 0.0049; // 4.9 mV/cm with 5 V power supply (1 cm accuracy) (can use meters or inches if we want)
  return distance;
}