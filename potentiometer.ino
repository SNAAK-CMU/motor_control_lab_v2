#include "potentiometer.h"

void setup_potentiometer(uint8_t PIN) {
  pinMode(PIN, INPUT);
}

// Input -> Voltage Range from 0V to 5V
// Output -> Value from 0 to 10
float read_potentiometer(uint8_t PIN) {
  float val = analogRead(PIN);
  //Serial.println(val);

  return val;
}