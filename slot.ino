#include "slot.h"

void setup_slot(uint8_t PIN) {
  pinMode(PIN, INPUT);
}

// returns true if blocked, false otherwise
bool read_slot(uint8_t PIN) {
  int slot_voltage = analogRead(PIN);
  return (slot_voltage < (1023 / 2)) ? true : false;
}