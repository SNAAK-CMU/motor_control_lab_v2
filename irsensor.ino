#include <IRremote.h>
#include "irsensor.h"
// #define IR_RECEIVE_PIN 7

// TODO: need to be able to send commands
void setup_irsensor(uint8_t PIN) {
  // Serial.begin(9600);
  IrReceiver.begin(PIN);
}

int read_irsensor(uint8_t PIN) {
  if (IrReceiver.decode()) {
    IrReceiver.resume();
    return (IrReceiver.decodedIRData.command);
  }
}



