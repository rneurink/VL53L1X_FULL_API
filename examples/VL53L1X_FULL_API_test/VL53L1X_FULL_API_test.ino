/**
 * This example contains a full API test
 * 
 */

#include "VL53L1X_FULL.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  while (!Serial) {
    delay(1);
  }
  #if defined(__AVR_ATtinyx17__)
  attachInterrupt(digitalPinToInterrupt(17), sw_reset, FALLING);
  #endif

  delay(5000);
}

void loop() {

}

#if defined(__AVR_ATtinyx17__)
void sw_reset() {
  _PROTECTED_WRITE(RSTCTRL.SWRR,1);
}
  #endif
