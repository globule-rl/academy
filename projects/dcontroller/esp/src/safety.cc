#include <pin.h>

float readCurrent(int pin) {
  int raw = analogRead(pin);
  float voltage = (raw / 4095.0) * 3.3;
  return (voltage - 2.5) / 0.066; // ACS712 30A example
}

void safetyCheck() {
  float i1 = readCurrent(CUR_M1);
  float i2 = readCurrent(CUR_M2);

  if (abs(i1) > CURRENT_LIMIT || abs(i2) > CURRENT_LIMIT) {
    stopMotors();
  }

  if (motion != STOPPED && millis() - lastHallTime > STALL_TIMEOUT) {
    stopMotors();
  }
}