#include <Preferences.h>

Preferences prefs;

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Height:");
  display.setCursor(0, 30);
  display.print(heightMM, 1);
  display.print("mm");
  display.display();
}

void handleButtons() {
  if (!digitalRead(BTN_UP)) moveUp();
  else if (!digitalRead(BTN_DOWN)) moveDown();
  else stopMotors();

  if (!digitalRead(BTN_M1)) {
    prefs.putFloat(PREF_H1, heightMM);
    delay(300);
  }

  if (!digitalRead(BTN_M2)) {
    prefs.putFloat(PREF_H2, heightMM);
    delay(300);
  }
}