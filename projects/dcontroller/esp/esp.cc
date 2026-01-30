#include <esp.h>

#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

Preferences prefs;
Adafruit_SSD1306 display(128, 64, &Wire);

// Hall counters
volatile long hallCountM1 = 0;
volatile long hallCountM2 = 0;

// Height
float heightMM = 0;

// Timing
unsigned long lastHallTime = 0;

// Motion state
enum Motion { STOPPED, MOVING_UP, MOVING_DOWN };
Motion motion = STOPPED;

// fast ram storage, pulse cnt, isr sensor interrupt handler
void IRAM_ATTR hallM1ISR() {
  hallCountM1++;
  lastHallTime = millis();
}

void IRAM_ATTR hallM2ISR() {
  hallCountM2++;
  lastHallTime = millis();
}

void stopMotors() {
  ledcWrite(PWM_CH_M1, 0);
  ledcWrite(PWM_CH_M2, 0);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  motion = STOPPED;
}

void moveUp() {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
  ledcWrite(PWM_CH_M1, MOTOR_SPEED);
  ledcWrite(PWM_CH_M2, MOTOR_SPEED);
  motion = MOVING_UP;
}

void moveDown() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
  ledcWrite(PWM_CH_M1, MOTOR_SPEED);
  ledcWrite(PWM_CH_M2, MOTOR_SPEED);
  motion = MOVING_DOWN;
}

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

void updateHeight() {
  long avgPulses = (hallCountM1 + hallCountM2) / 2;
  heightMM = avgPulses / PULSES_PER_MM;

  if (heightMM > MAX_HEIGHT_MM) {
    heightMM = MAX_HEIGHT_MM;
    stopMotors();
  }
  if (heightMM < MIN_HEIGHT_MM) {
    heightMM = MIN_HEIGHT_MM;
    stopMotors();
  }
}

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

void setup() {
  Serial.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_M1, INPUT_PULLUP);
  pinMode(BTN_M2, INPUT_PULLUP);

  ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_IN1, PWM_CH_M1);
  ledcAttachPin(M2_IN1, PWM_CH_M2);

  attachInterrupt(HALL_M1, hallM1ISR, RISING);
  attachInterrupt(HALL_M2, hallM2ISR, RISING);

  prefs.begin(PREF_NAMESPACE, false);

  Wire.begin(SDA_PIN, SCL_PIN);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  handleButtons();
  updateHeight();
  safetyCheck();
  updateDisplay();
  delay(50);
}
