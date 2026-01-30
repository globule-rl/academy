#include <pin.h>

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