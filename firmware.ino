#include <Arduino.h>

// From config.h: PWM Frequency, Bits (10), Max PWM (1023 for 10 bits)
#define PWM_FREQUENCY 20000
#define PWM_BITS 10
#define PWM_MAX (1 << PWM_BITS) - 1  // 1023

// Motor Pins from config.h (BTS7960: IN_A=RPWM, IN_B=LPWM)
#define MOTOR1_IN_A 9
#define MOTOR1_IN_B 10
#define MOTOR2_IN_A 13
#define MOTOR2_IN_B 14
#define MOTOR3_IN_A 4
#define MOTOR3_IN_B 5
#define MOTOR4_IN_A 15
#define MOTOR4_IN_B 16

// Static channel counter for LEDC uniqueness
int next_pwm_channel = 0;

// Function to setup and spin a motor forward at given PWM (0-1023)
void setupMotor(int in_a_pin, int in_b_pin, int &channel_a, int &channel_b, int pwm_value) {
  if (in_a_pin < 0) return;
  channel_a = next_pwm_channel++;
  channel_b = next_pwm_channel++;
  ledcSetup(channel_a, PWM_FREQUENCY, PWM_BITS);
  ledcSetup(channel_b, PWM_FREQUENCY, PWM_BITS);
  ledcAttachPin(in_a_pin, channel_a);
  ledcAttachPin(in_b_pin, channel_b);
  // Forward spin: Set RPWM to PWM_MAX - pwm_value, LPWM to PWM_MAX (adapted from default_motor.h)
  ledcWrite(channel_a, PWM_MAX - pwm_value);
  ledcWrite(channel_b, PWM_MAX);
  Serial.printf("Motor Setup: IN_A=%d (Ch %d, Duty %d), IN_B=%d (Ch %d, Duty %d)\n",
                in_a_pin, channel_a, PWM_MAX - pwm_value, in_b_pin, channel_b, PWM_MAX);
}

void setup() {
  
}

void loop() {
  
}