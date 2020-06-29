#include <Wire.h>

#include "motor.hpp"

using namespace Motor;

DCMotor::DCMotor(PinName forwardPin, PinName reversePin)
  : FORWARD_PIN(forwardPin), REVERSE_PIN(reversePin) {
  pwmOutput = 0;
  direction = Direction::Stop;

  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(REVERSE_PIN, OUTPUT);
  updatePWM();
}

void DCMotor::updatePWM() {
  int forwardPWM = 0;
  int reversePWM = 0;

  if (direction == Direction::Forward) {
    forwardPWM = pwmOutput;
  } else if (direction == Direction::Reverse) {
    reversePWM = pwmOutput;
  }

  pwm_start(FORWARD_PIN, PWM_FREQ, forwardPWM, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(REVERSE_PIN, PWM_FREQ, reversePWM, RESOLUTION_10B_COMPARE_FORMAT);
}

double DCMotor::getSpeed() {
  return (double) pwmOutput / PWM_MAX;
}

Direction DCMotor::getDirection() {
  return direction;

}

void DCMotor::forward() {
  setDirection(Direction::Forward);
}

void DCMotor::reverse() {
  setDirection(Direction::Reverse);
}

void DCMotor::stop() {
  setDirection(Direction::Stop);
}

void DCMotor::switchDirection() {
  if (direction == Direction::Forward) {
    direction = Direction::Reverse;
  } else if (direction == Direction::Reverse) {
    direction = Direction::Forward;
  }
}

void DCMotor::setSpeed(double speed) {
  if (0 <= speed && speed <= 1) {
    pwmOutput = speed * PWM_MAX;
    updatePWM();
  }
}

void DCMotor::setDirection(Direction dir) {
  direction = dir;
  updatePWM();
}

void DCMotor::setSpeedAndDirection(double speed, Direction dir) {
  setSpeed(speed);
  setDirection(dir);
}

unsigned int DCMotor::getPWMOutput() {
  return pwmOutput;
}

void DCMotor::setPWMOutput(unsigned int dutyCycle) {
  if (dutyCycle <= PWM_MAX) {
    pwmOutput = dutyCycle;
    updatePWM();
  }
}
