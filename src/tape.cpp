#include <Wire.h>

#include "tape.hpp"

TapeSensor::TapeSensor(PinName leftPin, PinName rightPin, unsigned int threshold)
  : LEFT_SENSOR(leftPin), RIGHT_SENSOR(rightPin) {
  setup(threshold);
}

TapeSensor::TapeSensor(uint32_t leftPin, uint32_t rightPin, unsigned int threshold)
  : LEFT_SENSOR(digitalPinToPinName(leftPin)), RIGHT_SENSOR(digitalPinToPinName(rightPin)) {
  setup(threshold);
}

TapeSensor::TapeSensor(PinName leftPin, uint32_t rightPin, unsigned int threshold)
  : LEFT_SENSOR(leftPin), RIGHT_SENSOR(digitalPinToPinName(rightPin)) {
  setup(threshold);
}

TapeSensor::TapeSensor(uint32_t leftPin, PinName rightPin, unsigned int threshold)
  : LEFT_SENSOR(digitalPinToPinName(leftPin)), RIGHT_SENSOR(rightPin) {
  setup(threshold);
}

void TapeSensor::setup(unsigned int threshold) {
  onThreshold = threshold;

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
}

unsigned int TapeSensor::getLeftReading() {
  return analogRead(LEFT_SENSOR);
}

bool TapeSensor::isLeftOn() {
  return getLeftReading() > onThreshold;
}

unsigned int TapeSensor::getRightReading() {
  return analogRead(RIGHT_SENSOR);
}

bool TapeSensor::isRightOn() {
  return getRightReading() > onThreshold;
}

unsigned int TapeSensor::getThreshold() {
  return onThreshold;
}

void TapeSensor::setThreshold(unsigned int threshold) {
  onThreshold = threshold;
}
