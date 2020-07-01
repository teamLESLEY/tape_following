#ifndef TAPE_HPP
#define TAPE_HPP

#include <Wire.h>

class TapeSensor {
  const PinName LEFT_SENSOR;
  const PinName RIGHT_SENSOR;

  unsigned int onThreshold;

  // Common setup between constructors
  void setup(unsigned int threshold);

public:
  TapeSensor(PinName leftPin, PinName rightPin, unsigned int threshold);
  TapeSensor(uint32_t leftPin, uint32_t rightPin, unsigned int threshold);
  TapeSensor(PinName leftPin, uint32_t rightPin, unsigned int threshold);
  TapeSensor(uint32_t leftPin, PinName rightPin, unsigned int threshold);

  unsigned int getLeftReading();
  bool isLeftOn();
  unsigned int getRightReading();
  bool isRightOn();

  unsigned int getThreshold();
  void setThreshold(unsigned int threshold);
};

#endif
