#ifndef MOTOR_H
#define MOTOR_H

#include <Wire.h>

namespace Motor {
  enum class Direction {Forward, Reverse, Stop};

  class DCMotor {
    static const unsigned int PWM_FREQ = 2000;
    static const unsigned int PWM_MAX = 1024;

    const PinName FORWARD_PIN;
    const PinName REVERSE_PIN;
    unsigned int pwmOutput;
    const unsigned int MOVEMENT_MIN_PWM;

    Direction direction;

    void updatePWM();

  public:
    DCMotor(PinName forwardPin, PinName reversePin, unsigned int minPWMToMove=0);

    double getSpeed();
    Direction getDirection();

    void forward();
    void reverse();
    void stop();
    void switchDirection();

    void setSpeed(double speed);
    void setDirection(Direction dir);
    void setSpeedAndDirection(double speed, Direction dir);

    unsigned int getPWMOutput();
    void setPWMOutput(unsigned int dutyCycle);
  };
}

#endif
