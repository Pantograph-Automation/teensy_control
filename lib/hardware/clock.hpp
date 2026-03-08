#pragma once

#include "Arduino.h"
#include "clock_interface.hpp"

class Clock : public ClockInterface {
  public:
    Clock() {};

    inline unsigned long microseconds() override { return micros(); }

    inline unsigned long microseconds() override { return millis(); }

};