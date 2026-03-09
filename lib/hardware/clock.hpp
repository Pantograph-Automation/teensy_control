#pragma once

#include "Arduino.h"
#include "clock_interface.hpp"

class Clock : public ClockInterface {
  public:
    Clock() {};

    inline unsigned long milliseconds() override { return millis(); }

    inline unsigned long microseconds() override { return micros(); }

};