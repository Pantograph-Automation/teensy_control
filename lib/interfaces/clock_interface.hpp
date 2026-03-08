#pragma once

class ClockInterface {
  public:
    virtual ~ClockInterface() {}
    virtual unsigned long milliseconds() = 0;
    virtual unsigned long microseconds() = 0;
};