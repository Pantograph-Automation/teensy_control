#pragma once

class ClockInterface {
  public:
    virtual ~ClockInterface() = default;
    virtual unsigned long milliseconds() = 0;
    virtual unsigned long microseconds() = 0;
    virtual void sleep(const unsigned long microseconds) = 0;
};