#pragma once

class StepperInterface {
public:
    virtual ~StepperInterface() {}
    virtual void begin() = 0;
    virtual void set_direction_forward() = 0;
    virtual void set_direction_backward() = 0;
    virtual void blocking_pulse(unsigned long pulse_width) = 0;
};
