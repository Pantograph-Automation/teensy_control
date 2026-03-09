#pragma once

class StepperInterface {
public:
    virtual ~StepperInterface() = default;
    virtual void begin() = 0;
    virtual void set_direction_forward() = 0;
    virtual void set_direction_backward() = 0;
    virtual void set_high() = 0;
    virtual void set_low() = 0;
};
