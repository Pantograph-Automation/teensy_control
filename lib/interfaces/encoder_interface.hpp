#pragma once

class EncoderInterface {
public:
    virtual ~EncoderInterface() = default;
    virtual void begin() = 0;
    virtual float read_angle() = 0;
};