#pragma once

class EncoderInterface {
public:
    virtual ~EncoderInterface() = default;
    virtual void begin() = 0;
    virtual float read_angle() = 0;
    virtual float read_angle(const float alpha) = 0;
    virtual float sample(const unsigned int num) = 0;
};

template <typename T>
inline T exponential_moving_average(const T output, const T current, const float alpha) {
  return (1.0 - alpha) * current + alpha * output;
}