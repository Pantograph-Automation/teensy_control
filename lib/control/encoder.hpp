#pragma once

#if defined(ARDUINO)
#include <AS5600.h>

class Encoder
{
  
  public:
    Encoder(TwoWire *wire = &Wire) {
      this->wire = wire;
      this->as5600 = AS5600(wire);
    };

    /**
     * @brief Initialize the encoder
     * @param timeout How long to wait (milliseconds) for encoder connection before returning
     */
    inline void begin()
    {
      wire->begin();
      wire->setClock(400000);
      as5600.begin();
      as5600.setDirection(AS5600_CLOCK_WISE);
    }
    
    /**
     * @brief Returns the encoder value, in radians
     */
    inline float read_angle()
    {
      return as5600.rawAngle() * AS5600_RAW_TO_RADIANS;
    };

    /**
     * @brief Sample the average of a set of readings for better accuracy
     */
    inline float sample(const unsigned int num, uint32_t delay = 20)
    {
      float reading = 0.0f;
      for (unsigned int i = 0; i < num; ++i) {
        reading += read_angle();
        delayMicroseconds(delay);
      }
      return reading / (float) num;
    };

  private:

    /** @brief I2C wire interface to use for this encoder */
    TwoWire* wire;

    /** @brief Hardware-dependant encoder object */
    AS5600 as5600;

};
#endif