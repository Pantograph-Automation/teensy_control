#pragma once

#include <AS5600.h>
#include "encoder_interface.hpp"

class Encoder : public EncoderInterface
{
  /**
   * @param wire I2C wire interface to use for this encoder
   */
  public:
    Encoder(TwoWire *wire = &Wire) {
      this->wire = wire;
      this->as5600 = AS5600(wire);
    };

    /**
     * @brief Initialize the encoder
     * @param timeout How long to wait (milliseconds) for encoder connection before returning
     */
    inline void begin() override
    {
      wire->begin();
      wire->setClock(400000);
      as5600.begin();
      as5600.setDirection(AS5600_CLOCK_WISE);
    }
    
    /**
     * @brief Returns the encoder value, in radians
     */
    inline float read_angle() override
    {
      return as5600.rawAngle() * AS5600_RAW_TO_RADIANS;
    };

  private:
    TwoWire* wire;
    AS5600 as5600;

};
