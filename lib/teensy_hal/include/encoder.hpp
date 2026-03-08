#pragma once

#include <AS5600.h>
#include "hardware_interfaces.hpp"

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
    inline void begin(unsigned long timeout) override
    {
      wire->begin();
      wire->setClock(400000);
      as5600.begin(4); //FIXME: Use a non-wired pin
      as5600.setDirection(AS5600_CLOCK_WISE);

      unsigned long start_time = millis();
      while (!as5600.isConnected()) {
        if (millis() - start_time > timeout) { return; }
      }
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
