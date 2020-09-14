/*
 * Copyright (c) 2020, Richard Henrichsen
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ANALOG_H
#define ANALOG_H

#include "system.h"
#include "gpio.h"
#include "analog_digital_converter.h"

/**
 * @brief A class to represent a single pin configured for analog input
 * @details This class manages the connection to the ADC, and does some of the math for reading.
 * This class does not initialize the ADC.
 */
class AnalogPin
{
public:
  /**
   * @brief Initializes a pin to read analog values.
   * @details This initialization includes configuring the GPIO, and adding a channel to the ADC.
   * This does not configure the ADC. ADC configuration must happen before
   * @param adc The ADC this pin is to use
   * @param basePort The GPIO definition for this pin to use
   * @param pin The GPIO number for the pin
   * @param adc_channel The ADC channel number for the pin, defined by the hardware
   */
  void init(AnalogDigitalConverter *adc, GPIO_TypeDef *basePort, uint16_t pin, uint8_t adc_channel);
  /**
   * @brief Get the last reading of the pin, in volts
   * @return The most recent reading, in volts
   */
  double read() const; // Returns a reading in volts
  /**
   * @brief Get the most recent reading of the pin, directly from the ADC.
   * See AnalogDigitalConverter.read for the details on interpreting this value.
   * @return The most recent raw reading
   */
  uint16_t read_raw() const;

private:
  GPIO gpio_;
  AnalogDigitalConverter *adc_;
  uint8_t index_;
};

#endif // ANALOG_H
