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


#ifndef ADC_H
#define ADC_H

#include "system.h"

/**
 * @brief A driver for the analog-digital converters available on the microcontroller.
 * @details Besides initialization, it is most convenient to work with the ADC with the
 * AnalogPin class, which handles adding channels and reading results.
 */
class AnalogDigitalConverter
{
public:
  /**
   * @brief Initializes the ADC according to the definition struct provided
   * @param A hardware struct defining the resources for the ADC
   */
  void init(const ADCHardwareStruct *adc_def);
  /**
   * @brief Adds a channel to the list that is scanned.
   * @details This returns the index assigned to the channel, which is needed to read it later
   * This method does not check for errors. Do not add more than CHANNEL_COUNT channels,
   * by calling add_channel more than CHANNEL_COUNT times
   * @param channel The index of the channel to be read
   * @return The index of the added channel
   */
  uint8_t add_channel(uint8_t channel);
  // Returns a value between 0 and RAW_READING_MAX. 0 represents 0V, and RAW_READING_MAX
  // represents REFERENCE_VOLTAGE
  // The parameter, index, is the return value from add_channel when the channel was added
  /**
   * @brief Reads a single channel.
   * @details The reading is between 0 and RAW_READING_MAX. 0 represents 0V, and
   * RAW_READING_MAX represents REFERENCE_VOLTAGE (typically 3.3V).
   * Takes the index of the channel as a parameter, which is returned from add_channel
   * @param index The index of the channel to be read.
   * @return
   */
  uint16_t read(uint8_t index) const;
  /**
   * @brief Checks if the adc has been initialized, i.e. init has been called.
   * @return if the adc has been initialized
   */
  bool is_initialized() const;
  /**
   * @brief Checks the number of used channels. If this exceeds CHANNEL_COUNT, stuff will break
   * @return The number of ADC channels in use for this ADC
   */
  uint8_t get_current_channel_count() const;

  /**
   * The reference voltage used for the ADC, which is hardware-dependent.
   * Unit is volts.
   */
  static constexpr double REFERENCE_VOLTAGE{3.3};
  /**
   * The maximum reading from a channel. This reading corresponds to REFERENCE_VOLTAGE volts.
   */
  static constexpr uint16_t RAW_READING_MAX{0xFFF};
  /**
   * A value indicating that the channel value has not yet been read by the ADC.
   * If you get this for more than just the startup time, your channel may not be initialized
   */
  static constexpr uint16_t NO_READING{0xFFFF};
  /**
   * The maximum number of channels on a single ADC. Hardware-dependent
   */
  static constexpr uint8_t CHANNEL_COUNT{16};

private:
  const ADCHardwareStruct *adc_def_;
  bool is_initialized_{false};
  uint8_t current_channels;

  // These two constants are used for adjusting the ADC configuration when a channel is added
  static constexpr uint32_t SQR1_CHANNEL_COUNT_MASK{~0xFF0FFFFF};
  static constexpr uint8_t SQR1_CHANNEL_COUNT_OFFSET{20};

  volatile uint32_t buffer[CHANNEL_COUNT];

  void init_dma();
  void start_dma();
};

#endif // ADC_H
