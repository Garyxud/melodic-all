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


#include "battery_monitor.h"

void BatteryMonitor::init(const BatteryMonitorHardwareStruct &def, AnalogDigitalConverter *adc, float voltage_multiplier, float current_multiplier)
{
  voltage_pin_.init(adc, def.voltage_gpio, def.voltage_pin, def.voltage_adc_channel);
  current_pin_.init(adc, def.current_gpio, def.current_pin, def.current_adc_channel);
  voltage_multiplier_ = voltage_multiplier;
  current_multiplier_ = current_multiplier;
}
float BatteryMonitor::read_voltage() const
{
  return static_cast<float>(this->voltage_pin_.read()) * this->voltage_multiplier_;
}
float BatteryMonitor::read_current() const
{
  return static_cast<float>(this->current_pin_.read()) * this->current_multiplier_;
}

void BatteryMonitor::set_voltage_multiplier(double multiplier)
{
  this->voltage_multiplier_  = multiplier;
}

void BatteryMonitor::set_current_multiplier(double multiplier)
{
  this->current_multiplier_ = multiplier;
}

bool BatteryMonitor::has_voltage_sense() const
{
  return (this->voltage_multiplier_ != 0);
}

bool BatteryMonitor::has_current_sense() const
{
  return (this->current_multiplier_ != 0);
}
