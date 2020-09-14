/*
 * Copyright (c) 2017, James Jackson
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

#include "system.h"
#include "pwm.h"
#include "led.h"

#include "revo_f4.h"

int main() {
	systemInit();

	LED info;
	info.init(LED2_GPIO, LED2_PIN);

	PWM_OUT esc_out[PWM_NUM_OUTPUTS];
	for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
	{
		esc_out[i].init(&pwm_config[i], 490, 2000, 1000);
		esc_out[i].write(1.0);
	}

	// Calibrate ESC
	while (millis() < 5000);

	for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
	{
		esc_out[i].write(0.0);
	}

	while (millis() < 1000);


	bool use_us_driver = true;;
	uint32_t throttle = 1000;
	while(1)
	{
		for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
		{
			if (use_us_driver)
			{
				esc_out[i].writeUs(throttle);
			}
			else
			{
				esc_out[i].write((float)(throttle - 1000) / 1000.0);
			}
		}
		throttle += 1;
		if (throttle > 2000)
		{
			throttle = 0;
			info.toggle();
			use_us_driver = !use_us_driver;
		}
		delay(2);
	}
}
