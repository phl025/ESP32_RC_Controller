//
// Outputs.cpp : Outputs command
//
// Author : PHL @2025
//

#include "Outputs.h"

// Class definition (methodes)

/// @brief Construct a new Output
Outputs::Outputs(void) // Constructor (called, when new ojects of that class are created)
{
	for (int i = 0; i < outputCount_; i++)
	{
#ifdef ESC3_DRIVER
		if (pins_[i] == 15) // Desable output '15' if ESC3_DRIVER defined, Pin is used for STEERING servo drive
			pins_[i] = 0;
#endif
		if (pins_[i] > 0)
		{
			// Check pin
			gpio_error_ = gpio_set_direction((gpio_num_t)pins_[i], GPIO_MODE_OUTPUT);
			if (gpio_error_ == ESP_OK)
			{
				//channel[i] = statusLED(false);
				channel[i] = OutputLED(false);
				channel[i].begin(pins_[i], i + 2, 20000); // Timers 2 to 14
				channel[i].off(0, 0);
			}
		}
	}
}

/*
void outputLED::begin(int pin1, int channel, int frequency, int resolution) {
    _pin1 = pin1;
    _channel = channel;
    _frequency = frequency;
    ledcSetup(_channel, _frequency, resolution); // 8 bit resolution, if no input argument
    ledcAttachPin(_pin1, _channel);
    if (_inverse) ledcWrite(_channel, 255);
    else ledcWrite(_channel, 0);
    // Save current value (Add by PHL025)
    _brightness = 0;
}
*/
//
// EOF
//