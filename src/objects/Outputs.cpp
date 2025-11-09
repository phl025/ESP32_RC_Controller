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
				channel[i] = statusLED(false);
				channel[i].begin(pins_[i], i + 2, 20000); // Timers 2 to 14
				channel[i].off(0, 0);
			}
		}
	}
}

//
// EOF
//