//
// Output.cpp : Output command
//
// Author : PHL @2025
//
// A Voir :
// https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32h2/api-reference/peripherals/ledc.html

#include "Output.h"

// Class definition (methodes)

/// @brief Construct a new Output
Output::Output(int8_t pin) // Constructor (called, when new ojects of that class are created)
{
	if (GPIO_IS_VALID_OUTPUT_GPIO(pin))
	{
		gpio_num_ = (gpio_num_t)pin;
		// pinMode(pin_, OUTPUT);		    // INPUT, INPUT_PULLDOWN, INPUT_PULLUP, OUTPUT
		// gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
		gpio_error_ = gpio_set_direction((gpio_num_t)gpio_num_, GPIO_MODE_OUTPUT);
		if (gpio_error_ == ESP_OK)
		{
			gpio_ok_ = true;
			set();
		}
	}
	else
	{
		gpio_error_ = -1; // NOK
		gpio_ok_ = false;
	}
}

/// @brief Set output to state value
void Output::set(bool state_cmd)
{
	// if (!gpio_ok_) return;
	//
	if (state_cmd)
		gpio_error_ = gpio_set_level(gpio_num_, 1);
	else
		gpio_error_ = gpio_set_level(gpio_num_, 0);
	//
	state = (gpio_get_level(gpio_num_) == 1);
}

/// @brief Set output (to 1)
void Output::set()
{
	// if (!gpio_ok_) return;
	//
	gpio_error_ = gpio_set_level(gpio_num_, 1);
	//
	state = (gpio_get_level(gpio_num_) == 1);
}

/// @brief Reset output (to 0)
void Output::reset()
{
	// if (!gpio_ok_) return;
	//
	gpio_error_ = gpio_set_level(gpio_num_, 0);
	//
	state = (gpio_get_level(gpio_num_) == 1);
}

/// @brief Get output value (0, 1 or -1)
int8_t Output::get()
{
	// gpio_get_level(gpio_num_);
	if (gpio_error_ == ESP_OK)
		return gpio_get_level(gpio_num_);
	else
		return gpio_error_;
}

//
// EOF
//