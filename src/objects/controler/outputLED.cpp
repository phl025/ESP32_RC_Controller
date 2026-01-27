//
// OutputLED.cpp : Output LED object
//
// Object inspire by statusLED / TheDIYGuy999
// Author : PHL @2025

#include "OutputLED.h"
//
#include <Arduino.h>    // For micros() et ledc
//#include <esp32-hal-ledc.h>
//#include <esp32-hal-misc.c>

// Class definition (methodes)

/// @brief Construct a new OutputLED object
// Constructor (called, when new ojects of that class are created)
OutputLED::OutputLED(bool inverse) 
{
	// Flash reset
	_flashState = 0;
	_flashCnt = 0;
	_last_us = 0;
	_inverse = inverse;
};

/// @brief Begin LED functions
/// @param pin output pin
/// @param channel channel
/// @param frequency frequency (Hz)
/// @param resolution resolution (8 bits by default)
void OutputLED::begin(uint8_t pin, uint8_t channel, uint16_t frequency, uint8_t resolution) 
{
	_pin = pin;
	_channel = channel;
	_frequency = frequency;
	ledcSetup(_channel, _frequency, resolution); // 8 bit resolution, if no input argument
	ledcAttachPin(_pin, _channel);
	// Current brightness
	_brightness = 0;
	//
	if (_inverse) ledcWrite(_channel, 255 - _brightness);
	else ledcWrite(_channel, _brightness);
}

/// @brief PWM Function, Set brightness value
/// @param brightness Brightness set point [0..225]
void OutputLED::pwm(uint16_t brightness) 
{
	// Flash reset
	_flashState = 0;
	_flashCnt = 0;

	// Current brightness
	_brightness = brightness;
	_last_us = micros();
	// Set brightness 
	if (_inverse) ledcWrite(_channel, 255 - _brightness);
	else ledcWrite(_channel, _brightness);
}

/// @brief PWM function : Set brightness (with ramp up or ramp down)
/// @param brightness Brightness set point [0..225]
/// @param onRamp Ramp up (us), Ex.: 100 -> 256*100 -> 25600us -> 25.6ms 
/// @param offRamp Ramp down (us), Ex.: 100 -> 256*100 -> 25600us -> 25.6ms  
void OutputLED::pwm(uint16_t brightness, uint16_t onRamp, uint16_t offRamp) 
{
	uint32_t curUs = micros();
	// Flash reset
	_flashState = 0;
	_flashCnt = 0;

	// Update brightness
	_pwm(brightness, onRamp, offRamp);
}

/// @brief ON function : Set ON brightness (with ramp)
/// @param brightness Brightness set point [0..225]
/// @param onRamp Ramp up (us), Ex.: 100 -> 256*100 -> 25600us -> 25.6ms 
void OutputLED::on(uint16_t brightness, uint16_t onRamp) 
{
	uint32_t cur_us = micros();
	// Flash reset
	_flashState = 0;
	_flashCnt = 0;

	// Update brightness
	_pwm(brightness, onRamp, 0);

	/*
	// Target reached
	if (_brightness == brightness)
	{
		_last_us = cur_us;
		return;
	}
	// 
	if (onRamp > 0) {
		// Off Ramp
		if (cur_us - _last_us >= onRamp) {
			_last_us = cur_us;
			// Decrease brightness
			if (_brightness < brightness) _brightness ++;
		}
	}
	else 
		_brightness = brightness;       // Change brightness immediately

	// Set brightness 
	if (_inverse) ledcWrite(_channel, 255 - _brightness);
	else ledcWrite(_channel, _brightness);
	*/
}

/// @brief OFF function : Set OFF brightness (with ramp)
/// @param brightness Brightness set point [0..225]
/// @param offRamp Ramp down (us), Ex.: 100 -> 256*100 -> 25600us -> 25.6ms 
void OutputLED::off(uint16_t brightness, uint16_t offRamp) 
{
	uint32_t cur_us = micros();
	// Flash reset
	_flashState = 0;
	_flashCnt = 0;

	// Update brightness
	_pwm(brightness, 0, offRamp);
/*
	// Target reached
	if (_brightness == brightness)
	{
		_last_us = cur_us;
		return;
	}
	//
	if (offRamp > 0) {
		// Off Ramp
		if (cur_us - _last_us >= offRamp) {
			_last_us = cur_us;
			// Decrease brightness
			if (_brightness > brightness) _brightness --;
		}
	}
	else 
		_brightness = brightness;       // Change brightness immediately

	// Set brightness 
	if (_inverse) ledcWrite(_channel, 255 - _brightness);
	else ledcWrite(_channel, _brightness);
*/
}

/// @brief Flash led function : Set ON / OFF cycle(s) brigthness
/// @param onDuration ON duration (ms)
/// @param offDuration OFF duration (ms)
/// @param pauseDuration Pause duration (ms)
/// @param pulses Pulses number, 0 = 1 cycle (ON/OFF/Pause), n = n cycles
/// @param delay Start delay
/// @param onBrightness ON brightness [0..225]
/// @param offBrightness OFF brightness [0..225]
/// @param onRamp Ramp up (us), Ex.: 100 -> 256*100 -> 25600us -> 25.6ms 
/// @param offRamp Ramp down (us), Ex.: 100 -> 256*100 -> 25600us -> 25.6ms  
/// @return Start state (step 2)
bool OutputLED::flash(uint32_t onDuration, uint32_t offDuration, uint32_t pauseDuration, uint8_t pulses, uint32_t delay, uint16_t onBrightness, uint16_t offBrightness, uint16_t onRamp, uint16_t offRamp)
{
	uint32_t cur_ms = millis();
	//uint16_t brightness;

	// State machine
	switch (_flashState) {
		case 0: 	// Step 0 : Reset and start cycle(s)
			_flash_last_ms = cur_ms;
			_flashState = 1;
			break;
			
		case 1: 	// Step 1 : Waiting for 'start delay', first pass
			if (cur_ms - _flash_last_ms >= delay) {
				_flashState = 2;
			}
			break;

		case 2: 	// Step 2 : Start On
			_flashCnt ++; 		// Increase loop counter
			_flash_last_ms = cur_ms;
			_flashState = 3;
			_flashBrightness = onBrightness;
			_start = true;
			break;

		case 3: 	// Step 3 : Waiting for 'ON duration'
			if (cur_ms - _flash_last_ms >= onDuration) 
			{
				_flashState = 4;
				_start = false;
			}
			break;

		case 4: 	// Step 4 : Start Off
			_flash_last_ms = cur_ms;
			_flashState = 5;
			_flashBrightness = offBrightness;
			break;

		case 5: 	// Step 5 : Waiting for 'OFF duration'
			if (cur_ms - _flash_last_ms >= offDuration) 
			{
				_flashState = 6;
				_start = false;
			}
			break;

		case 6: 	// Step 6 : Flash sequence finished ?
			if (_flashCnt >= pulses) 
			{
				_flashCnt = 0;
				_flash_last_ms = cur_ms;
				_flashState = 7; 	// Sequence finished
			} else {
				_flashState = 2;	// Next cycle
			}
			break;
			
		case 7: 	// Step 7 : Waiting for 'Pause' duration
			if (cur_ms - _flash_last_ms >= offDuration) 
			{
				_flashState = 2;
			}
			break;
	}
	// Brightness update
	_pwm(_flashBrightness, onRamp, offRamp);
	//
	return _start;
}

/*
** Private functions
*/

/// @brief PWM function : Set brightness (with ramp up or ramp down)
/// @param brightness Brightness set point [0..225]
/// @param onRamp Ramp up (us), Ex.: 100 -> 256*100 -> 25600us, 25.6ms 
/// @param offRamp Ramp down (us), Ex.: 100 -> 256*100 -> 25600us, 25.6ms  
void OutputLED::_pwm(uint16_t brightness, uint16_t onRamp, uint16_t offRamp) 
{
	uint32_t cur_us = micros();

	// Target reached
	if (_brightness == brightness)
	{
		_last_us = cur_us;
		return;
	}
	//
	if (_brightness > brightness)
	{
		// Ramp down
		if (offRamp > 0) 
		{
			if (cur_us - _last_us >= offRamp) 
			{
				_last_us = cur_us;
				// Decrease brightness
				_brightness --;
			}
		}
		else
			_brightness = brightness;       // Change brightness without ramp
	}
	// 
	if (_brightness < brightness) 
	{
		// Ramp down
		if (onRamp > 0) 
		{
			if (cur_us - _last_us >= onRamp) 
			{
				_last_us = cur_us;
				// Increase brightness
				_brightness ++;
			}
		}
		else
			_brightness = brightness;       // Change brightness without ramp
	}

	// Set brightness 
	if (_inverse) ledcWrite(_channel, 255 - _brightness);
	else ledcWrite(_channel, _brightness);
}


//
// EOF
//
