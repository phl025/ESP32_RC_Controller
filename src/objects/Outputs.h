//
// Outputs.h : Outputs command
//
// Author : PHL @2025

#ifndef OUTPUTS_H
#define OUTPUTS_H

// For GPIO
#include <driver/gpio.h>
//
#include "global.h"
// Ext_src : TheDIYGuy999 source code
#include "src_ext/statusLED.h"

// Class definition (header)
class Outputs {
// Data
private:
	int outputCount_ = OUTPUTS_MAX;
	uint8_t pins_[OUTPUTS_MAX] = {18, 5, 17, 16, 4, 2, 15, 23, 22, 3, 21, 19};
	esp_err_t gpio_error_;		// GPIO error code

public:
	statusLED channel[OUTPUTS_MAX];	// RX channel objects

// Methode
public:
	Outputs(void);
	//Outputs(int count, int* p_pin);
	//
	/*
	inline bool flash(uint8_t number, unsigned long onDuration, unsigned long offDuration, unsigned long pauseDuration, int pulses, int delay = 0, int bulbSimRamp = 0, int flashOffBrightness = 0)
	{ 
		return channel[number].flash(onDuration, offDuration, pauseDuration, pulses, delay, bulbSimRamp, flashOffBrightness);
	};
    inline void off(uint8_t number, int bulbSimRamp = 0, int offOffBrightness = 0) 
	{
		channel[number].off(bulbSimRamp, offOffBrightness);
	};
	*/
    //bool flash(unsigned long onDuration, unsigned long offDuration, unsigned long pauseDuration, int pulses, int delay = 0, int bulbSimRamp = 0, int flashOffBrightness = 0);
    //void on();
    //void off(int bulbSimRamp = 0, int offOffBrightness = 0);
    //void pwm(int brightness);

};


#endif
//
// EOF
//