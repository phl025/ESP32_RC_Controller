//
// OutputLED.h : Output LED object
//
// Object inspired by statusLED / TheDIYGuy999
// Author : PHL @2025

#ifndef OUTPUT_LED_H
#define OUTPUT_LED_H

// 
#include <stdint.h>

// Class definition (header)
class OutputLED {
// Data
private:
	bool _inverse;
	uint8_t _pin;
	uint8_t _channel;
	uint16_t _frequency;
	//
	int _brightness;        // Current brightness
	uint32_t _last_us;      // Last us, for pwm ramps
	//
	uint32_t _flash_last_ms;    // Flash, last ms
	uint8_t _flashState = 0;    // Flash state
	uint8_t _flashCnt;          
	int _flashBrightness;
	bool _start;

public:

// Methode
public:
	OutputLED(bool inverse = false);
	//
	void begin(uint8_t pin, uint8_t channel, uint16_t frequency, uint8_t resolution = 8);
	//
	void pwm(uint16_t brightness);
	void pwm(uint16_t brightness, uint16_t onRamp, uint16_t offRamp);
	// 
	bool flash(uint32_t onDuration, uint32_t offDuration, uint32_t pauseDuration, uint8_t pulses = 0, uint32_t delay = 0, uint16_t onBrightness = 255, uint16_t offBrightness = 0, uint16_t onRamp = 0, uint16_t offRamp = 0);
	// Obsolete (use pwm)
	void off(uint16_t brightness, uint16_t offRamp);
	void on(uint16_t brightness, uint16_t onRamp);

private:
	void _pwm(uint16_t brightness, uint16_t onRamp, uint16_t offRamp);

};


#endif
//
// EOF
//
