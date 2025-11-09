//
// Output.h : Output command
//
// Author : PHL @2025

#ifndef OUTPUT_H
#define OUTPUT_H
//
#include <stdint.h>
#include <driver/gpio.h> // For GPIO

// Class definition (header)
class Output
{
	// Data
private:
	gpio_num_t gpio_num_;	// GPIO Pin
	bool gpio_ok_ = false;	//
	esp_err_t gpio_error_;	// Error code

public:
	bool state = false;

	// Methode
public:
	Output(int8_t pin);
	void set(bool state);
	void set(void);
	void reset(void);
	int8_t get();
};

#endif
//
// EOF
//