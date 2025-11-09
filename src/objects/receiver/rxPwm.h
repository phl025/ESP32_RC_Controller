//
// RxPwm.cpp : RX_Reciver class for PWM inputs (CH1 to CH6)
//
// Author : PHL @2025

#ifndef RX_PWM_H
#define RX_PWM_H

#include <Arduino.h>
//
#include "objects/global.h"

// MODES
// PWM_RTM : Using RTM -> Not stable (Removed)
// IRS_INSIDE : Using embeded IRS -> OK (2025-11)
// PWM_GPIO_LOCAL : Using main.cpp ISR -> OK (2025-11)

// Class definition (header)
class RxPwm : public RxBase
{
	// Data
private:
	// Static for ISR
	static volatile unsigned int * pwmBuffer_;
	static volatile uint8_t *pwmPin_;
	static volatile bool new_isr_data_;

	// Array of channels number
	volatile uint8_t *pwmChannel_;
	volatile int nb_channels_;

protected:
	volatile short counter_ = 0;
	volatile bool new_pwm_data_ = false;

	// Model.h
	// 6 channels
	// const uint8_t PWM_CHANNELS[PWM_CHANNELS_NUM] = {1, 2, 3, 4, 5, 6};	 // Channel numbers
	// const uint8_t PWM_PINS[PWM_CHANNELS_NUM] = {13, 12, 14, 27, 35, 34}; // Input pin numbers (pin 34 & 35 only usable as inputs!)
	// 4 channels
	// const uint8_t PWM_CHANNELS[PWM_CHANNELS_NUM] = {1, 2, 3, 4};	 		// Channel numbers
	// const uint8_t PWM_PINS[PWM_CHANNELS_NUM] = {13, 12, 14, 27}; 			// Input pin numbers (pin 34 & 35 only usable as inputs!)

	// Methode
public:
	// RxPwm();
	RxPwm(int in_channels, uint8_t *p_chPin, uint8_t *p_chNum, int *p_chFailSafe, int *p_chIntegrator, bool *p_chReversed);
	//
	void attachInterrupts();
	inline bool getNewIsrData() const {return new_isr_data_;};
	void updateChannels(uint32_t cur_us) override;
	//
	void updateChannels(uint32_t cur_us, uint32_t *p_chValue);
	void setNewRead(uint32_t cur_us);
	//

protected:
	static void IRAM_ATTR gpio_isr_ch1();
	static void IRAM_ATTR gpio_isr_ch2();
	static void IRAM_ATTR gpio_isr_ch3();
	static void IRAM_ATTR gpio_isr_ch4();
	static void IRAM_ATTR gpio_isr_ch5();
	static void IRAM_ATTR gpio_isr_ch6();

};

#endif
//
// EOF
//
