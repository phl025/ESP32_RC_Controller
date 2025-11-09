//
// RxPwm.cpp : RX_Reciver class for PWM
//
// Author : PHL @2025

//
#include "objects/receiver/rxBase.h"
#include "objects/receiver/rxPwm.h"

// Class definition (methodes)

/// @brief RxPwm, Class creator for PWM reciever
/// @param in_channels Number of channels (must be <= MAX_CHANNELS)
/// @param in_average Average configuration (Default=1)
/// @param p_chFailSafe FailSafe configuration
/// @param p_chReversed Revesed configuration
RxPwm::RxPwm(int in_channels, uint8_t *p_chPin, uint8_t *p_chNum, int *p_chFailSafe, int *p_chIntegrator, bool *p_chReversed) : RxBase(in_channels, p_chFailSafe, p_chIntegrator, p_chReversed)
{
	// Limit
	if (in_channels > MAX_PWM_CHANNELS)
		in_channels = MAX_PWM_CHANNELS;
	// From model.h
	pwmPin_ = p_chPin;
	pwmChannel_ = p_chNum;
	//
	nb_channels_ = in_channels;
	counter_ = 0;
#ifdef PWM_RTM
	// Semaphore
	xPwmSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage variable access
	if ((xPwmSemaphore) != NULL)
		xSemaphoreGive((xPwmSemaphore)); // Make the PWM variable available for use, by "Giving" the Semaphore.
	else
		xPwmSemaphore = NULL;
#endif
	// Reset data
	for (int i = 0; i <= MAX_PWM_CHANNELS; i++)
	{
		channel[i].raw = 0;
		channel[i].rx = 0;
	}
	// For first readCommand
	new_data = true;
}

/// @brief Set new read
/// @param cur_us current timer (us), use micros()
void RxPwm::setNewRead(uint32_t cur_us)
{
	// Add PHL
	new_pwm_data_ = true;
	ready = true;
	last_good_read_ = cur_us;
	it_count_++;
}

/// @brief RxPwm, update channels data
/// @param cur_us current timer (us), use micros()
/// @param p_chValue channel[] value as us [500..2500]
void RxPwm::updateChannels(uint32_t cur_us, uint32_t *p_chValue) // uint32_t ch1, uint32_t ch2, uint32_t ch3, uint32_t ch4, uint32_t ch5, uint32_t ch6)
{
	// Delta us
	uint32_t _delta_us = timeDiff(cur_us, last_good_read_);
	bool new_pwm_data_ = false;

	// Raw channel update
	for (int i = 1; i <= nb_channels_; i++)
	{
		if (p_chValue[i] > 500 && p_chValue[i] < 2500)
		{
			channel[i].raw = (uint16_t)p_chValue[i];
			new_pwm_data_ = true;
		}
	}

	// Data update
	if (new_pwm_data_)
	{
		new_pwm_data_ = false;
		//
		ready = true;
		new_data = true;
		failSafe = false;
		last_good_read_ = cur_us;
		error = 1; // For debug
		rd_count_++;
	}

	// Signal lost (>1s)
	if ((_delta_us > 1000000L) && ready)
	{
		//  FailSafe channel
		channel[0].raw = 0;
		failSafe = true;
		ready = false;
		new_data = true;
		counter_ = 0;
		error = -3; // For debug
	}

	/*
	// Normalize, auto zero and reverse channels
	processRawChannels();

	// Failsafe for RC signals
	failSafe = (pulseWidthRaw[3] < 500 || pulseWidthRaw[3] > 2500);
	failsafeRcSignals();
	*/
}


// =======================================================================================================
// PWM SIGNAL INTERRUPT - GPIO ISR
// =======================================================================================================
/// @brief Input Edge interrupt CH1 -> Read pulse time [0..2000us]
void IRAM_ATTR RxPwm::gpio_isr_ch1()
{
	static uint32_t ton1;
	uint32_t cur_us;
	uint32_t delta;
	//
	cur_us = micros();
	//if (gpio_get_level((gpio_num_t)pwmPin_[0]))		//pwm_in_[0]))
	if (gpio_get_level((gpio_num_t)pwmPin_[0]))
		ton1 = cur_us;
	else
	{
		delta = cur_us - ton1;
		if (delta > 0)
		{
			pwmBuffer_[1] = delta;
			//new_isr_data_ = true;
		}
	}
}

/// @brief Input Edge interrupt CH2 -> Read pulse time [0..2000us]
void IRAM_ATTR RxPwm::gpio_isr_ch2()
{
	static uint32_t ton2;
	uint32_t cur_us;
	uint32_t delta;
	//
	cur_us = micros();
	if (gpio_get_level((gpio_num_t)pwmPin_[1]))
		ton2 = cur_us;
	else
	{
		delta = cur_us - ton2;
		if (delta > 0)
		{
			pwmBuffer_[2] = delta;
		}
	}
}

/// @brief Input Edge interrupt CH3 -> Read pulse time [0..2000us]
void IRAM_ATTR RxPwm::gpio_isr_ch3()
{
	static uint32_t ton3;
	uint32_t cur_us;
	uint32_t delta;
	//
	cur_us = micros();
	if (gpio_get_level((gpio_num_t)pwmPin_[2]))
		ton3 = cur_us;
	else
	{
		delta = cur_us - ton3;
		if (delta > 0)
		{
			pwmBuffer_[3] = delta;
			new_isr_data_ = true;
		}
	}
}

/// @brief Input Edge interrupt CH4 -> Read pulse time [0..2000us]
void IRAM_ATTR RxPwm::gpio_isr_ch4()
{
	static uint32_t ton4;
	uint32_t cur_us;
	uint32_t delta;
	//
	cur_us = micros();
	if (gpio_get_level((gpio_num_t)pwmPin_[3]))
		ton4 = cur_us;
	else
	{
		delta = cur_us - ton4;
		if (delta > 0)
		{
			pwmBuffer_[4] = delta;
		}
	}
}

/// @brief Input Edge interrupt CH5 -> Read pulse time [0..2000us]
//#if RX_CHANNELS > 4
void IRAM_ATTR RxPwm::gpio_isr_ch5()
{
	static uint32_t ton5;
	uint32_t cur_us;
	uint32_t delta;
	//
	cur_us = micros();
	if (gpio_get_level((gpio_num_t)pwmPin_[4]))
		ton5 = cur_us;
	else
	{
		delta = cur_us - ton5;
		if (delta > 0)
		{
			pwmBuffer_[5] = delta;
		}
	}
}
//#endif

/// @brief Input Edge interrupt CH6 -> Read pulse time [0..2000us]
//#if RX_CHANNELS > 5
void IRAM_ATTR RxPwm::gpio_isr_ch6()
{
	static uint32_t ton6;
	uint32_t cur_us;
	uint32_t delta;
	//
	cur_us = micros();
	if (gpio_get_level((gpio_num_t)pwmPin_[5]))
		ton6 = cur_us;
	else
	{
		delta = cur_us - ton6;
		if (delta > 0)
		{
			pwmBuffer_[6] = delta;
		}
	}
}
//#endif

/// @brief Attach GPIO interrupts
void RxPwm::attachInterrupts()
{
	//String msg = "Init pins ";
	new_isr_data_ = false;
	// Attach each pin interrupt
	Serial.printf("* RxPWM : Init Pins ");
	pinMode(pwmPin_[0], INPUT_PULLUP);
	Serial.printf("%i, ", pwmPin_[0]);
	attachInterrupt(pwmPin_[0], RxPwm::gpio_isr_ch1, CHANGE);
	pinMode(pwmPin_[1], INPUT_PULLUP);
	Serial.printf("%i, ", pwmPin_[1]);
	attachInterrupt(pwmPin_[1], RxPwm::gpio_isr_ch2, CHANGE);
	pinMode(pwmPin_[2], INPUT_PULLUP);
	Serial.printf("%i, ", pwmPin_[2]);
	attachInterrupt(pwmPin_[2], RxPwm::gpio_isr_ch3, CHANGE);
	pinMode(pwmPin_[3], INPUT_PULLUP);
	Serial.printf("%i, ", pwmPin_[3]);
	attachInterrupt(pwmPin_[3], RxPwm::gpio_isr_ch4, CHANGE);
	if (nb_channels_ > 4)
	{
		pinMode(pwmPin_[4], INPUT_PULLUP);
		Serial.printf("%i, ", pwmPin_[4]);
		attachInterrupt(pwmPin_[4], RxPwm::gpio_isr_ch5, CHANGE);
	}
	if (nb_channels_ > 5)
	{
		pinMode(pwmPin_[5], INPUT_PULLUP);
		Serial.printf("%i, ", pwmPin_[5]);
		attachInterrupt(pwmPin_[5], RxPwm::gpio_isr_ch6, CHANGE);
	}
	//
	Serial.printf("(%i channels) as INPUT_PULLUP\n", nb_channels_);

}

/// @brief RxPwm, update channels data
/// @param cur_us current timer (us), use micros()
void RxPwm::updateChannels(uint32_t cur_us)
{
	// Delta us
	uint32_t _delta_us = timeDiff(cur_us, last_good_read_);
	bool new_pwm_data_ = false;

	// New data
	if (new_isr_data_)
	{
		// Raw channel update
		for (int i = 1; i <= nb_channels_; i++)
		{
			if (pwmBuffer_[i] > 500 && pwmBuffer_[i] < 2500)
			{
				channel[i].raw = (uint16_t)pwmBuffer_[i];
				new_pwm_data_ = true;
			}
		}
		new_isr_data_ = false;
	}

	// Data update
	if (new_pwm_data_)
	{
		new_pwm_data_ = false;
		//
		ready = true;
		new_data = true;
		failSafe = false;
		last_good_read_ = cur_us;
		error = 1; // For debug
		rd_count_++;
	}

	// Signal lost (>1s)
	if ((_delta_us > 1000000L) && ready)
	{
		// Reset buffer
		for (int i = 1; i <= nb_channels_; i++)
			pwmBuffer_[i] = 0;
		//  FailSafe channel
		channel[0].raw = 0;
		failSafe = true;
		ready = false;
		new_data = true;
		counter_ = 0;
		error = -3; // For debug
	}

	/*
	// Normalize, auto zero and reverse channels
	processRawChannels();

	// Failsafe for RC signals
	failSafe = (pulseWidthRaw[3] < 500 || pulseWidthRaw[3] > 2500);
	failsafeRcSignals();
	*/
}

//
// EOF
//