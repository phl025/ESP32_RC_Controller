//
// RxPpm.cpp : RX_Reciver class for PPM
//
// Author : PHL @2025

//#include <Arduino.h>
#include "objects/global.h"
#include "objects/PHL_function.h"
//
#include "objects/receiver/rxBase.h"
#include "objects/receiver/rxPpm.h"

// Class definition (methodes)

/// @brief RxPpm, Class creator for PPM reciever
/// @param in_channels Number of channels (must be <= MAX_CHANNELS)
/// @param in_average Average configuration (Default=1)
RxPpm::RxPpm(int in_channels, short in_average) : RxBase(in_channels)
{
	// Limit
	if (in_channels > MAX_PPM_CHANNELS) in_channels = MAX_PPM_CHANNELS;
	//
	nb_channels_ = in_channels;
	average_cnt_ = in_average;
	counter_ = 0;
	// Reset data
	for (int i=0; i<=MAX_PPM_CHANNELS; i++)
	{
		ppm_in_[i] = 0;
		ppm_buf_[i] = 0;
		channel[i].raw = 0;
		channel[i].rx = 0;
	}
	// For first readCommand
	new_data = true;
}


/// @brief RxPpm, Class creator for PPM reciever
/// @param in_channels Number of channels (must be <= MAX_CHANNELS)
/// @param in_average Average configuration (Default=1)
/// @param p_chFailSafe FailSafe configuration
/// @param p_chReversed Revesed configuration
RxPpm::RxPpm(int in_channels, short in_average, int *p_chFailSafe, int* p_chIntegrator, bool *p_chReversed) : RxBase(in_channels, p_chFailSafe, p_chIntegrator, p_chReversed)
{
	// Limit
	if (in_channels > MAX_PPM_CHANNELS) in_channels = MAX_PPM_CHANNELS;
	//
	nb_channels_ = in_channels;
	average_cnt_ = in_average;
	counter_ = 0;
	// Reset data
	for (int i=0; i<=MAX_PPM_CHANNELS; i++)
	{
		ppm_in_[i] = 0;
		ppm_buf_[i] = 0;
		channel[i].raw = 0;
		channel[i].rx = 0;
	}
	// For first readCommand
	new_data = true;
}


/// @brief Get ppmInp value (for debug)
/// @param channel Channel number [0..MAX_CHANNELS]
/// @return ppmInp value (us), return 0 if channel < 0 or > MAX_CHANNELS
uint32_t RxPpm::GetPpmIn(int channel)
{
	if (channel <0) channel = 0;
	if (channel > MAX_CHANNELS) channel = 0;
	//
	return ppm_in_[channel];
}

/* Change to inline
/// @brief Get Last good read time (us)
/// @return Value (us)
uint32_t RxPpm::Getlast_good_read_(void)
{
	return lastGoodRead;
}
*/

// =======================================================================================================
// PPM SIGNAL INTERRUPT
// =======================================================================================================

/// @brief IT_PPM, Call by Interrupt (PPM digital pin rising edge)
/// @param cur_us current timer (us), use micros()
void RxPpm::IT_Ppm(uint32_t cur_us)
{
	uint64_t _timediff = cur_us - timelast_;
	timelast_ = cur_us;

	if (_timediff > 2500)	// Synch gap detected:
	{
	  	ppm_in_[MAX_PPM_CHANNELS] = ppm_in_[MAX_PPM_CHANNELS] + _timediff; 	// Add time
		if (average_cnt_ == NUM_OF_PPM_AVG)
		{
			///raw_channel[0] = 1500;
			channel[0].raw = 1500;
			for (int i = 0; i < MAX_PPM_CHANNELS + 1; i++)
			{
				ppm_buf_[i] = ppm_in_[i] / average_cnt_;
				ppm_in_[i] = 0;
			}
			last_good_read_ = cur_us;
			ready = true;
			new_ppm_data_ = true;		// New data
			average_cnt_ = 0;
		}
		average_cnt_++;
		counter_ = 0;
	}
	else
	{
		if (counter_ < MAX_PPM_CHANNELS)
		{
			ppm_in_[counter_] = ppm_in_[counter_] + _timediff;		// Add time
			counter_++;
		}
	}
	it_count_++;
}


/// @brief RxPpm, update channels data (with proccessing)
/// @param cur_us current timer (us), use micros()
void RxPpm::updateChannels(uint32_t cur_us)
{
	// Delta us
	uint32_t _delta_us = timeDiff(cur_us, last_good_read_);

	// Data update
	if (new_ppm_data_)
	{
		channel[0].raw = 0;
		//for (i = 1; i <= MAX_PPM_CHANNELS; i++)
		for (int i=0; i <= nb_channels_; i++)
		{
			channel[i].raw = ppm_buf_[i-1];
		}
		new_ppm_data_ = false;
		//
		ready = true;
		new_data = true;
		failSafe = false;
		last_good_read_ = cur_us;
		error = 1; // For debug
		rd_count_++;
		/// processRawChannels(FAILSAFE_CHANNEL, cur_us, last_rx_time_diff_);
	}

	// Signal lost (>1s)
	if ((_delta_us > 1000000L) && ready)
	{
		for (int i=0; i <= MAX_PPM_CHANNELS; i++)
		{
			ppm_in_[i] = 0;
		}
		// FailSafe channel
		ppm_buf_[FAILSAFE_CHANNEL-1] = 0;
		failSafe = true;
		ready = false;
		new_data = true;
		counter_ = 0;
		error = -3;		// For debug
	} 

	/*
	// NOTE: 8 channels is the maximum of this protocol!
	pulseWidthRaw[1] = ppmBuf[STEERING - 1];   // CH1 steering
	pulseWidthRaw[2] = ppmBuf[GEARBOX - 1];	   // CH2 3 position switch for gearbox (left throttle in tracked mode)
	pulseWidthRaw[3] = ppmBuf[THROTTLE - 1];   // CH3 throttle & brake
	pulseWidthRaw[4] = ppmBuf[HORN - 1];	   // CH5 jake brake, high / low beam, headlight flasher, engine on / off
	pulseWidthRaw[5] = ppmBuf[FUNCTION_R - 1]; // CH5 jake brake, high / low beam, headlight flasher, engine on / off
	pulseWidthRaw[6] = ppmBuf[FUNCTION_L - 1]; // CH6 indicators, hazards
	pulseWidthRaw[7] = ppmBuf[POT2 - 1];	   // CH7 pot 2
	pulseWidthRaw[8] = ppmBuf[MODE1 - 1];	   // CH8 mode 1 switch
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