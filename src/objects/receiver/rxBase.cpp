//
// RxBase.cpp : RX_Receiver base class
//
// Author : PHL @2025

#include <Arduino.h> // for map
//
#include "objects/global.h"
#include "objects/receiver/rxBase.h"

// Class definition (methodes)

/// @brief RxBase : Common class for reciever
/// @param channels Number of channels (must be <= MAX_CHANNELS)
RxBase::RxBase(int channels)
{
	// nb_channels = channels
	nb_channels_ = min(channels, MAX_CHANNELS);
	//
	fs_channel_ = new int[MAX_CHANNELS + 1];
	rev_channel_ = new bool[MAX_CHANNELS + 1];
	int_channel_ = new int[MAX_CHANNELS + 1];
	// Reset / Update data
	// for (int i = 0; i <= nb_channels; i++)
	for (int i = 0; i < MAX_CHANNELS; i++)
	{
		channel[i] = RxChannel(100);
		channel[i].raw = 1500;
		//
		rev_channel_[i] = false;
		fs_channel_[i] = 1500;
		int_channel_[i] = 100;
	}
}

/// @brief RxBase : Common class for reciever (prefered)
/// @param channels Number of channels (must be <= MAX_CHANNELS)
/// @param p_chFailSafe FailSafe channels configuration
/// @param p_chIntegrator Integrator configuration
/// @param p_chReversed Reverse channels configuration
RxBase::RxBase(int channels, int *p_chFailSafe, int *p_chIntegrator, bool *p_chReversed)
{
	// nb_channels = channels
	nb_channels_ = min(channels, MAX_CHANNELS);
	//
	fs_channel_ = p_chFailSafe;
	rev_channel_ = p_chReversed;
	int_channel_ = p_chIntegrator;
	//
	// Reset / Update data
	for (int i = 0; i < MAX_CHANNELS; i++)
	{
		channel[i] = RxChannel(p_chFailSafe[i], p_chIntegrator[i], p_chReversed[i]);
		channel[i].raw = p_chFailSafe[i];	//1500;
	}
}

/// @brief RxBase, update channels data (Set to 0)
/// @param cur_us current timer (us), use micros()
void RxBase::updateChannels(uint32_t cur_us)
{
	// Delta us
	// uint32_t _delta_us = timeDiff(cur_us, last_good_read_);
	last_good_read_ = cur_us;
	// Set all channels 0
	channel[0].raw = 0;
	for (int i = 1; i <= nb_channels_; i++)
	{
		channel[i].raw = 0;
	}
	new_data = true;
	failSafe = true;
	error = -1; // For debug
}

/// @brief Process raw channels (prefered)
/// @param cur_us Current time (us)
void RxBase::processRawChannels(uint32_t cur_us)
{
	// Delta time (us)
	delta_us_proccess = timeDiff(cur_us, last_us_proccess_);

	// Failsafe active ?
	if (failSafe)	// || !ready)
	{
		// Failsafe process
		processFailSafe();
	}
	// Process alls chanels
	for (int i = 1; i <= nb_channels_; i++)
	{
		// Channel process (raw conversion, integrator, edges...)
		channel[i].process(delta_us_proccess, cur_us, failSafe);
	}
	
	/*
	// Failsafe active ?
	if (!failSafe && ready)
	{
		// Process alls chanels
		for (int i = 1; i <= nb_channels_; i++)
		{
			// Channel process (raw conversion, integrator, edges...)
			channel[i].process(delta_us_proccess, cur_us);
			/*
			// Raw
			raw_value = raw_channel[i];
			// Curve : Not implemented
			//if (curveExponentialThrottle != null)
			//	raw_value = reMap(curveExponentialThrottle, raw_channel[i]);
			// Output
			if (!rev_channel[i])
				// Normal
				rx_channel[i] = raw_value;
			else
				// Reversed
				rx_channel[i] = map(raw_value, 0, 3000, 3000, 0);
			// Integrator
			// To DO...
			//uint16_t value = (int_channel[i] * 1000) / delta_us;
			//Ex = int_channel[i] = 2000 (2s)
			//if (rxd_channel[i] < rx_channel[i]) rxd_channel[i]++;
			//if (rxd_channel[i] > rx_channel[i]) rxd_channel[i]--;
			// value = (5 * 100) / delta_us; // 1000pts / 2000ms
			uint16_t value = (int_channel[i] * last_delta_proccess) / 10000;
			if (rxd_channel[i] < rx_channel[i])
			{
				rxd_channel[i] += value;
				if (rxd_channel[i] > rx_channel[i])
					rxd_channel[i] = rx_channel[i];
			}
			if (rxd_channel[i] > rx_channel[i])
			{
				rxd_channel[i] -= value;
				if (rxd_channel[i] < rx_channel[i])
					rxd_channel[i] = rx_channel[i];
			}

			// Save
			last_rx_ch[i] = rx_channel[i];
			*-/
		}
	}
	else
		// Failsafe process
		processFailSafe();
	*/
	//
	last_us_proccess_ = cur_us;
}

/// @brief Set FailSafe RC Signals
void RxBase::processFailSafe()
{
	// Failsafe actions
	for (int i = 1; i <= nb_channels_; i++)
	{
		if (fs_channel_[i] >= 0)
			channel[i].raw = fs_channel_[i];
			//channel[i].rx = fs_channel_[i];
		else
			//channel[i].rx = channel[i].last_rx;
			channel[i].raw = channel[i].last_rx;
		// Integrator
		channel[i].rx = channel[i].raw;
		channel[i].rxd = channel[i].rx;
	}
}

/// @brief Calculate time difference, including rollover (us or ms)
/// - Rollover (us) = ~4295s -> ~1H 11',
/// - Rollover (ms) = ~71580' -> ~49 days, 17h,
/// @param cur Curent time
/// @param last Last time
/// @return Delta value, (including overroll)
uint32_t RxBase::timeDiff(uint32_t cur, uint32_t last)
{
	// uint32_t delta_us;
	uint32_t delta_us;
	// float val1 , val2;

	// Delta time (us)
	if (cur < last)
		delta_us = (4294967295 - last) + cur; // Rollover, us (~4295s, ~1h 11' )
	else
		delta_us = cur - last;

	return delta_us;
}

/// @brief Set raw rx_channel value
/// @param channel_nb Channel number [0..MAX_CHANNELS]
/// @param Value [500..2500] us, return 0 if channel < 0 or > MAX_CHANNELS
void RxBase::setRawChannel(int channel_nb, uint16_t value)
{
	if ((channel_nb >= 0) && (channel_nb <= nb_channels_))
	{
		channel[channel_nb].raw = value;
	}
}

/// @brief Return rx_channel value (processed)
/// @param channel_nb Channel number [0..MAX_CHANNELS]
/// @return Value [500..2500] us, return 0 if channel < 0 or > MAX_CHANNELS
uint16_t RxBase::getRxChannel(int channel_nb)
{
	if (channel_nb < 0)
		channel_nb = 0;
	if (channel_nb > MAX_CHANNELS)
		channel_nb = 0;
	//
	return channel[channel_nb].rx;
}

/// @brief Return new_data -> new read cycle
/// @return Status
bool RxBase::Get_NewData(void)
{
	if (new_data)
	{
		new_data = false;
		return true;
	}
	else
		return false;
}

//
// EOF
//