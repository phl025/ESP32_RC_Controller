//
// RxSbus.cpp : RX_Reciver class for SBUS
//
// Author : PHL @2025

#include <Arduino.h>
#include <stdint.h>
//#include <objects/global.h>
#include <objects/receiver/rxBase.h>
#include <objects/receiver/rxSbus.h>
//
#include <src_ext\sbus.h>

HardwareSerial _sBusSerial(1);
bfs::SbusRx _sBus(&_sBusSerial);
std::array<int16_t, bfs::SbusRx::NUM_CH()> _SBUSchannels;

// Class definition (methodes)

/// @brief RxSbus, Class constructor for CRSF reciever
/// @param in_channels Number of channels (must be <= MAX_CHANNELS)
RxSbus::RxSbus(int in_channels) : RxBase(in_channels)
{
	// Reset data
	error = 0;
	for (int i=0; i<=MAX_CHANNELS; i++)
	{
		channel[i].raw = 0;
		channel[i].rx = 0;
	}
	// Start serial for SBUS
    //sBus.begin(COMMAND_RX, COMMAND_TX, sbusInverted, sbusBaud); // begin SBUS communication with compatible receivers
    _sBus.begin(COMMAND_RX, COMMAND_TX, true, 100000); // begin SBUS communication with compatible receivers
    if (!_sBusSerial)
	{
		error = -1;		// Invalid serial configuration
		return;
	}
	// For first readCommand
	new_data = true;
}

/// @brief RxSbus, Class constructor for CRSF reciever
/// @param in_channels Number of channels (must be <= MAX_CHANNELS)
/// @param p_chIntegrator Integrator configuration
/// @param p_chFailSafe FailSafe configuration
/// @param p_chReversed Revesed configuration
RxSbus::RxSbus(int in_channels, int *p_chFailSafe, int* p_chIntegrator, bool *p_chReversed) : RxBase(in_channels, p_chFailSafe, p_chIntegrator, p_chReversed)
{
	// Reset data
	error = 0;
	for (int i=0; i<=MAX_CHANNELS; i++)
	{
		channel[i].raw = 0;
		channel[i].rx = 0;
	}
	// Start serial for SBUS
    //sBus.begin(COMMAND_RX, COMMAND_TX, sbusInverted, sbusBaud); // begin SBUS communication with compatible receivers
    _sBus.begin(COMMAND_RX, COMMAND_TX, true, 100000); // begin SBUS communication with compatible receivers
    if (!_sBusSerial)
	{
		error = -1;		// Invalid serial configuration
		return;
	}
	// For first readCommand
	new_data = true;
}


/// @brief RxSbus, update channels data (raw_channel)
/// @param cur_us Current time (us)
void IRAM_ATTR RxSbus::updateChannels(uint32_t cur_us)
{
	// Delta us
	uint32_t _delta_us = timeDiff(cur_us, last_good_read_);

	// Failsafe status
	//failSafe = _sBus.failsafe();
	// New  data
	if (_sBus.read() && !_sBus.failsafe() && !_sBus.lost_frame())
	{
		rd_count_++;
		// Proportional channels (in Microseconds)
		_SBUSchannels = _sBus.ch();
		for (int i=1; i<=nb_channels_; i++)
		{
			//channel[i].raw = map(_SBUSchannels[i-1], 172, 1811, 1000, 2000);
			channel[i].raw = map(_SBUSchannels[i-1], 172, 1812, 1000, 2000);		// 992-820, 992+820 - PHL
		}
		ready = true;
		failSafe = false;
		new_data = true;
		last_good_read_ = cur_us;
		error = 1;		// For debug
	} 

	// 1s -> Signal lost
	if ((_delta_us > 1000000L) && ready)
	{
		failSafe = true;
		ready = false;
		new_data = true;
		error = -3;			// For debug
	}
	///processRawChannels(ready, cur_us, last_rx_time_diff_);
}

// Get RX buffer[index] raw value
uint8_t RxSbus::getBuffer(int index)
{
	if (index >= 0 && index < getBufferLen())
		return _sBus.getBuffer(index);
	else
        return 0xFF;
}

// Get RX buffer length
/*
uint8_t RxSbus::getBufferLen(void)
{
	return _sBus.getBufferLen();
}*/

//
// EOF
//