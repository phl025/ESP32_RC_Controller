//
// RxCrsf.cpp : RX_Reciver class for CRSF
//
// Author : PHL @2025

#include <Arduino.h>
#include <stdint.h>
#include <objects/global.h>
#include <objects/PHL_function.h>
//
#include <objects/receiver/rxBase.h>
#include <objects/receiver/RxCrsf.h>

HardwareSerial _crsfSerial(1);
AlfredoCRSF _crsf;

// Class definition (methodes)

/// @brief RxCrsf, Class constructor for CRSF reciever
/// @param in_channels Number of channels (must be <= MAX_CHANNELS)
RxCrsf::RxCrsf(int in_channels) : RxBase(in_channels)
{
	// Reset data
	error = 0;
	for (int i=0; i<=MAX_CHANNELS; i++)
	{
		channel[i].raw = 0;
		channel[i].rx = 0;
	}
	// Start serial for CRSF
	_crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, COMMAND_RX, COMMAND_TX);
	if (!_crsfSerial)
	{
		error = -1;		// Invalid serial configuration
		return;
	}
	_crsf.begin(_crsfSerial);
	// For first readCommand
	new_data = true;
}


/// @brief RxCrsf, Class constructor for CRSF reciever
/// @param in_channels Number of channels (must be <= MAX_CHANNELS)
/// @param p_chIntegrator Integrator configuration
/// @param p_chFailSafe FailSafe configuration
/// @param p_chReversed Revesed configuration
RxCrsf::RxCrsf(int in_channels, int *p_chFailSafe, int* p_chIntegrator, bool *p_chReversed) : RxBase(in_channels, p_chFailSafe, p_chIntegrator, p_chReversed)
{
	// Reset data
	error = 0;
	for (int i=0; i<=MAX_CHANNELS; i++)
	{
		channel[i].raw = 0;
		channel[i].rx = 0;
	}
	// Start serial for CRSF
	_crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, COMMAND_RX, COMMAND_TX);
	if (!_crsfSerial)
	{
		error = -1;		// Invalid serial configuration
		return;
	}
	_crsf.begin(_crsfSerial);
	// For first readCommand
	new_data = true;
}


/// @brief RxCrsf, update channels data (raw_channel)
/// @param cur_us Current time (us)
void IRAM_ATTR RxCrsf::updateChannels(uint32_t cur_us)
{
	// Delta us
	uint32_t _delta_us = timeDiff(cur_us, this->last_good_read_);

	// Signals are coming in via CRSF Protocol
	// look for a good CRSF packet from the receiver
	_crsf.update();
	
	// Failsafe status
	this->failSafe = !_crsf.isLinkUp();
	// Failsafe status
	if (!this->failSafe)
	{
		if (_crsf.newChannelsPacket())
		{
			this->rd_count_++;
			// Proportional channels (in Microseconds)
			for (int i=1; i<=nb_channels_; i++)
			{
				this->channel[i].raw = _crsf.getChannel(i);
			}
			this->ready = true;
			this->new_data = true;
			this->last_good_read_ = cur_us;
			this->error = 1;		// For debug
		}
		//else 		shunte 07/09/2025
		//	// No update
		//	return;
	}
	else
	{
		this->ready = false;	// For base_rx, failSafe
		error = -2;				// For debug
	}

	// 1s -> Signal lost
	if ((_delta_us > 1000000L) && this->ready)
	{
		#ifdef CHANNEL_DEBUG
		Serial.printf("Timeout (%is)", timediff);
		#endif
		this->ready = false;
		this->new_data = true;
		this->error = -3;		// For debug
		//engineOn = false;
		//counter = 0;
	}
	///processRawChannels(this->ready, cur_us, last_rx_time_diff_);
}

// Get RX buffer[index] raw value
uint8_t RxCrsf::getBuffer(int index)
{
	if (index >= 0 && index < CRSF_MAX_PACKET_LEN)
		return _crsf.getBuffer(index);
	else
        return 0xFF;
}

// Get RX buffer length
uint8_t RxCrsf::getBufferLen(void)
{
	//return CRSF_MAX_PACKET_LEN;
	return 26;
}


/* Change to inline
/// @brief Get Last good read time (us)
/// @return Value (us)
uint32_t RxCrsf::Getlast_good_read_(void)
{
	return this->lastGoodRead;
}


/// @brief Get itCount value
/// @return Value
uint32_t RxCrsf::GetItCount(void)
{
	return this->itCount;
}


/// @brief Get rdCount value
/// @return Value
uint32_t RxCrsf::GetRdCount(void)
{
	return this->rdCount;
}
*/

//
// EOF
//