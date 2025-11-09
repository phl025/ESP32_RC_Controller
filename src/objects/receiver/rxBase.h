//
// rxBase.h : RX_Reciver base class
//
// Author : PHL @2025

#ifndef RX_BASE_H
#define RX_BASE_H

//#include <Arduino.h>
#include <stdint.h>
// Objects
#include "objects/global.h"
#include "objects/receiver/RxChannel.h"

// RX Status
#define RXREADY 1
#define RXFAILSAFE -1
#define RX_COMMUNICATION_LOST -2

// Class definition (header)
class RxBase
{
// Data
public:
	RxChannel channel[MAX_CHANNELS + 1];	// RX channel objects
	//
	volatile bool new_data = false;		// New RX data present
	volatile bool ready = false;			// RX Ready (not Failsafe)
	volatile bool failSafe = true;			//
	volatile int16_t error = 0;			// Error code for debug
	volatile uint32_t delta_us_proccess;	// Delta time proccessed

protected:
	bool *rev_channel_;
	int *fs_channel_;
	int *int_channel_;
	//
	uint16_t nb_channels_ = MAX_CHANNELS;
	volatile uint32_t last_rx_time_diff_;		// Last time rx recived
	volatile uint32_t last_us_proccess_;		// Last time proccessed
	//
	volatile uint32_t timelast_ = 0;
	volatile uint32_t last_good_read_ = 0;
	volatile uint32_t it_count_ = 0;
	volatile uint32_t rd_count_ = 0;
	
// Methodes
public:
	RxBase(int channels);
	RxBase(int channels, int* p_chFailSafe, int* p_chIntegrator, bool* p_chReversed);
	//
	virtual void updateChannels(uint32_t cur_us);
	void processRawChannels(uint32_t cur_us);
	uint32_t timeDiff (uint32_t cur, uint32_t last);
	//
	void setRawChannel(int channel, uint16_t value);
	uint16_t getRxChannel(int channel);
	bool Get_NewData();
	uint8_t getBuffer(int index) const {return 0;}
	uint8_t getBufferLen() const {return 8;}

	/* Change to inline
	int getNbChannels();
	uint32_t GetItCount();
	uint32_t GetRdCount();
	uint32_t GetLastProccess();
	//uint8_t getBuffer(int index);
	*/
	// Inline function
	inline int getNbChannels() const {return nb_channels_;}
	inline uint32_t GetItCount() const {return it_count_;}
	inline uint32_t GetRdCount() const {return rd_count_;}
	inline uint32_t GetLastProccess()  const {return last_us_proccess_;}

protected :
	void processRawChannels(int failsafe_channel, uint32_t cur_us, uint32_t delta_us);
	// Obsolete :
	//void processRawChannels(bool ready, uint32_t cur_us, uint32_t delta_us);
	//void processRawChannels(int failsafe_channel, int* p_channelFailSafe, bool* p_channelReversed);
	void processFailSafe();

};
#endif

//
// EOF
//
