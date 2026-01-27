//
// RxChannel.h : RX Channel class for rxBase reciever
//
// Author : PHL @2025

#ifndef RX_CHANNEL_H
#define RX_CHANNEL_H

//#include <Arduino.h>
#include <stdint.h>
#include "objects/receiver/chTrigger.h"		// Trigger - PHL

// Class definition (header)
class RxChannel
{
// Data
public:
	uint16_t binary;    	// RX channel : binary value, from CRSF / SBUS comunication
	uint16_t raw;          	// RX channel : raw value
	uint16_t rx;           	// RX channel : proccessed
	uint16_t rxd;         	// RX channel : delayed
    uint16_t last_rx;		// RX channel : last good rx value
	ChTrigger trigger;		// Trigger,	using default values (1800, 250, 1200, 250, 50)
	//
	uint16_t mixVal;		// Multiswitch value (for debug)

protected:
	bool reverse_;			// Reverse configuration, FALSE : Normal, TRUE : Reversed
	int failsafe_;			// FailSafe configuration, [1000..1500..2000] : Defined position, [0] : OFF, [-1] : last good value
	int int_time_;			// Integrator time, ms to move from [1000] to [2000], default value : 100 ms
	uint8_t switchBits_;	// Multiswitch bits status
	uint8_t switchLatchs_;	// Multiswitch latch status
	//uint8_t switchBits_;	// Multiswitch bits status

// Methodes
public:
	RxChannel();
	RxChannel(int integrator);
	RxChannel(int failSafe, int integrator, bool Reversed);
    //
    void process(uint32_t delta_us, uint32_t cur_us, bool failsafe);
	uint8_t GetMix4(bool failsafe);
	uint8_t GetMix3P(bool failsafe);
	// Inline function
	inline bool getSwitchBit(int bit) const {return ((switchBits_ >> bit) & 0x01);}			// bitRead(mixVal, bit);}
	inline bool getSwitchLatch(int bit) const {return ((switchLatchs_ >> bit) & 0x01);}		// bitRead(mixVal, bit);}
};
#endif

//
// EOF
//