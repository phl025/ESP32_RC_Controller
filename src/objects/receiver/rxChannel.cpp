//
// RxChannel.cpp : RX Channel class for rxBase reciever
//
// Author : PHL @2025

#include <Arduino.h> // for map
//
#include "objects/receiver/RxChannel.h"
//
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
// Inverser le bit n (0 = LSB) d'une variable uint32_t
//void toggleBit(uint32_t &value, uint8_t n)
//{
//    value ^= (1UL << n);   // XOR avec le masque 1<<n
//}

// Class definition (methodes)

/// @brief RxChannel : Channel class
RxChannel::RxChannel()
{
	// Default values
	binary = 0;
	raw = 1500;
	rx = 1500;
	rxd = 1500;
	last_rx = 1500;
	// 
	switchLatchs_ = 0;
	switchBits_ = 0;
	// Config
	reverse_ = false;
	failsafe_ = 1500;
	int_time_ = 100;
}

/// @brief RxChannel : Channel class
/// @param integrator Integrator value for rxd signal
RxChannel::RxChannel(int integrator)
{
	// Default values
	binary = 0;
	raw = 1500;
	rx = 1500;
	rxd = 1500;
	last_rx = 1500;
	// 
	switchLatchs_ = 0;
	switchBits_ = 0;
	// Config
	reverse_ = false;
	failsafe_ = 1500;
	int_time_ = integrator;
}

/// @brief RxChannel : Channel class
/// @param failSafe FailSafe configuration
/// @param integrator Integrator configuration (value for rxd signal)
/// @param reversed Reverse channel configuration (FALSE = Normal, TRUE = Reversed)
RxChannel::RxChannel(int failSafe, int integrator, bool reversed)
{
	// Default values
	binary = 0;
	raw = 1500;
	rx = 1500;
	rxd = 1500;
	last_rx = 1500;
	// 
	switchLatchs_ = 0;
	switchBits_ = 0;
	// Config
	reverse_ = reversed;
	failsafe_ = failSafe;
	int_time_ = integrator;
}

/// @brief Process : channel proccess
/// @param delta_us Last delta process (us)
/// @param cur_us Current time (us)
/// @param failsafe Failsafe state
void RxChannel::process(uint32_t delta_us, uint32_t cur_us, bool failsafe)
{
	// Curve : Not implemented
	// if (curveExponentialThrottle != null)
	//	raw_value = reMap(curveExponentialThrottle, raw_channel[i]);
	// Rx value
	if (!reverse_)
		// Normal
		rx = raw;
	else
		// Reversed
		rx = map(raw, 0, 3000, 3000, 0);

	// Save
	if (!failsafe)
		last_rx = raw;
	
	// Integrator
	// int_time_ en ms pour 1000pts [1000] a [2000]
	float pts_ms = 1000.0 / (float)int_time_;
	uint16_t value = pts_ms * (float)delta_us / (float)1000;
	//
	if (rxd < rx)
	{
		rxd += value;
		if (rxd > rx)
			rxd = rx;
	}
	if (rxd > rx)
	{
		rxd -= value;
		if (rxd < rx)
			rxd = rx;
	}
	// Trigger
	trigger.update(rx, (cur_us / 1000));
}

/// @brief GetMix4, buttons multiswitch (8x btn)
/// @return Switch state, like '0x00000000'
uint8_t RxChannel::GetMix4(bool failsafe)
{
	// Use 'Mix4.lua' mixe script to encode 1,2,3 or 4 switchs (Ex. 'SA'+'SB'+'SC'+'SD')
	// [0..255] = 2^8 or 4 x 4 x 4 x 4 (4 states for each switch, dua push button)
	uint16_t val = raw - 988;
	uint16_t valr = val & 0x0003;

	if (failsafe)
	{
		mixVal = 0;
		switchBits_ = 0;
		switchLatchs_ = 0;
		return 0;
	}

	// LSB correction (if one pts lost, Test !!)
	if (valr == 3) val++;
	// For debug
	mixVal = val;
	//
	switchBits_ = uint8_t(val >> 2);
	return switchBits_;
}

/// @brief GetMix3P, 3-positions Multiswitch (4x)
/// @return Switch state, like '0x00000000'
uint8_t RxChannel::GetMix3P(bool failsafe)
{
	// Use 'Mix3P.lua' mixe script to encode 1,2,3 or 4 3-positions switchs (Ex. 'SA'+'SB'+'SC'+'SD')
	// [0..80] = 3 x 3 x 2 x 3 (3 states for each switch)
	uint16_t val = 0; 		//= (raw - 1020) / 12; 	// Tested 2026-01-13
	uint16_t valr = 0; 		//= (raw - 1020) % 12;
	uint8_t status = 0;
	uint8_t changed;
	uint8_t prevState = switchBits_;
	uint8_t latch = switchLatchs_;

	if (failsafe)
	{
		mixVal = 0;
		switchBits_ = 0;
		switchLatchs_ = 0;
		return 0;
	}

	// Decode
	val = (raw - 1020) / 12; 		// Tested 2026-01-13
	valr = (raw - 1020) % 12;
	// LSB correction (if few pts lost, Test !!)
	if (valr > 7) val++;
	// For debug
	mixVal = val;
	//
	static const int weights[8] = {1, 2, 3, 6, 9, 18, 27, 54}; 
	for (int bit = 7; bit >= 0; --bit) 
	{
		if (val >= weights[bit])
		{
			status |= 1UL << bit;		//bitSet(state, bit);
			val -= weights[bit];
		}
	}
	//
	changed = prevState ^ status;
	//
	for (int bit = 7; bit >= 0; --bit) 
	{
		// bit changed et status '1' -> Rising edge
		if (bitRead(changed, bit) && bitRead(status, bit))
			bitToggle(latch, bit);
	}
	// Save
	switchBits_ = status;
	switchLatchs_ = latch;
	return switchBits_;
	/*
	if (val >= 54) // MSB
	{
		// state |= 0b10000000;
		bitSet(state, 7);
		val -= 54;
	}
	if (val >= 27)
	{
		// state |= 0b01000000;
		bitSet(state, 6);
		val -= 27;
	}
	if (val >= 18)
	{
		// state |= 0b00100000;
		bitSet(state, 5);
		val -= 18;
	}
	if (val >= 9)
	{
		// state |= 0x000b0000;
		bitSet(state, 4);
		val -= 9;
	}
	if (val >= 6)
	{
		// state |= 0b00001000;
		bitSet(state, 3);
		val -= 6;
	}
	if (val >= 3)
	{
		// state |= 0b00000100;
		bitSet(state, 2);
		val -= 3;
	}
	if (val >= 2)
	{
		// state |= 0b00000010;
		bitSet(state, 1);
		val -= 2;
	}
	if (val >= 1) // LSB
	{
		// state |= 0b00000001;
		bitSet(state, 0);
		val -= 1;
	}
	//
	return state;
	*/
}

//
// EOF
//