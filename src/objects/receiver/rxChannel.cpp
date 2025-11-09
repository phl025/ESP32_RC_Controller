//
// RxChannel.cpp : RX Channel class for rxBase reciever
//
// Author : PHL @2025

#include <Arduino.h>		// for map
//
#include "objects/receiver/RxChannel.h"

// Class definition (methodes)

/// @brief RxChannel : Channel class
RxChannel::RxChannel()
{
    // Default values
	raw = 1500;
	rx = 1500;
	rxd = 1500;
	last_rx = 1500;
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
	raw = 1500;
	rx = 1500;
	rxd = 1500;
	last_rx = 1500;
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
	raw = 1500;
	rx = 1500;
	rxd = 1500;
	last_rx = 1500;
    // Config
	reverse_ = reversed;
	failsafe_ = failSafe;
    int_time_ = integrator;
}

/// @brief Process : channel proccess
/// @param delta_us Last delta process (us)
/// @param cur_us Current time (us)
void RxChannel::process(uint32_t delta_us, uint32_t cur_us)
{
    // Curve : Not implemented
    //if (curveExponentialThrottle != null)
    //	raw_value = reMap(curveExponentialThrottle, raw_channel[i]);
    // Output
    if (!reverse_)
        // Normal
        rx = raw;
    else
        // Reversed
        rx = map(raw, 0, 3000, 3000, 0);

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
    // Save
    last_rx = rx;
    // Trigger
    trigger.update(rx, (cur_us/1000));
}

//
// EOF
//