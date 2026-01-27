//
// iecTimer.cpp : IEC Timers TON, TOFF
//
// Author : PHL @2025

//#include "objects/controler/iecTimer.h"
#include "iecTimer.h"

// Class definition (methodes)

/// @brief Construct a new Tempo (TON)
IecTimer::IecTimer(void)                      // Constructor (called, when new ojects of that class are created)
{ 
    ton = false;
    run_on_ = false;
    toff = false;
    run_off_ = false;
}

/// @brief TON : IEC Timer delay when signal is 'ON'
/// @param input Signal input
/// @param cur_ms Current time ms (or us)
/// @param delay Delay time, ms (or us)
/// @return State of IEC Timer
bool IecTimer::TON(bool input, uint32_t cur_ms, uint32_t delay)
{
    // Start tempo
    if (input && !run_on_)
    {
        start_ms_ = cur_ms;
        run_on_ = true;
    }
    // Stop tempo
    if (!input)
        run_on_ = false;
    // Tempo 
    ton = run_on_ && (cur_ms - start_ms_ > delay);
    return ton;
}

/// @brief TOFF : IEC Timer delay when signal is 'OFF'
/// @param input Signal input
/// @param cur_ms Current time ms (or us)
/// @param delay Delay time, ms (or us)
/// @return State of IEC Timer
bool IecTimer::TOFF(bool input, uint32_t cur_ms, uint32_t delay)
{
    // Start tempo
    if (!input && !run_off_)
    {
        start_ms_ = cur_ms;
        run_off_ = true;
    }
    // Stop tempo
    if (input)
        run_off_ = false;
    // Tempo 
    toff = run_off_ && (cur_ms - start_ms_ > delay);
    return toff;
}

//
// EOF
//