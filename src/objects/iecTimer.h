//
// iecTimer.h : IEC Timers TON, TOFF
//
// Author : PHL @2025

#ifndef IEC_TIMER_H
#define IEC_TIMER_H
//
#include <stdint.h>

// Class definition (header)
class IecTimer {
// Data
public:
	bool ton = false;
  	bool toff = false;
    
private:
	uint32_t start_ms_;
	bool run_on_ = false;
	bool run_off_ = false;

// Methode
public:
    IecTimer(void);
    bool TON(bool input, uint32_t cur_ms, uint32_t delay);
    bool TOFF(bool input, uint32_t cur_ms, uint32_t delay);
	//

};

#endif
