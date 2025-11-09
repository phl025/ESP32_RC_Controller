//
// ChTrigger.h : Trigger object, for RC signal detection
//
// Author : PHL @2025

#ifndef CH_TRIGGER_H
#define CH_TRIGGER_H
//
// #include "Arduino.h"
#include "stdint.h"

// Class definition (header)
class ChTrigger
{
	// Data
private:
	int16_t hysteresis_ = 50; // Hysteresys : 50 = 10%
	//
	int16_t highLevel_ = 1800; // Trigger level 60%
	uint32_t highDelay_ = 500; // Trigger delay 500ms
	uint32_t highStart_;
	bool highEdgeFlag_;
	bool highEdgeDFlag_;
	//
	int16_t lowLevel_ = 1200; // Trigger level 60%
	uint32_t lowDelay_ = 500; // Trigger delay 500ms
	uint32_t lowStart_;
	bool lowEdgeFlag_;
	bool lowEdgeDFlag_;

public:
	bool high = false;
	bool highD = false;
	bool highEdge = false;
	bool highEdgeD = false;
	bool low = false;
	bool lowD = false;
	bool lowEdge = false;
	bool lowEdgeD = false;
	//
	bool latchHigh = false;
	bool latchLow = false;

	// Methode
public:
	ChTrigger(void);
	ChTrigger(uint32_t delay);
	ChTrigger(uint32_t highLevel, uint32_t highDelay, uint32_t lowLevel, uint32_t lowDelay, int16_t hysteresis);
	void update(uint16_t pulsewidth, uint32_t cur_ms);
	//
	void setLevel(uint32_t highLevel, uint32_t highDelay, uint32_t lowLevel, uint32_t lowDelay, int16_t hysteresis);
};

#endif
