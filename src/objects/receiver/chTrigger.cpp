//
// ChTrigger.cpp : Trigger object
//
// Author : PHL @2025

#include "chTrigger.h"

// Class definition (methodes)

/// @brief Construct a new ChTrigger
ChTrigger::ChTrigger(void) // Constructor (called, when new ojects of that class are created)
{
	//_duration = duration;
}

/// @brief Construct a new ChTrigger, with parameters
/// @param delay trigger delay, for _highD or _lowD
ChTrigger::ChTrigger(uint32_t delay) // Constructor (called, when new ojects of that class are created)
{
	if (delay > 10)
	{
		highDelay_ = delay;
		lowDelay_ = delay;
	}
	//_duration = duration;
}

/// @brief Construct a new ChTrigger, with parameters
/// @param highLevel High trigger level [1500..2000]
/// @param highDelay High trigger delay, for _highD (us)
/// @param lowlevel Low trigger level [1000..1500]
/// @param lowDelay Low trigger delay, for _lowD (us)
/// @param hysteresis Hysteresis for _hiqhEdgeFlag or _lowEdgeFlag
ChTrigger::ChTrigger(uint32_t highLevel, uint32_t highDelay, uint32_t lowLevel, uint32_t lowDelay, int16_t hysteresis) // Constructor (called, when new ojects of that class are created)
{
	highLevel_ = highLevel;
	lowLevel_ = lowLevel;
	highDelay_ = highDelay;
	lowDelay_ = lowDelay;
	hysteresis_ = hysteresis;
}

/// @brief Set / Change trigger parameters
/// @param highLevel High trigger level [1500..2000]
/// @param highDelay High trigger delay, for _highD (us)
/// @param lowlevel Low trigger level [1000..1500]
/// @param lowDelay Low trigger delay, for _lowD (us)
/// @param hysteresis Hysteresis for _hiqhEdgeFlag or _lowEdgeFlag
void ChTrigger::setLevel(uint32_t highLevel, uint32_t highDelay, uint32_t lowLevel, uint32_t lowDelay, int16_t hysteresis)
{
	highLevel_ = highLevel;
	lowLevel_ = lowLevel;
	highDelay_ = highDelay;
	lowDelay_ = lowDelay;
	hysteresis_ = hysteresis;
}

/// @brief ChTrigger update
/// @param signal Channel signal [1000..2000]
/// @param cur_ms Current ms for delay
void ChTrigger::update(uint16_t signal, uint32_t cur_ms)
{
	highEdge = false;
	lowEdge = false;
	highEdgeD = false;
	lowEdgeD = false;

	if (signal == 0)
	{
		low = false;
		lowD = false;
		high = false;
		highD = false;
		latchLow = false;
		latchHigh = false;
		return;
	}

	// High trigger
	if ((signal > highLevel_) && !highEdgeFlag_)
	{
		highEdge = true;
		highEdgeFlag_ = true;
		highStart_ = cur_ms;
		latchHigh = !latchHigh; 			// Toggle latchOn
	}
	if (signal < highLevel_ - hysteresis_) 	// 10%
		highEdgeFlag_ = false;
	//
	high = (signal > highLevel_);
	highD = highEdgeFlag_ && ((cur_ms - highStart_) > highDelay_);
	//
	if (highD && !highEdgeDFlag_)
	{
		highEdgeD = true;
		highEdgeDFlag_ = true;
	}
	if (!highD)
		highEdgeDFlag_ = false;

	// Low trigger
	if ((signal < lowLevel_) && !lowEdgeFlag_)
	{
		lowEdge = true;
		lowEdgeFlag_ = true;
		lowStart_ = cur_ms;
		latchLow = !latchLow; 				// Toggle latchOff
	}
	if (signal > lowLevel_ + hysteresis_)	// 10%
		lowEdgeFlag_ = false;
	//
	low = (signal < lowLevel_);
	lowD = lowEdgeFlag_ && ((cur_ms - lowStart_) > lowDelay_);
	//
	if (lowD && !lowEdgeDFlag_)
	{
		lowEdgeD = true;
		lowEdgeDFlag_ = true;
	}
	if (!lowD)
		lowEdgeDFlag_ = false;
}

// Version DIYGuy999
/*
// on off function (true, if > on, false if < off) ************************************************************
bool chTrigger::onOff(unsigned int pulsewidth, unsigned int on, unsigned int off) {
	_onOffPulsewidth = pulsewidth;
	_onOffOn = on;
	_onOffOff = off;

	if (_onOffPulsewidth > _onOffOn) _onOffState = true;
	if (_onOffPulsewidth < _onOffOff) _onOffState = false;

	return _onOffState;
}

// window function (true, if > min & < max) ************************************************************
bool chTrigger::window(unsigned int pulsewidth, unsigned int min, unsigned int max) {
	_windowPulsewidth = pulsewidth;
	_windowMin = min;
	_windowMax = max;

	if (_windowPulsewidth > _windowMin && _windowPulsewidth < _windowMax) _windowState = true;
	else _windowState = false;

	return _windowState;
}

// Momentary function (true, if pressed) ************************************************************
bool chTrigger::momentary(unsigned int pulsewidth, unsigned int target) {
	_momentaryPulsewidth = pulsewidth;
	_momentaryTarget = target;

	if (millis() - _momentaryPreviousMillis > 20) { // every 20ms
		_momentaryPreviousMillis = millis();
		if (_momentaryPulsewidth > 0) {
			if (_momentaryPulsewidth >= _momentaryTarget - 50 && _momentaryPulsewidth <= _momentaryTarget + 50) _momentaryState = true; // Range OK
			else _momentaryState = false;
		}
		_momentaryPulsewidthPrevious = _momentaryPulsewidth;
	}

	return _momentaryState;
}

// Toggle function (toggling true / false, if pressed shorter than "duration") ************************************************************
bool chTrigger::toggle(unsigned int pulsewidth, unsigned int target) {
	_togglePulsewidth = pulsewidth;
	_toggleTarget = target;
	_toggleDuration = _duration;

	if (millis() - _togglePreviousMillis > 20) { // every 20ms
		_togglePreviousMillis = millis();
		if (_togglePulsewidth >= _toggleTarget - 50 && _togglePulsewidth <= _toggleTarget + 50 // Range OK
			&& _togglePulsewidth >= _togglePulsewidthPrevious - 10 && _togglePulsewidth <= _togglePulsewidthPrevious + 10) _toggleTargetOk = true; // stable reading
		else _toggleTargetOk = false;

		_togglePulsewidthPrevious = _togglePulsewidth;
	}

	switch (_toggleStep) {
		case 0: //---- Step 0 (do nothing)
			_toggleStep = 1;
			break;

		case 1: //---- Step 1 (if reading is in target area)
			if (_toggleTargetOk) {
				_toggleStartTime = millis();
				_toggleStep = 2;
			}
			break;

		case 2: //---- Step 2 (if reading is not in target range anymore)
			if (!_toggleTargetOk) {
				if (millis() - _toggleStartTime < _toggleDuration) _toggleState = !_toggleState;
				_toggleStep = 0;
			}
			break;
	}

	return _toggleState;
}

// Toggle long function (toggling true / false, if pressed longer than "duration") ************************************************************
bool chTrigger::toggleLong(unsigned int pulsewidth, unsigned int target) {
	_toggleLongPulsewidth = pulsewidth;
	_toggleLongTarget = target;
	_toggleLongDuration = _duration;

	if (millis() - _toggleLongPreviousMillis > 20) { // every 20ms
		_toggleLongPreviousMillis = millis();
		if (_toggleLongPulsewidth >= _toggleLongTarget - 50 && _toggleLongPulsewidth <= _toggleLongTarget + 50 // Range OK
			&& _toggleLongPulsewidth >= _toggleLongPulsewidthPrevious - 10 && _toggleLongPulsewidth <= _toggleLongPulsewidthPrevious + 10) _toggleLongTargetOk = true; // stable reading
		else _toggleLongTargetOk = false;

		_toggleLongPulsewidthPrevious = _toggleLongPulsewidth;
	}

	switch (_toggleLongStep) {
		case 0: //---- Step 0 (do nothing)
			_toggleLongStep = 1;
			break;

		case 1: //---- Step 1 (if reading is in target area)
			if (_toggleLongTargetOk) {
				_toggleLongStartTime = millis();
				_toggleLongStep = 2;
			}
			break;

		case 2: //---- Step 2 (if time elapsed reading still needs to be in target area)
			if (millis() - _toggleLongStartTime > _toggleLongDuration) {
				if (_toggleLongTargetOk) _toggleLongState = !_toggleLongState;
				_toggleLongStep = 3;
			}
			break;

		case 3: //---- Step 3 (if reading is not in target range anymore)
			if (!_toggleLongTargetOk) {
				_toggleLongStep = 0;
			}
			break;
	}

	return _toggleLongState;
}
*/

//
// EOF
//