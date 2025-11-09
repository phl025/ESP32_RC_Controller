//
// soundPlayer.cpp : Sound player
//
// Author : PHL @2025

#include "soundPlayer.h"

// Class definition (methodes)

/// @brief SoundPlayer : Sound player class
/// @param p_samples Sample data
/// @param sampleRate Sample rate, 22050 by default
/// @param sampleCount Sample count
/// @param volumePercentage Volume
SoundPlayer::SoundPlayer(const signed char *p_samples, uint32_t sampleRate, uint32_t sampleCount, uint32_t volumePercentage)
{
	// Sound data
	_samples = p_samples;
	_sampleRate = sampleRate;
	_sampleCount = sampleCount;
	_volumePercentage = (int32_t)volumePercentage;
}


/// @brief Play sound continuously
/// @return Sample * volumePercentage
int32_t IRAM_ATTR SoundPlayer::play()
{
	static int32_t a;
	// Play sound
	if (_currentSample < _sampleCount - 1)
	{
		if (_volumePercentage == 100)
			a = (int32_t)_samples[_currentSample];
		else
			a = (int32_t)_samples[_currentSample] * _volumePercentage / 100;
		_currentSample++;
	}
	else
	{ // End of sample
		_currentSample = 0;
	}
	return a;
}


/// @brief Play sound by Trigger.
/// @param trigger Trigger : Start playing, reseted at end.
/// @return Sample * volumePercentage
int32_t IRAM_ATTR SoundPlayer::play(volatile bool &trigger)
{
	int32_t a;
	// Play sound
	if (trigger)
	{
		// fixedTimerTicks = 4000000 / hornSampleRate;			// our fixed sampling rate
		// timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // change timer ticks, autoreload true
		//
		if (_currentSample < _sampleCount - 1)
		{
			// a = _samples[_currentSample] * _volumePercentage / 100;
			if (_volumePercentage == 100)
				a = (int32_t)_samples[_currentSample];
			else
				a = (int32_t)_samples[_currentSample] * _volumePercentage / 100;
			_currentSample++;
		}
		else
		{ // End of sample
			trigger = 0;
			_currentSample = 0;
			a = 0;
		}
	}
	else
	{
		_currentSample = 0; // ensure, next sound will start @ first sample
		a = 0;
	}
	return a;
}


/// @brief Play sound with looping when trigger still presente.
/// @param trigger Trigger : Start playing, reseted at end.
/// @param loopBegin Loop : begin index
/// @param loopEnd Loop : End index
/// @return Sample * volumePercentage
int32_t IRAM_ATTR SoundPlayer::playLoop(volatile bool &trigger, volatile bool &latch, uint32_t loopBegin, uint32_t loopEnd)
{
	int32_t a;
	// Play sound
	if (trigger || latch)
	{
		if (_currentSample < _sampleCount - 1)
		{
			if (_volumePercentage == 100)
				a = (int32_t)_samples[_currentSample];
			else
				a = (int32_t)_samples[_currentSample] * _volumePercentage / 100;
			_currentSample++;
			// Loop, if trigger still present
			if (trigger && _currentSample == loopEnd)
				_currentSample = loopBegin;
		}
		else
		{ // End of sample
			latch = 0;
			_currentSample = 0;
			a = 0;
		}
	}
	else
	{
		_currentSample = 0; // ensure, next sound will start @ first sample
		a = 0;
	}
	return a;
}

//
// EOF
//