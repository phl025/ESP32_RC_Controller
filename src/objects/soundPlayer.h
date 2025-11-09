//
// soundPlayer.h : Output command
//
// Author : PHL @2025

#ifndef SOUND_H
#define SOUND_H
//
#include <stdint.h>
#include <esp_attr.h>

// Class definition (header)
class SoundPlayer
{
	// Data
private:
	const signed char *_samples;
	uint32_t _sampleRate = 22050; // Default : 22050Hz
	uint32_t _sampleCount = 0;	  // Default : no sound
	uint32_t _currentSample = 0;
	int32_t _volumePercentage = 20;

public:
	// bool state = false;

	// Methode
public:
	SoundPlayer(const signed char *p_samples, uint32_t sampleRate, uint32_t sampleCount, uint32_t volumePercentage);
	//
	int32_t IRAM_ATTR play(void);
	int32_t IRAM_ATTR play(volatile bool &trigger);
	int32_t IRAM_ATTR playLoop(volatile bool &trigger, volatile bool &latch, uint32_t LoopBegin, uint32_t loopEnd);
	//
	uint32_t getCurrentSample() const { return _currentSample; }
};

#endif
//
// EOF
//