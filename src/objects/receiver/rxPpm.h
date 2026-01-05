//
// RxPpm.cpp : RX_Reciver class for PPM
//
// Author : PHL @2025

#ifndef RX_PPM_H
#define RX_PPM_H

#include <Arduino.h>
//
#define BUFFER_SIZE 4	// Use 2, 4 or 8

// Class definition (header)
class RxPpm : public RxBase
{
// Data
protected:
	// Static for ISR

	volatile unsigned int ppm_buf_[MAX_PPM_CHANNELS+1]; 
	volatile unsigned int ppm_in_[MAX_PPM_CHANNELS+1];
	volatile short counter_ = 0;
	volatile short average_cnt_ = 0;
	volatile bool new_ppm_data_ = false;

private:
	volatile uint16_t ppmBuffer_ [MAX_PPM_CHANNELS+1][BUFFER_SIZE+1];
	
// Methode
public:
	RxPpm(int in_channels, short in_average);
	RxPpm(int in_channels, short in_average, int *p_channelFailSafe, int* p_chIntegrator, bool *p_channelReversed);
	//
	void IT_Ppm(uint32_t cur_us);
	void updateChannels(uint32_t cur_us) override;
	//
	uint32_t GetPpmIn(int channel);

private:
	uint16_t bufferUpdate(int channel, uint16_t value);

protected:
	//static void IRAM_ATTR gpio_isr();


};
#endif

//
// EOF
//
