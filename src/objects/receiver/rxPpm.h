//
// RxPpm.cpp : RX_Reciver class for PPM
//
// Author : PHL @2025

#ifndef RX_PPM_H
#define RX_PPM_H

// Class definition (header)
class RxPpm : public RxBase
{
// Data
protected:
	volatile unsigned int ppm_buf_[MAX_PPM_CHANNELS+1]; 
	volatile unsigned int ppm_in_[MAX_PPM_CHANNELS+1];
	volatile short counter_ = 0;
	volatile short average_cnt_ = 0;
	volatile bool new_ppm_data_ = false;
	
// Methode
public:
	RxPpm(int in_channels, short in_average);
	RxPpm(int in_channels, short in_average, int *p_channelFailSafe, int* p_chIntegrator, bool *p_channelReversed);
	//
	void IT_Ppm(uint32_t cur_us);
	void updateChannels(uint32_t cur_us) override;
	//
	uint32_t GetPpmIn(int channel);

	// Inline functions
	//inline uint32_t Getlast_good_read_() const {return last_good_read_;}
};
#endif

//
// EOF
//
