//
// rxCrsf.h : RX_Reciver class for CRSF
//
// Author : PHL @2025

#ifndef RX_CRSF_H
#define RX_CRSF_H

// Includes
#include <HardwareSerial.h>
#include "src_ext/AlfredoCRSF.h"

// Class definition (header)
class RxCrsf : public RxBase
{
// Data
protected:
	//volatile uint32_t timelast_ = 0;
	//volatile uint32_t last_good_read_ = 0;
	//volatile uint32_t it_count_ = 0;
	//volatile uint32_t rd_count_ = 0;
	
// Methode
public:
	RxCrsf(int in_channels);
	RxCrsf(int in_channels, int *p_channelFailSafe, int* p_chIntegrator, bool *p_channelReversed);
	//
	void updateChannels(uint32_t cur_us) override;
	//void updateChannels(uint32_t cur_us, AlfredoCRSF &crsf);
	uint8_t getBuffer(int index);
	uint8_t getBufferLen(void);

};
#endif

//
// EOF
//
