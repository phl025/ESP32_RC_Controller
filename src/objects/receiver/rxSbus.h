//
// rxSbus.h : RX_Reciver class for SBUS
//
// Author : PHL @2025

#ifndef RX_SBUS_H
#define RX_SBUS_H

// Includes
#include <HardwareSerial.h>
#include <src_ext\sbus.h>

// Class definition (header)
class RxSbus : public RxBase
{
// Data
protected:
	//volatile uint32_t timelast_ = 0;
	
// Methode
public:
	RxSbus(int in_channels);
	RxSbus(int in_channels, int *p_channelFailSafe, int* p_chIntegrator, bool *p_channelReversed);
    //
	void updateChannels(uint32_t cur_us) override;
	//
	uint8_t getBuffer(int index);
	uint8_t getBufferLen(void);
	//inline uint_8 getBufferLen() const {return _sBus.getBufferLen();}

};
#endif

//
// EOF
//
