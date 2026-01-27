//
// eepromData.h : EEPROM Data class
//
// Author : PHL, 2025

#ifndef EEPROM_DATA_H
#define EEPROM_DATA_H

// Includes
#include <stdint.h>
#include "global.h"

// 
//#define LEDS 12
//#define EEPROM_SIZE 256		// Max 512 !

// OutputPwmData Class
class OutputPwmData
{
// Data
public:
	uint8_t on;
	uint8_t off;

/*	
protected:
	uint8_t _onValue;
	uint8_t _offValue;
*/
// Methode
public:
	OutputPwmData ();
};

/// @brief OutputPwmData class creator
//OutputPwmData::OutputPwmData(){};	



// Class definition (header)
class EEpromData
{
// Data
public:
	OutputPwmData _led[OUTPUTS_MAX];

private:
	uint8_t _eepromId;
	uint16_t _eepromSize;

// Methode
public:
	// Methode
	EEpromData(uint16_t size);
	~EEpromData();
	//
	void Reset(uint16_t size, uint8_t value = 0);
	//
	void Read();
	void Write();
	void Init(uint8_t eeprom_id);
	void Debug();
	//

};

#endif
//
// EOF
//