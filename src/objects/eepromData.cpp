//
// eepromData.cpp : EEPROM Data
//
// Author : PHL @2025

// Includes
#include "eepromData.h"
// 
#include <EEPROM.h>

// Class definition (methodes)

/// @brief Construct a new OutputPwmData class
OutputPwmData::OutputPwmData(){};	

/// @brief Construct a new EEpromData object
EEpromData::EEpromData(uint16_t eeprom_size)
{
	_eepromSize = eeprom_size;
	EEPROM.begin(_eepromSize);		// EEPROM_SIZE;
};


/// @brief Destroy the EEpromData object.
EEpromData::~EEpromData()
{

};


void EEpromData::Reset(uint16_t size, uint8_t value)
{
	if (size > _eepromSize)
		size = _eepromSize;
	for (int16_t i = 0; i < size; i++)
	{
		EEPROM.write(i, 0);
	}
	EEPROM.commit();
}

void EEpromData::Read()
{
	int address = 0;
	//
	_eepromId = EEPROM.read(address);
	//
	for (int i = 0; i < OUTPUTS_MAX; i++)          // 1-2, 3-4, ...
	{
		address = i * 2;
		_led[i].on = EEPROM.read(address + 1);
		_led[i].off = EEPROM.read(address + 2);
	}
	//
	Serial.println("EEPROM read.");
}

void EEpromData::Write()
{
	int address = 0;
	//
	for (int i = 0; i < OUTPUTS_MAX; i++)          // 1-2, 3-4, ...
	{
		address = i * 2;
		_led[i].on = 255;
		_led[i].off = 0;
		EEPROM.write(address + 1, _led[i].on);
		EEPROM.write(address + 2, _led[i].off);
	}
	//
	EEPROM.commit();
	Serial.println("EEPROM written.");
}

void EEpromData::Init(uint8_t eeprom_id)
{
	int address = 0;
	//
	if (EEPROM.read(0) != eeprom_id)
	{ // Only do init, if ID has changed!
		EEPROM.write(address, eeprom_id);       // 0
		//
		for (int i = 0; i < OUTPUTS_MAX; i++)          // 1-2, 3-4, ...
		{
			address = i * 2;
			_led[i].on = 255;
			_led[i].off = 0;
			EEPROM.write(address + 1, _led[i].on);
			EEPROM.write(address + 2, _led[i].off);
		}
		//
		EEPROM.commit();
		Serial.println("EEPROM initialized.");
	}
}

void EEpromData::Debug()
{
	int address = 0;
	//
	Serial.print("EEPROM read data :\n");
	Serial.printf("eepromId = %d\n", EEPROM.read(address));
	//
	for (int i = 0; i < OUTPUTS_MAX; i++)          // 1-2, 3-4, ...
	{
		address = i * 2;
		Serial.printf("Led[%02d] = %01x - %01x\n", i, EEPROM.read(address+1), EEPROM.read(address + 2));
	}
	//
	Serial.print("EEPROM read data : END\n");
}

//
// EOF
//