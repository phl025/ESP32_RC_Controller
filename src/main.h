//
// main.h
//
#ifndef MAIN_H
#define MAIN_H

//
// INCLUDES
//
#pragma region INCLUDE_ZONE
// #include <Arduino.h>
#include <esp_attr.h>
#include "esp32-hal-dac.h"
// For
// #include <WiFi.h>			// For Wifi, ESP-NOW
// #include <WifiEspNow.h>		// For ESP-NOW (wifi communication protocol - Espressif)

// No need to install these, they come with the ESP32 board definition
#include <driver/gpio.h>		// For GPIO
#include <driver/uart.h>		// for UART macro UART_PIN_NO_CHANGE
#include <driver/rmt.h>			// for PWM signal detection
#include <driver/mcpwm.h>		// for servo PWM output
#include <rom/rtc.h>			// for displaying reset reason
#include <soc/rtc_wdt.h>		// for watchdog timer
#include <string.h>

// Objects
#include "objects/global.h"
#include "objects/escDriver.h"			// ESC Driver object, type RZ7886 - PHL
#include "objects/receiver/rxBase.h"	// Rx object, base - PHL
#include "objects/receiver/rxPpm.h"		// Rx object, PPM input mode - PHL
#include "objects/receiver/rxPwm.h"		// Rx object, PMM input mode - PHL
#include "objects/receiver/rxSbus.h"	// Rx object, SBUS - PHL
#include "objects/receiver/rxCrsf.h"	// Rx object, CRSF - PHL
#include "objects/receiver/chTrigger.h"	// Trigger - PHL
//
#include "objects/iecTimer.h"			// IEC Timer - PHL
// #include "objects/output.h"			// Output - PHL
#include "objects/outputs.h"			// Outputs - PHL
#include "objects/soundPlayer.h"		// Sound player - PHL

// Ext_src : TheDIYGuy999 source code
#include "src_ext/statusLED.h"
// SBUS (Test idem RC_Engine_Sound_V9.13)
#if defined EMBEDDED_SBUS
#include "src_ext/sbus.h" // For SBUS interface
#endif

// Configuration : model
#include "0_Model.h"

#pragma endregion INCLUDE_ZONE

//
// Datas
//

// Battery
#define BATTERY_DETECT_PIN 39 // Voltage divider resistors connected to pin "VN & GND"

// Serial DEBUG pins -----
#define DEBUG_RX UART_PIN_NO_CHANGE // 99 is just a dummy, because the "RX0" pin (GPIO3) is used for the headlights and causing issues, if rx enabled! -1 (3 headlights)
#define DEBUG_TX 1					// The "RX0" is on pin 1
uint32_t lastloop_ms = 0;

// These are used to print the reset reason on startup
const char *RESET_REASONS[] = {"POWERON_RESET", "NO_REASON", "SW_RESET", "OWDT_RESET", "DEEPSLEEP_RESET", "SDIO_RESET", "TG0WDT_SYS_RESET", "TG1WDT_SYS_RESET", "RTCWDT_SYS_RESET", "INTRUSION_RESET", "TGWDT_CPU_RESET", "SW_CPU_RESET", "RTCWDT_CPU_RESET", "EXT_CPU_RESET", "RTCWDT_BROWN_OUT_RESET", "RTCWDT_RTC_RESET"};

// ESP32 : Eviter les allocations dynamiques
// Dynamic definitions
EscDriver *esc_1 = NULL; // ESC1, Driver type RZ7886
EscDriver *esc_2 = NULL; // ESC2, Driver type RZ7886
EscDriver *esc_3 = NULL; // ESC3, Driver type RZ7886
// Static definitions
/*
#ifdef ESC1_DRIVER 	// ESC Driver
	EscDriver esc_1(1, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, ESC1_PIN1, ESC1_PIN2, ESC1_FREQUENCY);
#endif
#ifdef ESC2_DRIVER
	EscDriver esc_2(2, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, ESC2_PIN1, ESC2_PIN2, ESC2_FREQUENCY);
#endif
#ifdef ESC3_DRIVER
	EscDriver esc_3(3, MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2A, MCPWM2B, ESC3_PIN1, ESC3_PIN2, ESC3_FREQUENCY);
#endif
*/

//
// Outputs
//
Outputs *leds = NULL;
// Output 	Leds[OUTPUTS];
// Output	led1(PIN_OUT1);
// Output	led2(PIN_OUT2);
// Output	Led3(PIN_OUT2);

// Test
// statusLED indicatorL(false);
// statusLED indicatorR(false);
// statusLED shakerMotor(false);

//
// Tempos
//
IecTimer TON_Pwm_0;

//
// Triggers -> Move to receiver->channel[]
//
/*
ChTrigger trigCh2(1800, 1000, 1200, 1000, 50);
ChTrigger trigCh3;		    // use default values (1800, 500, 1200, 500, 50)
ChTrigger trigCh4;			// use default values (1800, 500, 1200, 500, 50)
ChTrigger trigCh5(1800, 500, 1200, 250, 50);
*/

//
// SOUND AREA
//
#pragma region SOUND_AREA
// Master volume
volatile uint8_t volumeIndex = 2;
const uint8_t numberOfVolumeSteps = 4;					   // The mumber of volume steps below
const uint8_t masterVolumePercentage[] = {100, 66, 33, 0}; // loud, medium, silent, no sound (more than 100% may cause distortions)

// Throttle
int16_t currentThrottle = 0;	  // 0 - 500 (Throttle trigger input)
int16_t currentThrottleFaded = 0; // Faded throttle for volume calculations etc.

// Engine state
// const int16_t maxRpm = 500;       // always 500
// const int16_t minRpm = 0;         // always 0
int32_t currentRpm = 0;			 // 0 - 500 (signed required!) --- TO BE REMOVED
const int16_t maxPwm = 100;		 // always 100
const int16_t minPwm = 0;		 // always 0
volatile int16_t currentPwm = 0; // 0 - 100 (signed required!)
// int32_t targetHydraulicRpm[3];    // The hydraulic RPM target for loader mode
volatile uint8_t engineState = 0; // Engine state
enum EngineState				  // Engine state enum
{
	STOPPED,  // Engine is off
	START,	  // Start step
	STARTING, // Engine is starting
	RUN,	  // Run step
	RUNNING,  // Engine is running
	STOP,	  // Stop
	STOPPING, // Engine is stopping
	PARKING_BRAKE,
	PAUSE // for tests
};
// OFF,      	// Engine is off

int16_t engineLoad = 0;					// 0 - 500
volatile uint16_t engineSampleRate = 0; // Engine sample rate
// int32_t speedLimit = maxRpm;            // The speed limit, depending on selected virtual gear

// Sampling intervals for interrupt timer (adjusted according to your sound file sampling rate)
uint32_t maxSampleInterval = 4000000 / sampleRate;
uint32_t minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;

// Interrupt timer for variable sample rate playback (engine sound)
hw_timer_t *variableTimer = NULL;
portMUX_TYPE variableTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t variableTimerTicks = maxSampleInterval;

// Interrupt timer for fixed sample rate playback (horn etc., playing in parallel with engine sound)
hw_timer_t *fixedTimer = NULL;
portMUX_TYPE fixedTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t fixedTimerTicks = maxSampleInterval;

// DAC Fader
volatile uint8_t dacOffset = 128; // 128, but needs to be ramped up slowly to prevent popping noise, if switched on
/// static uint32_t dacOffsetMicros;
// volatile uint32_t dacOffsetMicros;
// volatile boolean dacInit = false;

// Sound trigger
volatile bool engineOn = false;				   // Signal for engine on / off
volatile bool hornTrigger = false;			   // Trigger for horn on / off
volatile bool sirenTrigger = false;			   // Trigger for siren  on / off
volatile bool sound1Trigger = false;		   // Trigger for sound1  on / off
volatile bool sound2Trigger = false;		   // Trigger for sound1  on / off
volatile bool sound3Trigger = false;		   // Trigger for sound1  on / off
volatile bool dieselKnockTrigger = false;	   // Trigger Diesel ignition "knock"
volatile bool dieselKnockTriggerFirst = false; // The first Diesel ignition "knock" per sequence
volatile bool indicatorSoundOn = false;		   // Active, if indicator bulb is on

// Sound latches
volatile bool hornLatch = false;  // Horn latch bit
volatile bool sirenLatch = false; // Siren latch bit
// volatile bool sound1Latch = false;	// Sound 1 latch bit

// Sound volumes
volatile int16_t masterVolume = 50; // Master volume percentage
//
volatile uint16_t throttleDependentVolume = 20;	   // engine volume according to throttle position
volatile uint16_t throttleDependentRevVolume = 20; // engine rev volume according to throttle position

// Sounds
// a
SoundPlayer horn(hornSamples, hornSampleRate, hornSampleCount, hornVolumePercentage);
SoundPlayer siren(sirenSamples, sirenSampleRate, sirenSampleCount, sirenVolumePercentage);
// b
SoundPlayer sound1(sound1Samples, sound1SampleRate, sound1SampleCount, sound1VolumePercentage);
// #ifdef reversingSamples
#if reversingSampleCount > 10
SoundPlayer reversing(reversingSamples, reversingSampleRate, reversingSampleCount, reversingVolumePercentage);
#endif
#if indicatorSampleCount > 10
SoundPlayer indicator(indicatorSamples, indicatorSampleRate, indicatorSampleCount, indicatorVolumePercentage);
#endif
#if wastegateSampleCount > 10
SoundPlayer wastegate(wastegateSamples, wastegateSampleRate, wastegateSampleCount, wastegateVolumePercentage);
#endif
#if brakeSampleCount > 10
SoundPlayer airBrake(brakeSamples, brakeSampleRate, brakeSampleCount, brakeVolumePercentage);
#endif
#if parkingBrakeSampleCount > 10
SoundPlayer parkingBrake(parkingBrakeSamples, parkingBrakeSampleRate, parkingBrakeSampleCount, parkingBrakeVolumePercentage);
#endif
#if shiftingSampleCount > 10
SoundPlayer shifting(shiftingSamples, shiftingSampleRate, shiftingSampleCount, shiftingVolumePercentage);
#endif
#ifdef COUPLING_SOUND
SoundPlayer coupling(couplingSamples, couplingSampleRate, couplingSampleCount, couplingVolumePercentage);
SoundPlayer uncoupling(uncouplingSamples, uncouplingSampleRate, uncouplingSampleCount, couplingVolumePercentage);
#endif
// c
#ifdef EXCAVATOR_MODE
SoundPlayer bukerRattle(bucketRattleSamples, bucketRattleSampleRate, bucketRattleSampleCount, bucketRattleVolumePercentage);
SoundPlayer hydraulicFlow(hydraulicFlowSamples, hydraulicFlowSampleRate, hydraulicFlowSampleCount, hydraulicFlowVolumePercentage);
SoundPlayer trackRattle(trackRattleSamples, trackRattleSampleRate, trackRattleSampleCount, trackRattleSampleVolumePerentage);
#endif
// d2
#ifdef BATTERY_PROTECTION
SoundPlayer outOfFuel(outOfFuelSamples, outOfFuelSampleRate, outOfFuelSampleCount, outOfFuelVolumePercentage);
#endif
// Add PHL
#ifdef SOUND2
SoundPlayer sound2(sound2Samples, sound2SampleRate, sound2SampleCount, sound2VolumePercentage);
#endif
#ifdef SOUND3
SoundPlayer sound3(sound3Samples, sound3SampleRate, sound3SampleCount, sound3VolumePercentage);
#endif

// Local functions
void IRAM_ATTR variablePlaybackTimer();
void IRAM_ATTR fixedPlaybackTimer();
void dacOffsetFade();
void VolumeSet();

// END SOUND AREA
#pragma endregion SOUND_AREA

// Multitasking
volatile uint32_t it_count = 0;
volatile int32_t coreLoop = -1;
volatile int32_t coreFT = -1;
volatile int32_t coreVT = -1;
volatile int32_t coreT1 = -1;
//
struct mtVariables_t
{
	int coreLoop = -1;
	int coreFT = -1;
	int coreVT = -1;
	int coreT1 = -1;
	int isrVT_Cnt = 0;
} mtVariables;

// For debug
volatile uint32_t bgdCount = 0;
uint16_t cpProccess = 0; // Proccess count
uint8_t debugLine = 0;
uint8_t debugCh = 0;

// For TCY Loop()
uint32_t lastLoop_us = 0;
uint32_t minTcy = 4294967285;
uint32_t maxTcy = 0;

// Receiver mode
#if defined CRSF_COMMUNICATION
// CRSF RX Communication
RxCrsf *rx_data;

#elif defined SBUS_COMMUNICATION
// SBUS RX Communication
RxSbus *rx_data;

#elif defined PPM_COMMUNICATION
// PPM RX Communication
RxPpm *rx_data;

#elif defined PWM_COMMUNICATION
// PWM Mode - TO DO..
RxPwm *rx_data;
// Declare static data
volatile uint8_t* RxPwm::pwmPin_ = PWM_PINS;
volatile unsigned int* RxPwm::pwmBuffer_ = new unsigned int [MAX_PWM_CHANNELS];
volatile bool RxPwm::new_isr_data_ = false;

#else
// Other, Not defined
RxBase *rx_data;
// End communication
#endif

#endif
//
// EOF
//