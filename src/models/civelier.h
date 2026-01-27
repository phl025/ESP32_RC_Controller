//
// Model definition : Civelier
//
#ifndef MODEL_H
#define MODEL_H

// Configuration name
#define CONFIGURATION "Civellier, TX16S + RP3-H / CRSF 16ch"

// Sound model
#include "vehicles/Kifanlo.h"

// Remote control : RadioMaster TX16S + RP3-H
#define RX_CHANNELS 16

// 3-positions Multiswitch 
#define MIX3P 10		// Use channel 10

// 8 buttons Multiswitch 
//#define MIX4  10		// Use channel 10

// CRSF communication
#define CRSF_COMMUNICATION          // OK 2025-04-15 (by AlfredoCRSF)

// SBUS communication (RX header, 13 channels. This is my preferred communication protocol)--------
//#define SBUS_COMMUNICATION          // control signals are coming in via the SBUS interface (comment it out for classic PWM RC signals)
//#define EMBEDDED_SBUS               // Embedded SBUS code is used instead of SBUS library, if defined (removed)
boolean sbusInverted = true;        // true = standard (non inverted) SBUS signal
uint32_t sbusBaud = 100000;         // Standard is 100000. Try to lower it, if your channels are coming in unstable. Working range is about 96000 - 104000.
uint16_t sbusFailsafeTimeout = 100; // Failsafe is triggered after this timeout in milliseconds (about 100)

// PPM communication (RX header, 8 channels, working fine, but channel signals are a bit jittery)
//#define PPM_COMMUNICATION           // control signals are coming in via the PPM interface (comment it out for classic PWM RC signals)
#ifdef PPM_COMMUNICATION
#define RX_CHANNELS 8
#endif

// PWM Mode
//#define PWM_COMMUNICATION			// Defined by default
//#define PWM_GPIO					// Use GPIO_ISR, not RTM (Prefered - Removed)
#ifdef PWM_COMMUNICATION
// Channel inputs
#if RX_CHANNELS == 4
uint8_t PWM_CHANNELS[RX_CHANNELS] = {1, 2, 3, 4};	 		// Channel numbers
uint8_t PWM_PINS[RX_CHANNELS] = {13, 12, 14, 27};			// Input pin numbers (pin 34 & 35 only usable as inputs!)
//
#elif RX_CHANNELS == 6
uint8_t PWM_CHANNELS[RX_CHANNELS] = {1, 2, 3, 4, 5, 6};		// Channel numbers
uint8_t PWM_PINS[RX_CHANNELS] = {13, 12, 14, 27, 35, 34};	// Input pin numbers (pin 34 & 35 only usable as inputs!)
#endif
#endif

// Channel assignment (use NONE for non existing channels!)
#define STEERING 1           // CH1 steering
#define THROTTLE 3           // CH3 throttle & brake

/*
// Remote channel #######    // Sound controller channel ##########################################
#define STEERING 1           // CH1 steering
#define GEARBOX NONE         // CH2 3 position switch for gearbox (left throttle in tracked mode)
//#define GEARBOX 3          // CH2 3 position switch for gearbox (left throttle in tracked mode)
#define THROTTLE 3           // CH3 throttle & brake (right throttle in tracked mode)
#define HORN 2               // CH4 horn and bluelight / siren
#define FUNCTION_R 5         // CH5 jake brake, high / low beam, headlight flasher, engine on / off
#define FUNCTION_L 6         // CH6 indicators, hazards
#define POT2 NONE            // CH7 pot 2
#define MODE1 NONE           // CH8 mode 1 switch
#define MODE2 NONE           // CH9 mode 2 switch
#define MOMENTARY1 NONE      // CH10
#define HAZARDS NONE         // CH11
#define INDICATOR_LEFT NONE  // CH12
#define INDICATOR_RIGHT NONE // CH13
*/

// Sound trigger
// channel[x].trigger : low / high / lowEdge / highEdge / lowD / highD / lowEdgeD / highEdgeD / latchLow / latchHigh (D=Delayed)
#define MASTER_VOL_TRIG rx_data->channel[5].trigger.highEdgeD
//
#if defined(MIX3P) || defined(MIX4)
  #define HORN_TRIG 	rx_data->channel[MIX3P].getSwitchBit(0)
  #define SOUND1_TRIG 	rx_data->channel[MIX3P].getSwitchBit(1)
  //#define HORN_TRIG 	rx_data->getMultiSwitch(0)		
  //#define SOUND1_TRIG 	rx_data->getMultiSwitch(1)
#else
  #define HORN_TRIG 	rx_data->channel[10].trigger.lowEdge
  #define SOUND1_TRIG 	rx_data->channel[10].trigger.highEdge
#endif
#define SOUND2_TRIG		rx_data->channel[6].trigger.high || rx_data->channel[6].trigger.low

// Leds / Output command -> See below 'modelOutput'
// Ouputs, ULN2003 (IC2) : 23 (3x), 22, 3, 21, 19 
// Ouputs, ULN2003 (IC3) : 18, 5, 17, 16, 4, 2, 15 
//#define PIN_OUT1		18
//#define PIN_OUT2		5
//#define OUT1_CMD		rx_data->channel[7].trigger.latchLow
//#define OUT2_CMD		rx_data->channel[7].trigger.latchHigh



// Channels reversed or not
bool channelReversed[17] = {
	false,	// CH0 (unused) -> used for failsafe (PPM)
	false,	// CH1
	false,	// CH2
	false,	// CH3
	false,	// CH4
	false,	// CH5
	false,	// CH6
	false,	// CH7
	false,	// CH8
	false,	// CH9
	false,	// CH10
	false,	// CH11
	false,	// CH12
	false,	// CH13
	false,	// CH14
	false,	// CH15
	false	// CH16
};

// Channels FailSafe configuration [1000..1500..2000] : Channel position, [0] : OFF, [-1] : last good value
int channelFailSafe[17] = {
	0,		// CH0 (unused) -> used for failsafe (PPM)
	1501,	// CH1
	1502,	// CH2
	1503,	// CH3
	1504,	// CH4
	1505,	// CH5
	1500,	// CH6
	1500,	// CH7
	1500,	// CH8
	1500,	// CH9
	1020,	// CH10	-- 1020 : 0, Mix3P
	1500,	// CH11
	1500,	// CH12
	1500,	// CH13
	1500,	// CH14
	1500,	// CH15
	1500	// CH16
};

// Output channels definition (Not avalable for PWM mode) [1..6]: PWM1 .. PWM6
int PwmOutputChannel[7] = {
	0,		// CH0 (unused)
	1,		// CH1 or ESC3
	2,		// CH2 or ESC3
	6,		// CH3 or ESC2, Channel 6 (SB + FM6)	
	7,		// CH4 or ESC2, Channel 7 (SC + FM6)	
	0,		// CH5 or ESC1
	0,		// CH6 or ESC1
};

// Channels intergration time, ms for [1000 to 2000], initale value : 100 ms, 2000ms for ESC
int channelIntregrator[17] = {
	0,		// CH0 (unused) -> used for failsafe (PPM)
	100,	// CH1
	100,	// CH2
	2000,	// CH3
	100,	// CH4
	100,	// CH5
	100,	// CH6
	100,	// CH7
	100,	// CH8
	100,	// CH9
	100,	// CH10
	100,	// CH11
	100,	// CH12
	100,	// CH13
	100,	// CH14
	100,	// CH15
	100		// CH16
};

//
// ESC
//
#define ESC1_DRIVER
//#define ESC2_DRIVER
//#define ESC3_DRIVER
//

// ESC 1, type RZ7886
#ifdef ESC1_DRIVER
uint16_t ESC1_FREQUENCY = 500;	// 500 Hz is recommended. It is not audible, if virtual engine sound is running. Higher frequencies may overheat the driver IC!
uint8_t ESC1_DRAGBRAKE_DUTY = 10;
//
#define ESC1_CH		3	// ESC1, Channel number
#define ESC1_PIN1 	33 	// ESC1, RZ7886 motor driver pin 1 
#define ESC1_PIN2 	32 	// ESC1, RZ7886 motor driver pin 2
//
#else
#define ESC1_CH		0	// ESC1, Channel number
#endif

// ESC 2, type RZ7886
#ifdef ESC2_DRIVER
uint16_t ESC2_FREQUENCY = 500;	// 500 Hz is recommended. It is not audible, if virtual engine sound is running. Higher frequencies may overheat the driver IC!
uint8_t ESC2_DRAGBRAKE_DUTY = 10;
//
#define ESC2_CH		4	// ESC2, Channel number
#define ESC2_PIN1 	14 	// ESC2, RZ7886 motor driver pin 1 
#define ESC2_PIN2 	27 	// ESC2, RZ7886 motor driver pin 2
#else
#define ESC2_CH		0	// ESC2, Channel number
#endif

// ESC 3, type RZ7886
#ifdef ESC3_DRIVER
uint16_t ESC3_FREQUENCY = 500;	// 500 Hz is recommended. It is not audible, if virtual engine sound is running. Higher frequencies may overheat the driver IC!
uint8_t ESC3_DRAGBRAKE_DUTY = 10;
//
#define ESC3_CH		2	// ESC3, Channel number
#define ESC3_PIN1 	13 	// ESC3, RZ7886 motor driver pin 1 
#define ESC3_PIN2 	12 	// ESC3, RZ7886 motor driver pin 2
#else
#define ESC3_CH		0	// ESC3, Channel number
#endif

// Model output
// #define OUTPUTS		12
// uint8_t OUTPUT_PINS[OUTPUTS] = {18, 5, 17, 16, 4, 2, 15, 23, 22, 3, 21, 19};
// Hardware IC3 : [0] 18:Sidelight, [1] 5:Rooflight, [2] 17:Reversing, [3] 16:Foglight, [4] 4:Indicator_R, [5] 2:Indicator_L, [6] 15:TailLight
// Hardware IC2 : [7] 23:Shaker (3x), [8] 22:Cabinlight, [9] 3:Headlight, [10] 21:Beaconlight1, [11] 19:Beaconlight2

// Outputs : Defined for the model
inline void modelOutput(RxChannel *rx_channel, OutputLED *leds, bool rxReady, bool engineStarting)
{
	uint8_t crankingDim = 0;
	// Brigthness adjustment
	uint8_t brightness = map(rx_channel[13].rx, 988, 2012, 0, 255);

	// Engine starting dimming 
	if (engineStarting) crankingDim = 1;	// 1 = 50%, 2 = 25%

	// Indicator R [4] pin 'D4'
	if (rx_channel[STEERING].trigger.low)
		leds[4].flash(250, 250, 0, 0, 0, 255, 0, 0, 390);	// 390 us -> ~100ms
	else
		leds[4].pwm(0, 0, 400);		// 400 us -> ~100ms
		//leds[4].off(0, 400);		// 400 us -> ~100ms

	// Indicator L [5] pin 'D2'
	if (rx_channel[STEERING].trigger.high)
		leds[5].flash(250, 250, 0, 0, 0, 255, 0, 0, 390);
	else
		leds[5].pwm(0, 0, 400);		// 400 us -> ~100ms
		//leds[5].off(0, 400);
		
	// Fogligth [3] pin 'D16' : Fishing light
	if(rx_channel[MIX3P].getSwitchLatch(6) && rxReady)
	//if (rx_channel[8].trigger.latchLow && rxReady)
		leds[3].pwm(255 - crankingDim, 0, 0);
		//leds[3].pwm(brightness >> crankingDim, 0, 0);		// No ramp
	else
		leds[3].pwm(0, 0, 400);		// = leds[3].off(0, 400);

	// Roofligth [1] pin 'D5' : Fishing light (cabine)
	if(rx_channel[MIX3P].getSwitchLatch(7) && rxReady)
	//if (rx_channel[8].trigger.latchHigh && rxReady)
		//leds[1].pwm(255 - crankingDim, 0, 0);
		leds[1].pwm((brightness >> crankingDim), 0, 0);	// No ramp
	else
		leds[1].pwm(0, 0, 400);		// 400 us -> ~100ms

	// Cabineligth [8] pin'D22'
	if (rxReady)
		//leds[8].pwm(255 - crankingDim);
		leds[8].pwm(255 >> crankingDim);			// No ramp
	else
		//leds[8].off(0, 1000);		// 1000 -> 1000*256us = 256ms
		leds[8].pwm(0, 0, 400);		// 400 us -> ~100ms
		
}

/*
// Channels auto zero adjustment or not (don't use it for channels without spring centered neutral position, switches or unused channels)
boolean channelAutoZero[17] = {
	false, // CH0 (unused)
	false, // CH1
	false, // CH2
	false, // CH3
	false, // CH4
	false, // CH5
	false, // CH6
	false, // CH7
	false, // CH8
	false, // CH9
	false, // CH10
	false, // CH11
	false, // CH12
	false, // CH13
	false, // CH14
	false, // CH15
	false  // CH16
};

// Channels signal range calibration -----
//const uint16_t pulseNeutral = 25;   //50;
//const uint16_t pulseSpan = 450;

// Automatic or manual modes -----
// #define AUTO_LIGHTS
// #define AUTO_ENGINE_ON_OFF
// #define AUTO_INDICATORS
*/

#endif
//
// EOF
//