//
// Model definition : Boulogne Etaple
//
#ifndef MODEL_H
#define MODEL_H

// Configuration name
#define CONFIGURATION "TX16S + ER4 / PWM 4ch / Boulogne Etaple"

// Sound model
#include "vehicles/Kifanlo.h"
// Sound trigger
// channel[x].trigger : low / high / lowEdge / highEdge / lowD / highD / lowEdgeD / highEdgeD / latchLow / latchHigh (D=Delayed)
#define MASTER_VOL_TRIG false		//rx_data->channel[5].trigger.highEdgeD
//#define HORN_TRIG 		rx_data->channel[2].trigger.lowEdge
#define HORN_TRIG 		rx_data->channel[2].trigger.low
#define HORN_LATCH 		rx_data->channel[2].trigger.low
#define SOUND1_TRIG 	rx_data->channel[2].trigger.highEdge
#define SOUND2_TRIG		false		//"rx_data->channel[6].trigger.high || rx_data->channel[6].trigger.low"

// Leds / Output command
// Ouputs, ULN2003 (IC2) : 23 (3x), 22, 3, 21, 19 
// Ouputs, ULN2003 (IC3) : 18, 5, 17, 16, 4, 2, 15 
//#define OUTPUTS		7
//uint8_t OUTPUT_PINS[7] = {18, 5, 17, 16, 4, 2, 15};
/*
typedef enum {
    SIDE_L = 18,
    ROOF_L = 5,
	REVERSING_L = 17,
	FOG_L = 16,
	INDICATOR_R = 4,
	INDICATOR_L = 2,
	TAIL_L = 15
    SHAKER = 23,
    CABIN_L = 22,
    HEAD_L = 3,
    BEACON_L1 = 21,
	BEACON_L2 = 19,
} OutputName_t;
*/
//#define OUTPUTS		12
uint8_t OUTPUT_PINS[OUTPUTS_MAX] = {18, 5, 17, 16, 4, 2, 15, 23, 22, 3, 21, 19};
// Hardware IC3 : 18:Sidelight, 5:Rooflight, 17:Reversing, 16:Foglight, 4:Indicator R, 2:Indicator_L, 15:TailLight
// Hardware IC2 : 23:Shaker (3x), 22:Cabinlight, 3:Headlight, 21:Beaconlight, 19:Beaconlight2
#define PIN_OUT1		18
#define PIN_OUT2		5
#define OUT1_CMD		rx_data->channel[4].trigger.latchLow
#define OUT2_CMD		rx_data->channel[4].trigger.latchHigh
//uint8_t cabLightsBrightness = 100; // Usually 255, 100 for Actros & Ural

// Remote control : RadioMaster TX16S + ER4 (4xPWM) : 
//#define RX_CHANNELS 4
// Remote control : RadioMaster TX16S + ER6 (6xPWM) : All channels are 'ON' almost at the same time, not CH1 then CH2 then CH3 ....
#define RX_CHANNELS 6

// PWM Mode
#define PWM_COMMUNICATION	// For more parameters, see global.h

// Channel inputs
#if RX_CHANNELS == 4
uint8_t PWM_CHANNELS[RX_CHANNELS] = {1, 2, 3, 4};	 		// Channel numbers
uint8_t PWM_PINS[RX_CHANNELS] = {13, 12, 14, 27};			// Input pin numbers (pin 34 & 35 only usable as inputs!)

#elif RX_CHANNELS == 6
uint8_t PWM_CHANNELS[RX_CHANNELS] = {1, 2, 3, 4, 5, 6};		// Channel numbers
uint8_t PWM_PINS[RX_CHANNELS] = {13, 12, 14, 27, 35, 34};	// Input pin numbers (pin 34 & 35 only usable as inputs!)

#endif

//#define PWM_LOCAL
// PWM RC signal input pins (active, if no other communications profile is enabled) -----
// Channel numbers may be different on your recveiver!
// CH1: (steering)
// CH2: (gearbox) (left throttle in TRACKED_MODE)
// CH3: (throttle) (right throttle in TRACKED_MODE)
// CH4: (horn and bluelight / siren)
// CH5: (high / low beam, transmission neutral, jake brake etc.)
// CH6: (indicators, hazards)
//#define PWM_CHANNELS_NUM 6											// Number of PWM signal input pins 6
//uint8_t PWM_CHANNELS[PWM_CHANNELS_NUM] = {1, 2, 3, 4, 5, 6};	 	// Channel numbers
//uint8_t PWM_PINS[PWM_CHANNELS_NUM] = {13, 12, 14, 27, 35, 34};		// Input pin numbers (pin 34 & 35 only usable as inputs!)

// Channel assignment (use NONE for non existing channels!)
#define STEERING 1			// CH1 steering
#define THROTTLE 3			// CH3 throttle & brake

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
	1500,	// CH1
	1500,	// CH2
	1500,	// CH3
	1500,	// CH4
	1000,	// CH5
	1500,	// CH6
	0,		// CH7
	0,		// CH8
	0,		// CH9
	0,		// CH10
	0,		// CH11
	0,		// CH12
	0,		// CH13
	0,		// CH14
	0,		// CH15
	0		// CH16
};

// Output channels definition (Not avalable for PWM mode) [1..6]: PWM1 .. PWM6
int OutputChannel[7] = {
	0,		// CH0 (unused)
	0,		// CH1 or ESC3
	0,		// CH2 or ESC3
	0,		// CH3 or ESC2
	0,		// CH4 or ESC2
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
#else
#define ESC1_CH		0	// ESC1, Channel number
#endif

// ESC 2
#define ESC2_CH		0	// ESC2, Channel number (not used)

// ESC 3
#define ESC3_CH		0	// ESC3, Channel number (not used)

// Model output
//#define OUTPUTS		12
//uint8_t OUTPUT_PINS[OUTPUTS] = {18, 5, 17, 16, 4, 2, 15, 23, 22, 3, 21, 19};
// Hardware IC3 : [0] 18:Sidelight, [1] 5:Rooflight, [2] 17:Reversing, [3] 16:Foglight, [4] 4:Indicator R, [5] 2:Indicator_L, [6] 15:TailLight
// Hardware IC2 : [7] 23:Shaker (3x), [8] 22:Cabinlight, [9] 3:Headlight, [10] 21:Beaconlight, [11] 19:Beaconlight2
//
// For outputModel()
#include "src_ext/statusLED.h"

// Outputs : Defined for the model
inline void modelOutput(RxChannel *rx_channel, statusLED *leds)
{
	// Indicator R [4], pin 'D4'
	if (rx_channel[STEERING].trigger.low)
		leds[4].flash(375, 375, 0, 0, 0, 250, 0);
	else
		leds[4].off(250, 0);

	// Indicator L [5], pin 'D2'
	if (rx_channel[STEERING].trigger.high)
		leds[5].flash(375, 375, 0, 0, 0, 250, 0);
	else
		leds[5].off(250, 0);
}

#endif
//
// EOF
//