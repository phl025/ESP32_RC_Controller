//
// Global definitions
//

// For all modes
#define MAX_CHANNELS	16		// CRSF, SBUS...

// PPM Rx
#define NUM_OF_PPM_AVG 	 1	
#define MAX_PPM_CHANNELS 8

// PWM Rx
#define MAX_PWM_CHANNELS 6		// Input CH1 to CH6
//#define ISR_INSIDE			// Interrupt on RX_PWM class (Prefered) - OK (2025-11)
//#define PWM_GPIO_LOCAL        // Use GPIO_ISR (GPIO ISR defined on main.cpp - Removed)
//#define PWM_RTM               // Use RTM (NOK - Removed)

// PPM - PWM
#define FAILSAFE_CHANNEL 3

// CRSF
//#define MAX_CRSF_CHANNELS 16

// Serial command pins for SBUS, IBUS, PPM, SUMD -----
#define COMMAND_RX 		36		// pin 36, labelled with "VP", connect it to "Micro RC Receiver" pin "TXO"
#define COMMAND_TX 		-1		// pin -1 : for UART macro UART_PIN_NO_CHANGE (see "driver/uart.h")

// RX : SBUS Mode
#define EMBEDDED_SBUS

// Servo frequency
#define SERVO_FREQUENCY	50      // usually 50Hz, some servos may run smoother @ 100Hz

// Outputs
#define OUTPUTS_MAX		12

//
// EOF
//