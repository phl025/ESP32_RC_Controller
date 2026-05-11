//
// Global definitions
//
//#include "my_setup/TFT_eSPI_Setup.h"

// For all modes
#define MAX_CHANNELS	16		// CRSF, SBUS...

// PPM Rx
#define NUM_OF_PPM_AVG 	 1	
#define MAX_PPM_CHANNELS 8

// PWM Rx
#define MAX_PWM_CHANNELS 6		// Input CH1 to CH6

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

// Dashboard
#define USE_DASHBOARD
#ifdef USE_DASHBOARD
// TFT 0.96 inch, 80x160, 16 bit color, driver ST7735 : Config dans platformio.ini
//#define STD_DASHBOARD
//#define FREVIC_DASHBOARD
// TFT 1.3 inch, 240x240, 16 bit color, driver ST7789 : Config dans platformio.ini
#define NAVY_DASHBOARD
#endif

// EEPROM 
//#define EEPROM_SIZE     256 	// 256 Bytes (512 is maximum)


//
// EOF
//