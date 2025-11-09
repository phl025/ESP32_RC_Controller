//
// EscDriver.h : EscDriver class, Driver for RC ESC
//
// Author : PHL @2025

#ifndef ESC_DRIVER_H
#define ESC_DRIVER_H
//
#include <driver/mcpwm.h> // for servo PWM output

// ESC Mode (AVR)
#define MODE_STOP 0
#define MODE_FWD 1
#define MODE_REV 2
#define MODE_EMG 99
// ESC ramps (AVR)
#define RAMP_ACC 10    // Accel ramp : acceleration neutre -> AV ou neutre -> AR
#define RAMP_DEC 10    // Decel ramp : decelleration AV ou AR vers neutre

// Class definition (header)
class EscDriver
{
// Data
private:
	int number_ = 1;
	mcpwm_unit_t unit_;				// PCPWM Unit number
	mcpwm_timer_t timer_;			// PCPWM Timer number
	mcpwm_io_signals_t chanelA_;	// PCPWM Output Pin A
	mcpwm_io_signals_t chanelB_;	// PCPWM Output Pin B
	uint8_t drag_brake_ = 10;		// Brake value (%)

	// For ESC ramp up/down
	uint16_t escRampTimeMs_ = 20;	// Time base for ESP ramp process (ms) (Ex for BOAT)
	uint16_t escRampAcc_ = 2000;	// Acceleration time ramp (ms), Example = 1000 = 1s
	uint16_t escRampDec_ = 2000;	// Decceleration time ramp (ms), Example = 1000 = 1s
	uint32_t lastEscMs_;		// Last ms call
	
	/*
	int _driveState;				// Driver status machine
	uint8_t _escRampTimeMs = 20;	// Time base for ESP management (ms) (BOAT_MODE)
	uint8_t _escRampAcc = 8;		// Acceleration rampe (µs), ~ 450 / escRampAcc * escRampTimeMs, Example = 450/9*20 -> 1s
	uint8_t _escRampDec = 8;		// Decceleration rampe (µs), ~ 450 / escRampDec * escRampTimeMs, Example = 450/9*20 -> 1s
	uint32_t _escMillis;
	uint32_t _lastDebugTime;
	*/

	// Type AVR
	// Signal configuration
	uint16_t neutre_ = 1500;		// Neutre position [1500]
	uint16_t neutre_min_ = 1460;	// Neutre mini [<1500]
	uint16_t neutre_max_ = 1540;	// Neutre maxi [>1500]
	uint16_t out_min_ = 25;			// Output mini [0.50%]
	uint16_t out_max_ = 100;		// Output maxi [50..100%]
	uint16_t val_max_ = 2000;     	// 1500 + 420 //450
	uint16_t val_min_ = 1000;     	// 1500 - 420 //450
	uint8_t  mode_ = MODE_EMG;		// Mode : EMG / STOP / FWD / REV
	// Status
	bool  neutre_ok_ = false;		// Neutre passed
	uint16_t last_value_;			// Last rx value
	uint16_t signal_pwm_ = 0;     	// Signal for PWM command [1000..2000]
	
	// Debug
	int motorPwm_;
	
// Methode
public:
	// Methode
	EscDriver(int number, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_io_signals_t chanelA, mcpwm_io_signals_t chanelB, int pinA, int pinB, uint32_t frequency);
	~EscDriver();
	//
	void setPwm(uint8_t pwmA, uint8_t pwmB);
    int getPwm();
	void update(uint16_t rx_value, bool debug);
	void updateRamp(uint16_t rx_value, bool debug);
	//
	void rampAVR(uint16_t rx_value, uint32_t cur_ms);
    //

private:
	uint16_t speedRamp(uint16_t in_val);

};

#endif

//
// Utilisation :
/*
// Version 1, allocation dynamique (ptr)
escDriver *esc_1;
esc_1 = new escDriver(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, RZ7886_PIN1, RZ7886_PIN2, RZ7886_FREQUENCY);
esc_1->set_pwm(motor_chA, motor_chB);

// Version 2, allocation statique
escDriver esc_1(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, RZ7886_PIN1, RZ7886_PIN2, RZ7886_FREQUENCY);
esc_1.set_pwm(motor_chA, motor_chB);
*/