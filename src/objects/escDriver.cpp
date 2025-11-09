//
// EscDriver.cpp : EscDriver class, Driver for RC ESC
//
// Author : PHL @2025

#include <Arduino.h>		// for map, min
//#include "stdint.h"
//#include <driver/mcpwm.h> 	// for servo PWM output
#include <WString.h>

// ESC 
#include "escDriver.h"

// Class definition (methodes)

/*
// ESC Mode (AVR)
#define MODE_STOP 0
#define MODE_FWD 1
#define MODE_REV 2
#define MODE_EMG 99
// ESC ramps (AVR)
#define RAMP_ACC 10    // Accel ramp : acceleration neutre -> AV ou neutre -> AR
#define RAMP_DEC 10    // Decel ramp : decelleration AV ou AR vers neutre

/// @brief EscDriver class, Driver for RC motor ESC
class EscDriver
{
private:
	// data
	int _number = 1;
	mcpwmunit__t unit_;
	mcpwmtimer__t timer_;
	mcpwm_io_signals_t _chanelA;
	mcpwm_io_signals_t _chanelB;
	uint8_t drag_brake_ = 10;

	// For ESC ramp up/down
	uint16_t escRampTimeMs_ = 20;	// Time base for ESP ramp process (ms) (Ex for BOAT)
	uint16_t escRampAcc_ = 2000;	// Acceleration time ramp (ms), Example = 1000 = 1s
	uint16_t escRampDec_ = 2000;	// Decceleration time ramp (ms), Example = 1000 = 1s
	static uint32_t lastEscMs_;
		
	/-*
	int _driveState;				// Driver status machine
	uint8_t escRampTimeMs_ = 20;	// Time base for ESP management (ms) (BOATmode_)
	uint8_t escRampAcc_ = 8;		// Acceleration rampe (µs), ~ 450 / escRampAcc * escRampTimeMs, Example = 450/9*20 -> 1s
	uint8_t escRampDec_ = 8;		// Decceleration rampe (µs), ~ 450 / escRampDec * escRampTimeMs, Example = 450/9*20 -> 1s
	uint32_t _escMillis;
	uint32_t _lastDebugTime;
	*-/

	// Type AVR
	// Signal configuration
	uint16_t neutre_ = 1500;		// Neutre position [1500]
	uint16_t neutre_min_ = 1480;	// Neutre mini [<1500]
	uint16_t neutre_max_ = 1520;	// Neutre maxi [>1500]
	uint16_t val_max_ = 2000;     	// 1500 + 420 //450
	uint16_t val_min_ = 1000;     	// 1500 - 420 //450
	uint8_t mode_ = MODE_EMG;		// Mode : EMG / STOP / FWD / REV
	// Status
	bool neutre_ok_ = false;		// Neutre passed
	uint16_t last_value_;			// Last rx value
	uint16_t signal_pwm_ = 0;     	// Signal for PWM command [1000..2000]


	// Debug
	int motorPwm_;
	//char _msg[50];
	
	// Methode
	//long map(long x, long in_min, long in_max, long out_min, long out_max);

public:
	// Methode
	EscDriver(int number, mcpwmunit__t unit, mcpwmtimer__t timer, mcpwm_io_signals_t chanelA, mcpwm_io_signals_t chanelB, int pinA, int pinB, uint32_t frequency);
	~EscDriver();
	//
	void set_pwm(uint8_t pwmA, uint8_t pwmB);
	void update(uint16_t rx_value, bool debug);
	void updateRamp(uint16_t rx_value, bool debug);
	//
	void rampAVR(uint16_t rx_value, uint32_t cur_ms);
	uint16_t Speed_Ramp(uint16_t in_val);
    //
    int get_pwm();
};
*/


 /// @brief Construct a new EscDriver object, with parameters
 /// @param number ESC number
 /// @param unit MCPWM unit, MCPWMunit__0 or MCPWMunit__1
 /// @param timer MCPWM timer, MCPWMunit__0 or MCPWMunit__1
 /// @param chanelA MCPWM operator, for chanel A (MCPWM0A or MCPWM0B)
 /// @param chanelB MCPWM operator, for chanel B (MCPWM0A or MCPWM0B)
 /// @param pinA PWM pin number, chanel A
 /// @param pinB PWM pin number, chanel B
 /// @param frequency PWM frequency (Hz), 500 Hz is recommended.
EscDriver::EscDriver(int number, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_io_signals_t chanelA, mcpwm_io_signals_t chanelB, int pinA, int pinB, uint32_t frequency)
{
	// Datas
	number_ = number;
	unit_ = unit;
	timer_ = timer;
	chanelA_ = chanelA;
	chanelB_ = chanelB;
	drag_brake_ = 10;
	// 1. set our ESC output pin
	// mcpwm_gpio_init(MCPWMunit__1, MCPWM0A, RZ7886_PIN1); // Set RZ7886 pin 1 as PWM0A
	// mcpwm_gpio_init(MCPWMunit__1, MCPWM0B, RZ7886_PIN2); // Set RZ7886 pin 2 as PWM0B
	mcpwm_gpio_init(unit_, chanelA_, pinA); // Set RZ7886 pin 1 as PWM0A
	mcpwm_gpio_init(unit_, chanelB_, pinB); // Set RZ7886 pin 2 as PWM0B

	// 2. configure MCPWM parameters
	mcpwm_config_t pwm_config;
	pwm_config.frequency = frequency; // frequency
	pwm_config.cmpr_a = 0;			  // duty cycle of PWMxa = 0
	pwm_config.cmpr_b = 0;			  // duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // 0 = not inverted, 1 = inverted

	// 3. configure channels with settings above
	// mcpwm_init(MCPWMunit__1, MCPWMtimer__0, &pwm_config); // Configure PWM0A & PWM0B
	mcpwm_init(unit_, timer_, &pwm_config); // Configure PWM0A & PWM0B
}


/// @brief Destroy the EscDriver object.
EscDriver::~EscDriver()
{
	setPwm(0, 0);
}

/// @brief Set PWM for both chanels
/// @param pwmA PWM value, chanel A, Forward [0..255]
/// @param pwmB PWM value, chanel B, Reverse [0..255]
void EscDriver::setPwm(uint8_t pwmA, uint8_t pwmB)
{
	if (pwmA > 0 && pwmB == 0)	// Forward
	{
		mcpwm_set_signal_high(unit_, timer_, MCPWM_OPR_A); 		// Pin A high
		mcpwm_set_duty(unit_, timer_, MCPWM_OPR_B, pwmA);
		mcpwm_set_duty_type(unit_, timer_, MCPWM_OPR_B, MCPWM_DUTY_MODE_1); // MCPWM_DUTYmode__1 = inverse PWM mode, high, if 0% PWM
	}
	if (pwmB > 0 && pwmA == 0)	// Reverse
	{
		mcpwm_set_signal_high(unit_, timer_, MCPWM_OPR_B); 		// Pin B high
		mcpwm_set_duty(unit_, timer_, MCPWM_OPR_A, pwmB);
		mcpwm_set_duty_type(unit_, timer_, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); // MCPWM_DUTYmode__1 = inverse PWM mode, high, if 0% PWM
	}
	if (pwmA == pwmB)
	{
		// Both pins pwm @ the same time = variable drag brake
		mcpwm_set_duty(unit_, timer_, MCPWM_OPR_A, pwmA); 		// RZ7886_DRAGBRAKE_DUTY);
		mcpwm_set_duty(unit_, timer_, MCPWM_OPR_B, pwmB); 		// RZ7886_DRAGBRAKE_DUTY);
		mcpwm_set_duty_type(unit_, timer_, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
		mcpwm_set_duty_type(unit_, timer_, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
	}
}


/// @brief Update ESC output
/// @param rx_value Rx value (500..2000)
/// @param debug Enable printf for debug
void EscDriver::update(uint16_t rx_value, bool debug)
{
	uint16_t escSignal;
	uint8_t motorDriverDuty;
	
	//escSignal = map(rx_value, escPulseMin, escPulseMax, 1000, 2000);
	//escSignal = map(rx_value, 1000, 2000, 1000, 2000);
	escSignal = rx_value;
	//
	if (escSignal > neutre_max_)	//1520)
	{ 	// Forward
		//motorDriverDuty = min(map(escSignal, 1500, 2000, 0, 100), 100L);
		motorDriverDuty = min(map(escSignal, neutre_max_, 2000, out_min_, out_max_), 100L);
		setPwm(motorDriverDuty, 0);
		motorPwm_ = motorDriverDuty;
		//if (debug) 
			//snprintf(_msg, 50, "- ESC %i : FW, rx %u, pwm %u\n", _number, rx_value, motorDriverDuty);
			// Serial.printf("- ESC 1 : FW, rx %u, pwm %u\n", rx_value, motorDriverDuty);
	}
	else if (escSignal < neutre_min_)	//1480)
	{ 	// Reverse
		//motorDriverDuty = min(map(escSignal, 1500, 1000, 0, 100), 100L);
		motorDriverDuty = min(map(escSignal, neutre_min_, 1000, out_min_, out_max_), 100L);
		setPwm(0, motorDriverDuty);
		motorPwm_ = (int8_t)(0-motorDriverDuty);
		//if (debug) 
			// Serial.printf("- ESC 1 : RW, rx %u, pwm %u\n", rx_value, motorDriverDuty);
	}
	else
	{	// Neutral
		motorDriverDuty = 0; 	// Just for cosmetic reasons in ESC_DEBUG
		motorPwm_ = 0;
		// Both pins pwm @ the same time = variable drag brake
		setPwm(drag_brake_, drag_brake_);
		//if (debug) 
			//Serial.printf("- ESC 1 : No, rx %u, pwm %u\n", rx_value, motorDriverDuty);
	}
}


/// @brief Update ESC output, whith rampe up/down
/// @param rx_value 
/// @param debug 
void EscDriver::updateRamp(uint16_t rx_value, bool debug)
{
	uint16_t escSignal;
	uint8_t motorDriverDuty;
	
	//escSignal = map(rx_value, escPulseMin, escPulseMax, 1000, 2000);
	//escSignal = map(rx_value, 1000, 2000, 1000, 2000);
	escSignal = rx_value;
	//
	if (escSignal > 1520)
	{ 	// Forward
		motorDriverDuty = min(map(escSignal, 1500, 2000, 0, 100), 100L);
		setPwm(motorDriverDuty, 0);
		motorPwm_ = motorDriverDuty;
		//if (debug) 
			//snprintf(_msg, 50, "- ESC %i : FW, rx %u, pwm %u\n", _number, rx_value, motorDriverDuty);
			// Serial.printf("- ESC 1 : FW, rx %u, pwm %u\n", rx_value, motorDriverDuty);
	}
	else if (escSignal < 1480)
	{ 	// Reverse
		motorDriverDuty = min(map(escSignal, 1500, 1000, 0, 100), 100L);
		setPwm(0, motorDriverDuty);
		motorPwm_ = (int8_t)(0-motorDriverDuty);
		//if (debug) 
			// Serial.printf("- ESC 1 : RW, rx %u, pwm %u\n", rx_value, motorDriverDuty);
	}
	else
	{	// Neutral
		motorDriverDuty = 0; 	// Just for cosmetic reasons in ESC_DEBUG
		motorPwm_ = 0;
		// Both pins pwm @ the same time = variable drag brake
		setPwm(drag_brake_, drag_brake_);
		//if (debug) 
			//Serial.printf("- ESC 1 : No, rx %u, pwm %u\n", rx_value, motorDriverDuty);
	}
}


/// @brief Return PWM value
/// @return Value [-100..+100]
int32_t EscDriver::getPwm ()
{
	return motorPwm_;
}


/// @brief ESC with ramp (idem AVR)
/// @param rx_value Rx value command [1000..2000]
/// @param cur_ms Current us
void EscDriver::rampAVR(uint16_t rx_value, uint32_t cur_ms)
{
	// Stop command
	if ((rx_value > neutre_min_) && (rx_value < neutre_max_))
		neutre_ok_ = true;
	// Stop passed
	if (neutre_ok_)
	{
		// Every ms
		if (cur_ms - lastEscMs_ > escRampTimeMs_)
		{
			signal_pwm_ = speedRamp(rx_value);
			lastEscMs_ = cur_ms;
		}
	}
	else
	{
		last_value_ = neutre_;
		signal_pwm_ = neutre_;
	}
	// PWM
	update(signal_pwm_, false);
}


/// @brief ESC Ramp process
/// @param in_val Rx value [1000..2000]
/// @return Signal processed [1000..2000]
uint16_t EscDriver::speedRamp(uint16_t in_val) 
{
	unsigned int val = in_val;
	//uint16_t rampAcc = 1000 / (escRampAcc_ / escRampTimeMs_);		//escRampAcc_, escRampDec_, 
	//uint16_t rampDec = 1000 / (escRampDec_ / escRampTimeMs_);
	uint16_t rampAcc = (val_max_ - val_min_) / (escRampAcc_ / escRampTimeMs_);		//escRampAcc_, escRampDec_, 
	uint16_t rampDec = (val_max_ - val_min_) / (escRampDec_ / escRampTimeMs_);

	// Get current mode
	if ((last_value_ > neutre_max_) && (mode_ != MODE_EMG)) {
		mode_ = MODE_FWD;
	} else if ((last_value_ < neutre_min_) && (mode_ != MODE_EMG)) {
		mode_ = MODE_REV;
	} else if ((last_value_ >= neutre_min_) && (last_value_ <= neutre_max_)) {
		mode_ = MODE_STOP;
	} else {
		mode_ = MODE_EMG;
	}

	// Stop
	if (mode_ == MODE_STOP) {
		// FWD ramp
		if (in_val > last_value_ + rampAcc)
			val = last_value_ + rampAcc;
		// REV ramp
		if (in_val < last_value_ - rampAcc) 
			val = last_value_ - rampAcc;
	}

	// Forward
	if (mode_ == MODE_FWD) {
		// Acc ramp : neutre -> FWD
		if (in_val > last_value_ + rampAcc)
			val = last_value_ + rampAcc;
		// Dec ramp : FWD -> neutre
		if (in_val < last_value_ - rampDec)
			val = last_value_ - rampDec;
	}

	// Reverse
	if (mode_ == MODE_REV) {
		// Acc ramp : neutre -> REV
		if (in_val < last_value_ - rampAcc)
			val = last_value_ - rampAcc;
		// Dec ramp : REV -> neutre
		if (in_val > last_value_ + rampDec)
			val = last_value_ + rampDec;
	}
	//
	last_value_ = val;
	return val;
}




/*
void EscDriver::escBoat(uint16_t rx_value)
{
	//escRampTime = escRampTimeMs_;

	// Additional brake detection signal, applied immediately. Used to prevent sound issues, if braking very quickly
	//brakeDetect = ((pulse() == 1 && escPulse() == -1) || (pulse() == -1 && escPulse() == 1));

	if (millis() - lastEscMs_ > escRampTimeMs_)
	{ // About very 20 - 75ms
		lastEscMs_ = millis();

		// Drive state machine
		switch (_driveState)
		{

		case 0: // Standing still ---------------------------------------------------------------------
			//escIsBraking = false;
			//escInReverse = false;
			//escIsDriving = false;
			//escPulseWidth = pulseZero[3]; // ESC to neutral position

			if (pulse() == 1 && engineRunning && !neutralGear)
				_driveState = 1; // Driving forward
			if (pulse() == -1 && engineRunning && !neutralGear)
				_driveState = 3; // Driving backwards
			break;

		case 1: // Driving forward ---------------------------------------------------------------------
			//escIsBraking = false;
			//escInReverse = false;
			//escIsDriving = true;
			// Acceleration
			if (pulseWidth[3] > escPulseWidth && currentSpeed < speedLimit && !batteryProtection)
			{
				if (escPulseWidth >= escPulseMaxNeutral)
					escPulseWidth += escRampAcc;
				else
					escPulseWidth = escPulseMaxNeutral;
			}
			// Decelleration
			if ((pulseWidth[3] < escPulseWidth || batteryProtection) && escPulseWidth > pulseZero[3])
				escPulseWidth -= escRampDec; // Decelleration

			// Limit
			if (escPulseWidth > escPulseMax)
				escPulseWidth = escPulseMax;
			//
			if (escPulseWidth < escPulseMaxNeutral)
				escPulseWidth = pulseZero[3];

			if (pulse() != 1 && escPulse() == 0)
				_driveState = 0; // standing still
			break;

		/-*
		case 2: // Braking forward ---------------------------------------------------------------------
			escIsBraking = true;
			escInReverse = false;
			escIsDriving = false;
			if (escPulseWidth > pulseZero[3])
				escPulseWidth -= brakeRampRate; // brake with variable deceleration
			if (escPulseWidth < pulseZero[3] + brakeMargin && pulse() == -1)
				escPulseWidth = pulseZero[3] + brakeMargin; // Don't go completely back to neutral, if brake applied
			if (escPulseWidth < pulseZero[3] && pulse() == 0)
				escPulseWidth = pulseZero[3]; // Overflow prevention!

			if (pulse() == 0 && escPulse() == 1 && !neutralGear)
			{
				driveState = 1; // Driving forward
				airBrakeTrigger = true;
			}
			if (pulse() == 0 && escPulse() == 0)
			{
				driveState = 0; // standing still
				airBrakeTrigger = true;
			}
			break;
		*-/

		case 3: // Driving backwards ---------------------------------------------------------------------
			//escIsBraking = false;
			//escInReverse = true;
			//escIsDriving = true;
			// Acceleration
			if (pulseWidth[3] < escPulseWidth && currentSpeed < speedLimit && !batteryProtection)
			{
				if (escPulseWidth <= escPulseMinNeutral)
					escPulseWidth -= escRampAcc;
				else
					escPulseWidth = escPulseMinNeutral;
			}
			// Decelleration
			if ((pulseWidth[3] > escPulseWidth || batteryProtection) && escPulseWidth < pulseZero[3])
				escPulseWidth += escRampDec;

			// Limit
			if (escPulseWidth < escPulseMin)
				escPulseWidth = escPulseMin;
			//
			if (escPulseWidth > escPulseMinNeutral)
				escPulseWidth = pulseZero[3];

			if (pulse() != -1 && escPulse() == 0)
				_driveState = 0; // standing still
			break;
		
		/-*
		case 4: // Braking backwards ---------------------------------------------------------------------
			escIsBraking = true;
			escInReverse = true;
			escIsDriving = false;
			if (escPulseWidth < pulseZero[3])
				escPulseWidth += brakeRampRate; // brake with variable deceleration
			if (escPulseWidth > pulseZero[3] - brakeMargin && pulse() == 1)
				escPulseWidth = pulseZero[3] - brakeMargin; // Don't go completely back to neutral, if brake applied
			if (escPulseWidth > pulseZero[3] && pulse() == 0)
				escPulseWidth = pulseZero[3]; // Overflow prevention!

			if (pulse() == 0 && escPulse() == -1 && !neutralGear)
			{
				driveState = 3; // Driving backwards
				airBrakeTrigger = true;
			}
			if (pulse() == 0 && escPulse() == 0)
			{
				driveState = 0; // standing still
				airBrakeTrigger = true;
			}
			break;
			*-/
		} // End of state machine
	}
}
*/

