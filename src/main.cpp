//
// ESP32_RC_Controler : RC Controler with engine sound
//
// PHL @2025

// SHIFT + ALT + F : Format document
//
// Ctrl K + Ctrl j: UnFold
// Ctrl K + Ctrl 0: fold all levels (namespace, class, method, and block)
// Ctrl K + Ctrl 1: namespace / @Component(For Angular)
// Ctrl K + Ctrl 2: class / methods
// Ctrl K + Ctrl 3: methods / blocks
// Ctrl K + Ctrl 4: blocks / inner blocks
// Ctrl K + Ctrl [ or ]: current cursor block

/* TYPES
1 byte  : bool, byte, char, unsigned char, int8_t, uint8_t
2 bytes : short, int16_t, uint16_t
4 bytes : int, long, unsigned int, unsigned long, int32_t, uint32_t
8 bytes : long long, unsigned long long, int34_t, uint64_t
4 bytes : float,
8 bytes : double
*/

#include "main.h"

// Local functions
void Task1code(void *pvParameters);
void setupMcpwm();
void mcpwmOutput();
void outputUpdate();
void SerialDebug(uint32_t cur_ms, uint32_t _last_proccess);
// void TriggerSet();
void SoundTriggerSet();

// =======================================================================================================
// PPM Functions
// =======================================================================================================
#pragma region PPM_FUNCTIONS
#ifdef PPM_COMMUNICATION
// =======================================================================================================
// PPM Signal INTERRUPT
// =======================================================================================================
// Interrupt call by PPM digital pin rising edge
void IRAM_ATTR IT_Signal_Ppm()
{
	it_count++;
	rx_data->IT_Ppm(micros());
}

#endif
#pragma endregion PPM_FUNCTIONS

// =======================================================================================================
// PWM Functions
// =======================================================================================================
#pragma region PWM_FUNCTIONS
#ifdef PWM_COMMUNICATION
// Declare static data
volatile uint8_t* RxPwm::pwmPin_ = PWM_PINS;
volatile unsigned int* RxPwm::pwmBuffer_ = new unsigned int [MAX_PWM_CHANNELS];
volatile bool RxPwm::new_isr_data_ = false;
#endif 	// PWM_COMMUNICATION
#pragma endregion PWM_FUNCTIONS

// =======================================================================================================
// Setup
// =======================================================================================================
/// @brief Setup program
void setup()
{
	// Watchdog timers need to be disabled, if task 1 is running without delay(1)
	disableCore0WDT();
	// disableCore1WDT(); // Core 1 WDT can stay enabled TODO

	// Setup RTC (Real Time Clock) watchdog
	rtc_wdt_protect_off(); // Disable RTC WDT write protection
	rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
	rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
	rtc_wdt_set_time(RTC_WDT_STAGE0, 30000); // set 10s timeout
	rtc_wdt_enable();						 // Start the RTC WDT timer
											 // rtc_wdt_disable();            // Disable the RTC WDT timer
	rtc_wdt_protect_on();					 // Enable RTC WDT write protection

	// Set pin modes
	pinMode(COMMAND_RX, INPUT_PULLDOWN);

	// Serial setup
	Serial.begin(115200, SERIAL_8N1, DEBUG_RX, DEBUG_TX); // USB serial (for DEBUG) Mode, Rx pin (99 = not used), Tx pin
	delay(1000);										  // Give serial port/connection some time to get ready
	Serial.printf("\n");
	Serial.printf("**************************************************************************************************\n");
	Serial.printf("* Setup\n");
	Serial.printf("**************************************************************************************************\n");
	Serial.printf("* XTAL Frequency: %i MHz, CPU Clock: %i MHz, APB Bus Clock: %i Hz\n", getXtalFrequencyMhz(), getCpuFrequencyMhz(), getApbFrequency());
	Serial.printf("* Internal RAM size: %i Byte, Free: %i Byte\n", ESP.getHeapSize(), ESP.getFreeHeap());
	for (uint8_t coreNum = 0; coreNum < 2; coreNum++)
	{
		uint8_t resetReason = rtc_get_reset_reason(coreNum);
		if (resetReason <= (sizeof(RESET_REASONS) / sizeof(RESET_REASONS[0])))
		{
			Serial.printf("* Core %i reset reason: %i: %s\n", coreNum, rtc_get_reset_reason(coreNum), RESET_REASONS[resetReason - 1]);
		}
	}
	lastloop_ms = millis();
	Serial.printf("* File : %s\n", __FILE__);
	Serial.printf("* - Compiled : %s at %s \n", __DATE__, __TIME__);
	Serial.printf("* Configuration : %s\n", CONFIGURATION);
	//
	Serial.printf("* Setup MCPWM \n");
	setupMcpwm();
#ifdef PWM_COMMUNICATION
	Serial.printf("* - PWM1 to PWM4, Not avalaible if RX_PWM Mode\n");
#else
	Serial.printf("* - PWM 1 : Ch[%i]\n", OutputChannel[1]);
	Serial.printf("* - PWM 2 : Ch[%i]\n", OutputChannel[2]);
	Serial.printf("* - PWM 3 : Ch[%i]\n", OutputChannel[3]);
	Serial.printf("* - PWM 4 : Ch[%i]\n", OutputChannel[4]);
#endif
	Serial.printf("* - PWM 5 : Ch[%i]\n", OutputChannel[5]);
	Serial.printf("* - PWM 6 : Ch[%i]\n", OutputChannel[6]);

	// ESC contiguration
#ifdef ESC1_DRIVER // ESC 1
	esc_1 = new EscDriver(1, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, ESC1_PIN1, ESC1_PIN2, ESC1_FREQUENCY);
	Serial.printf("* Setup ESC 1 : Ch[%i], Unit %i, timer %i, Pin %i - %i, %iHz at %x\n", ESC1_CH, MCPWM_UNIT_1, MCPWM_TIMER_0, ESC1_PIN1, ESC1_PIN2, ESC1_FREQUENCY, esc_1);
	esc_1->setPwm(ESC1_DRAGBRAKE_DUTY, ESC1_DRAGBRAKE_DUTY);
#endif
#ifdef ESC2_DRIVER // ESC 2
	esc_2 = new EscDriver(2, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, ESC2_PIN1, ESC2_PIN2, ESC2_FREQUENCY);
	Serial.printf("* Setup ESC 2 : Ch[%i], Unit %i, timer %i, Pin %i - %i, %iHz at %x\n", ESC2_CH, MCPWM_UNIT_1, MCPWM_TIMER_1, ESC2_PIN1, ESC2_PIN2, ESC2_FREQUENCY, esc_2);
	esc_2->setPwm(ESC2_DRAGBRAKE_DUTY, ESC2_DRAGBRAKE_DUTY);
#endif
#ifdef ESC3_DRIVER // ESC 3
	esc_3 = new EscDriver(3, MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2A, MCPWM2B, ESC3_PIN1, ESC3_PIN2, ESC3_FREQUENCY);
	Serial.printf("* Setup ESC 3 : Ch[%i], Unit %i, timer %i, Pin %i - %i, %iHz at %x\n", ESC3_CH, MCPWM_UNIT_1, MCPWM_TIMER_2, ESC3_PIN1, ESC3_PIN2, ESC3_FREQUENCY, esc_3);
	esc_3->setPwm(ESC3_DRAGBRAKE_DUTY, ESC3_DRAGBRAKE_DUTY);
#endif
	// Once write with the "normal" way, the write registers directly according to: https://forum.arduino.cc/t/esp32-dacwrite-ersetzen/653954/5
	// all further writes are done directly in the register since
	// it's much faster
	dacWrite(DAC1, 128);
	dacWrite(DAC2, 128);

	// Task 1 setup (running on core 0)
	/*
	TaskHandle_t Task1;
	// Create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
	xTaskCreatePinnedToCore(
		Task1code, 		// Task function
		"Task1",   		// name of task
		8192,      		// Stack size of task (8192)
		//NULL,			// parameter of the task
		(void*)&mtVariables,	// parameter : struct for data exchange
		1,         		// priority of the task (1 = low, 3 = medium, 5 = highest)
		&Task1,    		// Task handle to keep track of created task
		0);        		// pin task to core 0
	*/

	// Refresh sample intervals (important, because MAX_RPM_PERCENTAGE was probably changed above)
	maxSampleInterval = 4000000 / sampleRate;
	minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;
	masterVolume = 50;

	// Interrupt timer for variable sample rate playback
	variableTimer = timerBegin(0, 20, true);						   // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
	timerAttachInterrupt(variableTimer, &variablePlaybackTimer, true); // edge (not level) triggered
	timerAlarmWrite(variableTimer, variableTimerTicks, true);		   // autoreload true
	timerAlarmEnable(variableTimer);								   // enable

	// Interrupt timer for fixed sample rate playback
	fixedTimer = timerBegin(1, 20, true);						 // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
	timerAttachInterrupt(fixedTimer, &fixedPlaybackTimer, true); // edge (not level) triggered
	timerAlarmWrite(fixedTimer, fixedTimerTicks, true);			 // autoreload true
	timerAlarmEnable(fixedTimer);								 // enable

	Serial.printf("* Setup communication :\n");
#if defined CRSF_COMMUNICATION
	// CRSF Communication
	rx_data = new RxCrsf(RX_CHANNELS, channelFailSafe, channelIntregrator, channelReversed);
	Serial.printf("* - CRSF mode, %i channels, error code : %i\n", rx_data->getNbChannels(), rx_data->error);

#elif defined SBUS_COMMUNICATION
	// SBUS Communication
	rx_data = new RxSbus(RX_CHANNELS, channelFailSafe, channelIntregrator, channelReversed);
	Serial.printf("* - SBUS mode, %i channels, error code : %i\n", rx_data->getNbChannels(), rx_data->error);

//#elif defined SBUS_COMMUNICATION_LOCAL
	// SBUS Communication - Local (Old test)
/*
	rx_data = new RxBase(RX_CHANNELS, channelFailSafe, channelIntregrator, channelReversed);
	if (MAX_RPM_PERCENTAGE > maxSbusRpmPercentage)
		MAX_RPM_PERCENTAGE = maxSbusRpmPercentage;				// Limit RPM range
	sBus.begin(COMMAND_RX, COMMAND_TX, sbusInverted, sbusBaud); // begin SBUS communication with compatible receivers

	sbusInit = false;
	Serial.printf("Initializing SBUS LOCAL (sbusInverted = %s, needs to be true for most standard radios) ...\n", sbusInverted ? "true" : "false");
	Serial.printf("(Make sure radio and receiver are connected, turned on, bound and configured for SBUS output.)\n");
	while (!sbusInit)
	{
		readSbusCommands(micros()); // SBUS communication (pin 36)
		// indicatorL.flash(70, 75, 500, 3); // Show 3 fast flashes on indicators!
		// indicatorR.flash(70, 75, 500, 3);
		rtc_wdt_feed(); // Feed watchdog timer
	}
	Serial.printf("... SBUS initialization succesful!\n");
*/

#elif defined PPM_COMMUNICATION
	// PPM Mode
	// if (MAX_RPM_PERCENTAGE > maxPpmRpmPercentage)
	//   MAX_RPM_PERCENTAGE = maxPpmRpmPercentage;						 	// Limit RPM range
//#ifdef PPM_LOCAL	
	Serial.printf("* Configuration\n");
	for (int i = 0; i <= MAX_CHANNELS; i++)
	{
		Serial.printf("Ch[%i]: rev %i, fs %i\n", i, channelReversed[i], channelFailSafe[i]);
	}
	//Serial.printf("* Setup PPM Communication\n");
	rx_data = new RxPpm(RX_CHANNELS, 1, channelFailSafe, channelIntregrator, channelReversed);
	Serial.printf("* - Setup PPM Communication, attach interrupt \n");
	attachInterrupt(digitalPinToInterrupt(COMMAND_RX), IT_Signal_Ppm, RISING); // begin PPM communication with compatible receivers
																			   // attachInterrupt(GPIO_NUM_36, PpmIT, RISING); 	// begin PPM communication with compatible receivers
//#else
	// PPM Inside RxPpm class
	//rx_data = new RxPpm(RX_CHANNELS, 1, channelFailSafe, channelIntregrator, channelReversed);
	//rx_data->attachInterrupts();

//#endif

#elif defined PWM_COMMUNICATION
	// PWM Mode
	rx_data = new RxPwm(RX_CHANNELS, PWM_PINS, PWM_CHANNELS, channelFailSafe, channelIntregrator, channelReversed);
	rx_data->attachInterrupts();
	Serial.printf("* - PWM mode (GPIO ISR), %i channels, error code : %i\n", rx_data->getNbChannels(), rx_data->error);

#else
	// No Mode
	rx_data = new RxBase(RX_CHANNELS, channelFailSafe, channelIntregrator, channelReversed);
	Serial.printf("* - Error : No communication mode selected\n");

#endif // Communication mode

	// RX Channel triggers
	// trigger default values are (1800, 500, 1200, 500, 50)
	rx_data->channel[STEERING].trigger.setLevel(1600, 250, 1400, 250, 50);
	rx_data->channel[2].trigger.setLevel(1800, 1000, 1200, 1000, 50);

	// Leds (Use one timer by output, Timer 0,1 : reserved, use 2..15)
	leds = new Outputs();
	// for (int i = 0; i < OUTPUTS_MAX; i++)
	//{
	//	leds[i] = statusLED(false);
	//	leds[i].begin(OUTPUT_PINS[i], i+2, 20000);	// Timers 2 to 14
	// }
	// indicatorL.begin(2, 2, 20000);			// Indicator_L, Timer 2, 20kHz
	// indicatorR.begin(4, 3, 20000);			// Indicator_R, Timer 2, 20kHz
	// shakerMotor.begin(23, 13, 20000); 	// Shaker motor, Timer 13, 20kHz

	// TCY
	minTcy = 4294967295;
	maxTcy = 0;
	lastLoop_us = micros();

	// End
	//Serial.printf("*\n");
	Serial.printf("* Mémoire libre après allocation: %d bytes\n", esp_get_free_heap_size());
	Serial.printf("* Setup done\n\n");
	Serial.printf("**************************************************************************************************\n");
	Serial.printf("* Loop\n");
	Serial.printf("**************************************************************************************************\n");
}

// =======================================================================================================
// Loop : Main loop
// =======================================================================================================
/// @brief Loop, background cycle
void loop()
{
	bool new_data;
	// Curent ms, us
	uint32_t curTcy;
	//
	uint32_t cur_ms = millis();
	uint32_t cur_us = micros();
	bgdCount++;

	// Running core
	if (coreLoop == -1)
		coreLoop = xPortGetCoreID();
/*
	// RX signal update
#if defined SBUS_COMMUNICATION 			// Test : TX16S + RP3-H (SBus mode)
	// SBUS Communication
	rx_data->updateChannels(cur_us);

	/*
	#elif defined IBUS_COMMUNICATION		// NOT TESTED
		readIbusCommands(); // IBUS communication (pin 36)
		//mcpwmOutput();		// PWM servo signal output

	#elif defined SUMD_COMMUNICATION		// NOT TESTED
		readSumdCommands(); // SUMD communication (pin 36)
		//mcpwmOutput();		// PWM servo signal output
	*-/

#elif defined CRSF_COMMUNICATION		// Tested TX16S + RP3-H / ER6 (CRSF Mode) (2025-04)
	// CRSF communication (RX1 modified to pin 36)
	rx_data->updateChannels(cur_us);

#elif defined PPM_COMMUNICATION 		// Tested MC2020 (2025-04)
	// PPM communication (pin 36)
	rx_data->updateChannels(cur_us);

#elif defined PWM_COMMUNICATION 		// Tested TX16S + ER6 (2025-11)
	rx_data->updateChannels(cur_us);

#endif // RX signal update
*/

	// RX channels update
	rx_data->updateChannels(cur_us);
	// Update
	uint32_t _last_proccess = rx_data->GetLastProccess();
	uint32_t _delta_us = rx_data->timeDiff(cur_us, _last_proccess);
	// New data or 100ms
	new_data = rx_data->Get_NewData();		// Get and reset if true
	if (new_data || (_delta_us > 500000UL)) // New data or Timeout 500ms
	{
		/*
		if (_delta_us > 500000UL && rx_data->error == 0) // Timeout 500ms
		{
			rx_data->failSafe = true;
			rx_data->error = -4;
			//rx_data->setRawChannel(FAILSAFE_CHANNEL, 0);	// For PPM
		}
		*/
		// Data process
		rx_data->processRawChannels(cur_us);
		cpProccess++;
		mcpwmOutput(); // PWM servo signal output
		// Esc 1
		if (esc_1 != NULL)
		{
			// esc_1->rampAVR(rx_data->channel[ESC1_CH].rx, cur_ms);	// Rampe managed by ESC (old)
			esc_1->update(rx_data->channel[ESC1_CH].rxd, false); // Rampe managed by channel[].rxd
			currentPwm = abs(esc_1->getPwm());					 // [0..100] %
			currentRpm = currentPwm * 5;						 // [0..500]
		}
		else
		{
			currentPwm = abs(map(rx_data->channel[THROTTLE].rxd, 1000, 2000, -100, 100));
			currentRpm = currentPwm * 5; // [0..500]
		}
		// Esc 2
		if (esc_2 != NULL)
			// esc_2->rampAVR(rx_data->channel[ESC2_CH].rx, cur_ms);	// Rampe managed by ESC
			esc_2->update(rx_data->channel[ESC2_CH].rxd, false); // Rampe managed by channel[].rxd
		// Esc 3
		if (esc_3 != NULL)
			// esc_3->rampAVR(rx_data->channel[ESC3_CH].rx, cur_ms);	// Rampe managed by ESC
			esc_3->update(rx_data->channel[ESC3_CH].rxd, false); // Rampe managed by channel[].rxd
		//
		// Triggers
		//
		/*
		trigCh1.update(rx_data->_channel[1].rx, cur_ms);
		trigCh2.update(rx_data->channel[2].rx, cur_ms);
		trigCh3.update(rx_data->_channel[3].rx, cur_ms);
		trigCh4.update(rx_data->channel[4].rx, cur_ms);
		*/
		//
		// IEC Timer
		//
		TON_Pwm_0.TON((currentPwm == 0), cur_ms, 20000); // 20s

		//
		// Engine sound
		//
		if (rx_data->channel[THROTTLE].rxd > 1700 && !engineOn)
			engineOn = true;
		if ((rx_data->failSafe || TON_Pwm_0.ton) && engineOn)
			engineOn = false;

		// Speed (sample rate) output
		engineSampleRate = map(currentPwm, minPwm, maxPwm, maxSampleInterval, minSampleInterval); // Idle
		// Sound trigger
		SoundTriggerSet();
		// Volume adjustment
		VolumeSet();
		// Fader
		// dacOffsetFade(); 	// Move to Task1

		// Engine sound (end)
	}
	// Output update
	modelOutput(rx_data->channel, leds->channel);
	// outputUpdate();			// Output

	// Debug chanels
	if (cur_ms > lastloop_ms + 500)
	{
		lastloop_ms = cur_ms;
		debugLine = 1;
	}
	if (debugLine > 0)
		SerialDebug(lastloop_ms, _last_proccess);

	// Loop TCY
	cur_us = micros();
	curTcy = (uint32_t)cur_us - lastLoop_us;
	lastLoop_us = cur_us;
	if (cur_ms > 2000)
	{
		if (curTcy < minTcy)
			minTcy = curTcy;
		if (curTcy > maxTcy)
			maxTcy = curTcy;
	}

	// Feeding the RTC watchtog timer is essential!
	rtc_wdt_feed();
}

// =======================================================================================================
// Task 1
// =======================================================================================================
/// @brief Task1code : Task1 programm
/// @param pvParameters
void Task1code(void *pvParameters)
{
	mtVariables_t *my_var = (mtVariables_t *)pvParameters;
	for (;;)
	{
		// Running core
		if (my_var->coreT1 < 0)
			my_var->coreT1 = xPortGetCoreID();

		// ESC control & low discharge protection
		// #ifdef BOAT_MODE
		//	esc_boat();
		// #else
		//	esc();
		// #endif

		// DAC offset fader
		dacOffsetFade();

		// Engine
		// if (rx_data->rxd_channel[ESC1_CH] > 1750 && !engineOn)
		//	engineOn = true;
		// currentRpm = 50;	//abs(esc_1->getPwm());

		// measure loop time
		// loopTime = loopDuration(); // for debug only

		// Feeding the RTC watchtog timer is essential!
		rtc_wdt_feed(); // TODO, test only
	}
}

// =======================================================================================================
// Outputs
// =======================================================================================================

/// @brief Output update
void outputUpdate()
{
	// outputModel(rx_data->channel, leds->channel);
	return;

	// int l_curPwm = std::abs(currentPwm);
	// l_curPwm = l_curPwm << 1;
	// int l_curPwm = abs(map(currentPwm, -100, 100, -255, 255));
	//  Output / Led 1
	// Led1.set(rx_data->channel[4].trigger.latchLow);
	// led1.set(OUT1_CMD);
	//  Output / Led 2
	// led2.set(OUT2_CMD);
	//  Output / Led 3
	// Led3.set(OUT3_CMD);
	// led_d2.pwm(l_curPwm);
	// led_d4.pwm(255-l_curPwm);

	/*
	// Indicator R [4], pin 'D4'
	if (rx_data->channel[STEERING].trigger.low)
	{
		leds->channel[4].flash(375, 375, 0, 0, 0, 250, 0);
		//leds->flash(4, 375, 375, 0, 0, 0, 250, 0);
		//indicatorR.flash(375, 375, 0, 0, 0, 250, 0);
	}
	else
		leds->channel[4].off(250, 0);
		//leds->off(250, 0);
		//indicatorR.off(250, 0);

	// Indicator L [5], pin 'D2'
	if (rx_data->channel[STEERING].trigger.high)
		leds->channel[5].flash(375, 375, 0, 0, 0, 250, 0);
		//leds->flash(5, 375, 375, 0, 0, 0, 250, 0);
		//indicatorL.flash(375, 375, 0, 0, 0, 250, 0);
	else
		leds->channel[5].off(250, 0);
		//leds->off(250, 0);
		//indicatorL.off(250, 0);
	*/
}

/// @brief Setup MCPWM configuration (ch1 to ch6, ch7)
void setupMcpwm()
{
	// Configure MCPWM parameters
	mcpwm_config_t pwm_config;
	pwm_config.frequency = SERVO_FREQUENCY; // frequency usually = 50Hz, some servos may run smoother @ 100Hz
	pwm_config.cmpr_a = 0;					// duty cycle of PWMxa = 0
	pwm_config.cmpr_b = 0;					// duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // 0 = not inverted, 1 = inverted

#ifndef PWM_COMMUNICATION
	// Set our servo output pins
#ifndef ESC3_DRIVER
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 13); // Set steering as PWM0A
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 12); // Set shifting as PWM0B
	// Configure channels with settings above
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B
#else
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 15); // Set steering as PWM0A
	// Configure channels with settings above
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B
#endif
#ifndef ESC2_DRIVER
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, 14); // Set xx as PWM1A
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, 27); // Set xx as PWM1B
	// Configure channels with settings above
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); // Configure PWM1A & PWM1B
#endif
#endif // PWM_COMMUNICATION
#ifndef ESC1_DRIVER
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, 32); // Set xx as PWM2A
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, 33); // Set xx as PWM2B
	// Configure channels with settings above
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config); // Configure PWM2A & PWM2B
#endif
}

/// @brief Set MCPWM outputs (ch1 to ch6, ch7)
void mcpwmOutput()
{
	// Set PWM chanels [1000..2000]
#ifndef PWM_COMMUNICATION
#ifndef ESC3_DRIVER
	// CH1 [1000..2000], Pin 13
	// mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, rx_data->_channel[1].rx);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, rx_data->channel[OutputChannel[1]].rx);
	// CH2, Pin 12
	// mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, rx_data->_channel[2].rx);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, rx_data->channel[OutputChannel[2]].rx);
#else
	//  CH1 [1000..2000], Pin 15 (PHL)
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, rx_data->channel[1].rx);
#endif

#ifndef ESC2_DRIVER
	// CH3, Pin 14
	// mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, rx_data->_channel[3].rx);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, rx_data->channel[OutputChannel[3]].rx);
	// CH4, Pin 27
	// mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, rx_data->_channel[4].rx);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, rx_data->channel[OutputChannel[4]].rx);
#endif
#endif
#ifndef ESC1_DRIVER
	// CH5, Pin 33
	// mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, rx_data->_channel[5].rx);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, rx_data->channel[OutputChannel[5]].rx);
	// CH6, Pin 32
	// mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, rx_data->_channel[6].rx);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, rx_data->channel[OutputChannel[6]].rx);
#endif
}

/// @brief Serial debug, send line by line to reduce loop cycle time
/// @param cur_ms Current ms
/// @param _last_proccess Last process time
void SerialDebug(uint32_t cur_ms, uint32_t _last_proccess)
{
#ifndef DEBUG_RX_DATA
	// Printf debug, line by line
	switch (debugLine)
	{
	case 1:
		Serial.printf("* Loop() : Tcy min %lu, max %lu us\n", minTcy, maxTcy);
		debugLine++;
		break;
	case 2:
		Serial.printf("* loop %lu ms, BGD %lu, IT %lu, %lu, rd %lu, error %i\n", cur_ms, bgdCount, it_count, rx_data->GetItCount(), rx_data->GetRdCount(), rx_data->error);
		debugLine++;
		break;
	case 3:
		Serial.printf("* Proccess : last %lu, cpt %lu cycle %u us\n", _last_proccess, cpProccess, rx_data->delta_us_proccess); //_delta_us);
		debugLine++;
		break;
	case 4:
#ifdef ESC1_DRIVER
		// Serial.printf("- ESC 1 : rx %u, pwm %i, engine %i, state %i\n", rx_data->channel[ESC1_CH].rx, esc_1->getPwm(), engineOn, engineState);
		Serial.printf("- ESC 1 : rx %u, pwm %i, engine %i, state %i\n", rx_data->channel[ESC1_CH].rx, esc_1->getPwm(), engineOn, engineState);
#endif
		debugLine++;
		break;
	case 5:
#ifdef ESC2_DRIVER
		Serial.printf("- ESC 2 : rx %u, pwm %i\n", rx_data->channel[ESC2_CH].rx, esc_2->getPwm());
#endif
		debugLine++;
		break;
	case 6:
#ifdef ESC2_DRIVER
		Serial.printf("- ESC 3 : rx %u, pwm %i\n", rx_data->channel[ESC3_CH].rx, esc_3->getPwm());
#endif
		//
		debugLine++;
		break;
	case 7:
		Serial.printf("- Cores : Loop %i, Task %i, FixT %i, VarT %i, %i\n", coreLoop, mtVariables.coreT1, mtVariables.coreFT, mtVariables.coreVT, mtVariables.isrVT_Cnt);
		// Serial.printf("- Cores : Loop %i, Task %i\n", coreLoop, mtVariables.coreT1);
		debugLine++;
		break;
	case 8:
		if (rx_data->failSafe)
#ifdef PPM_COMMUNICATION
			Serial.printf("* FailSafe, Ch[%i]: raw %u, rx %u us\n", FAILSAFE_CHANNEL, rx_data->channel[FAILSAFE_CHANNEL].raw, rx_data->channel[FAILSAFE_CHANNEL].rx);
#else
			Serial.printf("* FailSafe active\n");
#endif
		debugLine = 10;
		break;
	case 10:
		debugCh = 0;
		//Serial.printf("Ch[%i]: raw %4u rx %4u rxd %4u (us)\n", debugCh, rx_data->channel[debugCh].raw, rx_data->channel[debugCh].rx, rx_data->channel[debugCh].rxd);
		debugLine++;
		debugCh++;
		break;
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
	case 17:
	case 18:
	case 19:
	case 20:
	case 21:
	case 22:
	case 23:
	case 24:
	case 25:
	case 26:
		if (debugCh <= rx_data->getNbChannels())
		{
			Serial.printf("Ch[%2i]: raw %4u rx %4u rxd %4u (us)", debugCh, rx_data->channel[debugCh].raw, rx_data->channel[debugCh].rx, rx_data->channel[debugCh].rxd);
			Serial.printf(", Trig : %u, %u, %u, %u, %u, %u", rx_data->channel[debugCh].trigger.low, rx_data->channel[debugCh].trigger.lowD, rx_data->channel[debugCh].trigger.latchLow, rx_data->channel[debugCh].trigger.high, rx_data->channel[debugCh].trigger.highD, rx_data->channel[debugCh].trigger.latchHigh);
			Serial.printf("\n");
		}
		debugLine++;
		debugCh++;
		break;
	case 27:
		debugLine = 40;	// 30;
		break;
	case 30:
		debugCh = 1;
	case 31:
	case 32:
	case 33:
	case 34:
	case 35:
	case 36:
	case 37:
	case 38:
		if (debugCh <= rx_data->getNbChannels())
			Serial.printf("Trig Ch[%i]: %u, %u, %u, %u, %u, %u\n", debugCh, rx_data->channel[debugCh].trigger.low, rx_data->channel[debugCh].trigger.lowD, rx_data->channel[debugCh].trigger.latchLow, rx_data->channel[debugCh].trigger.high, rx_data->channel[debugCh].trigger.highD, rx_data->channel[debugCh].trigger.latchHigh);
		debugCh++;
		debugLine++;
		break;
	case 39:
		debugLine++;
		break;
	case 40:
		debugLine = 100;
		break;
	case 100:
		Serial.printf("\n");
		debugLine = 0;
		break;

	default:
		debugLine++;
		break;
	}
#else
#if defined SBUS_COMMUNICATION
	int len = rx_data->getBufferLen(); // 25
#elif defined CRSF_COMMUNICATION
	int len = rx_data->getBufferLen(); // 25
	// if ((rx_data->getBuffer(0) == 0xC8) && (rx_data->getBuffer(2)==0x16))
	if (rx_data->getBuffer(0) == 0xC8)
		len = rx_data->getBuffer(1) + 2;
	// int len = 26;
#endif
	// SBUS - CRSF buffer
	// Serial.printf("%8lu ms,", cur_ms);
	// Serial.printf("RX buffer : ");
	for (int i = 0; i < len; i++)
	{
		Serial.printf("%02X ", rx_data->getBuffer(i));
	}
	Serial.printf("\n");
	debugLine = 0;

#endif
}

// =======================================================================================================
// Sound functions
// =======================================================================================================
#pragma region SOUND_FUNCTIONS

/// @brief DAC Offset fader : DAC offset slowly to prevent it from popping, if ESP32 powered up after amplifier
void dacOffsetFade()
{
	// volatile uint8_t dacOffset = 0;      // 128, but needs to be ramped up slowly to prevent popping noise, if switched on
	static uint32_t dacOffsetMicros;
	static boolean dacInit = false;

	if (!dacInit)
	{
		if (micros() - dacOffsetMicros > 100)
		{ // Every 0.1ms
			dacOffsetMicros = micros();
			dacOffset++; // fade DAC offset slowly to prevent it from popping, if ESP32 powered up after amplifier
			if (dacOffset == 128)
				dacInit = true;
		}
	}
}

/// @brief Sounds trigger
void SoundTriggerSet()
{
	// Master volume
	// if (rx_data->channel[5].trigger.highEdgeD && (currentPwm == 0))
	if (MASTER_VOL_TRIG && (currentPwm == 0))
	{
		if (volumeIndex < numberOfVolumeSteps - 1)
			volumeIndex++; // Switch volume steps
		else
			volumeIndex = 0;
	}
	masterVolume = masterVolumePercentage[volumeIndex]; // Write volume

	// Horn
	// if (rx_data->channel[10].trigger.lowEdge) hornLatch = true;
	if (HORN_TRIG)
		hornLatch = true;
	// hornTrigger = HORN_LATCH;		//rx_data->channel[2].trigger.low;
	// if (HORN_TRIG) hornTrigger = true;

	// if (rx_data->channel[10].trigger.highEdge) sound1Trigger = true;
	if (SOUND1_TRIG)
		sound1Trigger = true;

	// Winch sound
	// if (trigCh6.high || trigCh6.low) sound2Trigger = true;
	// if (rx_data->channel[6].trigger.high || rx_data->channel[6].trigger.low) sound2Trigger = true;
	if (SOUND2_TRIG)
		sound2Trigger = true;
}

/// @brief Volume calculations for sound generator
void VolumeSet()
{
	static uint32_t throttleFaderMicros;
	static boolean blowoffLock;

	// Temp
	bool escIsBraking = false;
	bool brakeDetect = false;
	bool engineRunning = (engineState == RUNNING);

	// Trottle [0..500]	-- TO BE CHANGE TO PWM [0..100]
	currentThrottle = currentPwm * 5;

	// As a base for some calculations below, fade the current throttle to make it more natural
	if (micros() - throttleFaderMicros > 500)
	{ // Every 0.5ms
		throttleFaderMicros = micros();
		// Throttle Fader
		if (currentThrottleFaded < currentThrottle && !escIsBraking && currentThrottleFaded < 499)
			currentThrottleFaded += 2;
		if ((currentThrottleFaded > currentThrottle || escIsBraking) && currentThrottleFaded > 2)
			currentThrottleFaded -= 2;

		// Calculate throttle dependent engine idle volume
		if (!escIsBraking && !brakeDetect && engineRunning)
			throttleDependentVolume = map(currentThrottleFaded, 0, 500, engineIdleVolumePercentage, fullThrottleVolumePercentage);
		// else throttleDependentVolume = engineIdleVolumePercentage; // TODO
		else
		{
			if (throttleDependentVolume > engineIdleVolumePercentage)
				throttleDependentVolume--;
			else
				throttleDependentVolume = engineIdleVolumePercentage;
		}

		// Calculate throttle dependent engine rev volume
		if (!escIsBraking && !brakeDetect && engineRunning)
			throttleDependentRevVolume = map(currentThrottleFaded, 0, 500, engineRevVolumePercentage, fullThrottleVolumePercentage);
		// else throttleDependentRevVolume = engineRevVolumePercentage; // TODO
		else
		{
			if (throttleDependentRevVolume > engineRevVolumePercentage)
				throttleDependentRevVolume--;
			else
				throttleDependentRevVolume = engineRevVolumePercentage;
		}

		/*
		// Calculate throttle dependent Diesel knock volume
		if (!escIsBraking && !brakeDetect && engineRunning && (currentThrottleFaded > dieselKnockStartPoint))
			throttleDependentKnockVolume = map(currentThrottleFaded, dieselKnockStartPoint, 500, dieselKnockIdleVolumePercentage, 100);
		// else throttleDependentKnockVolume = dieselKnockIdleVolumePercentage;
		else
		{
			if (throttleDependentKnockVolume > dieselKnockIdleVolumePercentage)
				throttleDependentKnockVolume--;
			else
				throttleDependentKnockVolume = dieselKnockIdleVolumePercentage;
		}

		// Calculate engine rpm dependent jake brake volume
		if (engineState == RUNNING) 	//(engineRunning)
			rpmDependentJakeBrakeVolume = map(currentRpm, 0, 500, jakeBrakeIdleVolumePercentage, 100);
		else
			rpmDependentJakeBrakeVolume = jakeBrakeIdleVolumePercentage;

#if defined RPM_DEPENDENT_KNOCK // knock volume also depending on engine rpm
		// Calculate RPM dependent Diesel knock volume
		if (currentRpm > 400)
			rpmDependentKnockVolume = map(currentRpm, knockStartRpm, 500, minKnockVolumePercentage, 100);
		else
			rpmDependentKnockVolume = minKnockVolumePercentage;
#endif

		// Calculate engine rpm dependent turbo volume
		if (engineRunning)
			throttleDependentTurboVolume = map(currentRpm, 0, 500, turboIdleVolumePercentage, 100);
		else
			throttleDependentTurboVolume = turboIdleVolumePercentage;

		// Calculate engine rpm dependent cooling fan volume
		if (engineRunning && (currentRpm > fanStartPoint))
			throttleDependentFanVolume = map(currentRpm, fanStartPoint, 500, fanIdleVolumePercentage, 100);
		else
			throttleDependentFanVolume = fanIdleVolumePercentage;

		// Calculate throttle dependent supercharger volume
		if (!escIsBraking && !brakeDetect && engineRunning && (currentRpm > chargerStartPoint))
			throttleDependentChargerVolume = map(currentThrottleFaded, chargerStartPoint, 500, chargerIdleVolumePercentage, 100);
		else
			throttleDependentChargerVolume = chargerIdleVolumePercentage;

		// Calculate engine rpm dependent wastegate volume
		if (engineRunning)
			rpmDependentWastegateVolume = map(currentRpm, 0, 500, wastegateIdleVolumePercentage, 100);
		else
			rpmDependentWastegateVolume = wastegateIdleVolumePercentage;
		*/
	}
}

/// @brief Variable playback timer : for engine sounds
/// @return void.
void IRAM_ATTR variablePlaybackTimer()
{
	static uint32_t curEngineSample = 0;		  // Index of currently loaded engine sample
	static uint32_t curRevSample = 0;			  // Index of currently loaded engine rev sample
	static uint32_t curTurboSample = 0;			  // Index of currently loaded turbo sample
	static uint32_t curFanSample = 0;			  // Index of currently loaded fan sample
	static uint32_t curChargerSample = 0;		  // Index of currently loaded charger sample
	static uint32_t curStartSample = 0;			  // Index of currently loaded start sample
	static uint32_t curJakeBrakeSample = 0;		  // Index of currently loaded jake brake sample
	static uint32_t curHydraulicPumpSample = 0;	  // Index of currently loaded hydraulic pump sample
	static uint32_t curTrackRattleSample = 0;	  // Index of currently loaded train track rattle sample
	static uint32_t lastDieselKnockSample = 0;	  // Index of last Diesel knock sample
	static uint16_t attenuator = 0;				  // Used for volume adjustment during shutdown
	static uint32_t attenuatorMillis = 0;		  // Used for volume adjustment during shutdown
	static uint16_t speedPercentage = 0;		  // Slows the engine down during shutdown
	static int32_t a, a1, a2, a3, b, c, d, e = 0; // Input signals for mixer: a = engine, b = additional sound, c = turbo sound, d = fan sound, e = supercharger sound
	static int32_t f = 0;						  // Input signals for mixer: f = hydraulic pump
	static int32_t g = 0;						  // Input signals for mixer: g = train track rattle
	uint16_t a1Multi = 0;						  // Volume multipliers

	// Running core
	if (mtVariables.coreVT < 0)
		mtVariables.coreVT = xPortGetCoreID();
	//
	mtVariables.isrVT_Cnt++;
	//
	switch (engineState)
	{
	case STOPPED:												  // Engine stopped
		variableTimerTicks = 4000000 / startSampleRate;			  // our fixed sampling rate
		timerAlarmWrite(variableTimer, variableTimerTicks, true); // change timer ticks, autoreload true
		//
		a = 0; // volume = zero
		if (engineOn)
		{
			engineState = START;
			// engineState = STARTING;
			// curStartSample = 0;
			// engineStart = true;
		}
		break;

	case START: // Engine start --------------------------------------------------------------------
		// variableTimerTicks = 4000000 / startSampleRate;			  // our fixed sampling rate
		// timerAlarmWrite(variableTimer, variableTimerTicks, true); // change timer ticks, autoreload true
		curStartSample = 0;
		engineState = STARTING;
		break;

	case STARTING:												  // Engine start --------------------------------------------------------------------
		variableTimerTicks = 4000000 / startSampleRate;			  // our fixed sampling rate
		timerAlarmWrite(variableTimer, variableTimerTicks, true); // change timer ticks, autoreload true
		//
		if (curStartSample < startSampleCount - 1)
		{
			// a = startSamples[curStartSample];
			a = (startSamples[curStartSample] * throttleDependentVolume / 100 * startVolumePercentage / 100);
			curStartSample++;
		}
		else
		{
			curStartSample = 0;
			engineState = RUN;
			// engineState = RUNNING;
			// curEngineSample = 0;
			// engineStart = false;
			// engineRunning = true;
			// airBrakeTrigger = true;
		}
		dacWrite(DAC1, constrain(a + 128, 0, 255));
		break;

	case RUN: // Engine running ------------------------------------------------------------------
		// Engine idle & revving sounds (mixed together according to engine rpm, new in v5.0)
		// variableTimerTicks = engineSampleRate;					  // our variable idle sampling rate!
		// timerAlarmWrite(variableTimer, variableTimerTicks, true);   // change timer ticks, autoreload true
		curEngineSample = 0;
		engineState = RUNNING;
		break;

	case RUNNING: // Engine running ------------------------------------------------------------------
		// Engine idle & revving sounds (mixed together according to engine rpm, new in v5.0)
		variableTimerTicks = engineSampleRate;					  // our variable idle sampling rate!
		timerAlarmWrite(variableTimer, variableTimerTicks, true); // change timer ticks, autoreload true
		//
		a2 = 0;
		a3 = 0;
		if (curEngineSample < sampleCount - 10)
		{
			curEngineSample++;
			a1 = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100); // Idle sound
																										  // a1 = (samples[curEngineSample] * 20 / 100); // Idle sound
			// a3 = 0;

#ifdef REV_SOUND
			// Optional rev sound, recorded at medium rpm. Note, that it needs to represent the same number of ignition cycles as the
			// idle sound. For example 4 or 8 for a V8 engine. It also needs to have about the same length. In order to adjust the length
			// or "revSampleCount", change the "Rate" setting in Audacity until it is about the same.
			a2 = (revSamples[curRevSample] * throttleDependentRevVolume / 100 * revVolumePercentage / 100); // Rev sound
			if (curRevSample < revSampleCount)
				curRevSample++;
#endif
		}
		else
		{
			curEngineSample = 0;
			curRevSample = 0;
			// a1 = (samples[curEngineSample] * 20 / 100); // Idle sound
			// if (jakeBrakeRequest)
			//	engineJakeBraking = true;
			// lastDieselKnockSample = 0;
			// dieselKnockTrigger = true;
			// dieselKnockTriggerFirst = true;
		}
		//
		// Engine sound mixer
		//
#ifdef REV_SOUND
		// if (rx_data->channel[8].rx > 1750)
		//{
		//  Mixing the idle and rev sounds together, according to engine rpm
		//  Below the "revSwitchPoint" target, the idle volume precentage is 90%, then falling to 0% @ max. rpm.
		//  The total of idle and rev volume percentage is always 100%
		if (currentRpm > revSwitchPoint)
			a1Multi = map(currentRpm, idleEndPoint, revSwitchPoint, 0, idleVolumeProportionPercentage);
		else
			a1Multi = idleVolumeProportionPercentage; // 90 - 100% proportion
		if (currentRpm > idleEndPoint)
			a1Multi = 0;
		//
		a1 = a1 * a1Multi / 100;		 // Idle volume
		a2 = a2 * (100 - a1Multi) / 100; // Rev volume
		a = a1 + a2 + a3;				 // Idle and rev sounds mixed together
										 //}
										 // else
		//{
		//	a = a1 + a3; // Idle sound only
		//}
#else
		/ a = a1 + a3; // Idle sound only
#endif
		// Stop command
		if (!engineOn)
		{
			speedPercentage = 100;
			attenuator = 1;
			// engineState = STOP;
			engineState = STOPPING;
			// engineStop = true;
			// engineRunning = false;
		}
		break;

	case STOP: // Engine stop --------------------------------------------------------------------
		// variableTimerTicks = 4000000 / sampleRate * speedPercentage / 100; // our fixed sampling rate
		// timerAlarmWrite(variableTimer, variableTimerTicks, true);		   	 // change timer ticks, autoreload true
		engineState = STOPPING;
		break;

	case STOPPING: // Engine stop --------------------------------------------------------------------
		// Engine idle sounds
		variableTimerTicks = 4000000 / sampleRate * speedPercentage / 100; // our fixed sampling rate
		timerAlarmWrite(variableTimer, variableTimerTicks, true);		   // change timer ticks, autoreload true
		//
		if (curEngineSample < sampleCount - 1)
		{
			a = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100 / attenuator);
			curEngineSample++;
		}
		else
		{
			curEngineSample = 0;
		}
		// Fade engine sound out
		if (millis() - attenuatorMillis > 100)
		{ // Every 50ms
			attenuatorMillis = millis();
			attenuator++;		   // attenuate volume
			speedPercentage += 20; // make it slower (10)
		}
		//
		if (attenuator >= 50 || speedPercentage >= 500)
		{
			a = 0;
			speedPercentage = 100;
			// parkingBrakeTrigger = true;
			// engineState = PARKING_BRAKE;
			// engineState = OFF;
			engineState = STOPPED;
			// engineStop = false;
		}
		break;

	case PARKING_BRAKE: // Parking brake bleeding air sound after engine is off
		/*
		if (!parkingBrakeTrigger)
		{
			engineState = OFF;
		}*/
		break;

	default:
		break;

	} // end of switch case

	// DAC output (groups a, b, c mixed together)
	// - Mix signals : add 128 offset, write  to DAC
	// dacWrite(DAC1, constrain(((a * 8 / 10) + (b / 2) + (c / 5) + (d / 5) + (e / 5) + f + g) * masterVolume / 100 + dacOffset, 0, 255));
	dacWrite(DAC1, constrain(a * masterVolume / 100 + dacOffset, 0, 255));
	// dacWrite(DAC1, constrain(a + 128, 0, 255));
}

/// @brief Fixed p=Playback timer : for others sounds
/// @return
void IRAM_ATTR fixedPlaybackTimer()
{
	// static uint32_t curHornSample = 0;							  // Index of currently loaded horn sample
	// static uint32_t curSirenSample = 0;							  // Index of currently loaded siren sample
	// static uint32_t curSound1Sample = 0;						  // Index of currently loaded sound1 sample
	// static uint32_t curSound2Sample = 0;						  // Index of currently loaded sound2 sample - Add PHL
	// static uint32_t curReversingSample = 0;						  // Index of currently loaded reversing beep sample
	// static uint32_t curIndicatorSample = 0;						  // Index of currently loaded indicator tick sample
	// static uint32_t curWastegateSample = 0;						  // Index of currently loaded wastegate sample
	// static uint32_t curBrakeSample = 0;							  // Index of currently loaded brake sound sample
	// static uint32_t curParkingBrakeSample = 0;					  // Index of currently loaded brake sound sample
	// static uint32_t curShiftingSample = 0;						  // Index of currently loaded shifting sample
	// static uint32_t curDieselKnockSample = 0;					  // Index of currently loaded Diesel knock sample
	// static uint32_t curCouplingSample = 0;						  // Index of currently loaded trailer coupling sample
	// static uint32_t curUncouplingSample = 0;					  // Index of currently loaded trailer uncoupling sample
	// static uint32_t curHydraulicFlowSample = 0;					  // Index of currently loaded hydraulic flow sample
	// static uint32_t curTrackRattleSample = 0;					  // Index of currently loaded track rattle sample
	// static uint32_t curBucketRattleSample = 0;					  // Index of currently loaded bucket rattle sample
	// static uint32_t curTireSquealSample = 0;					  // Index of currently loaded tire squeal sample
	// static uint32_t curOutOfFuelSample = 0;						  // Index of currently loaded out of fuel sample
	static int32_t a, a1, a2 = 0;									   // Input signals "a" for mixer
	static int32_t b, b0, bs2, b1, b2, b3, b4, b5, b6, b7, b8, b9 = 0; // Input signals "b" for mixer
	static int32_t c, c1, c2, c3 = 0;								   // Input signals "c" for mixer
	static int32_t d, d1, d2 = 0;									   // Input signals "d" for mixer
	// static boolean knockSilent = 0;								  // This knock will be more silent
	// static boolean knockMedium = 0;								  // This knock will be medium
	// static uint8_t curKnockCylinder = 0;						  // Index of currently ignited zylinder
	//

	// Running core
	if (mtVariables.coreFT < 0)
		mtVariables.coreFT = xPortGetCoreID();

	// For all sound - 22050Hz
	fixedTimerTicks = 181;								// 4000000 / 22050;			// our fixed sampling rate
	timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // change timer ticks, autoreload true

	//*************************************************************************
	//* Group "a" (Horn and Siren sounds)
	//*************************************************************************
	// Horn sound
#ifdef HORN_LOOP
	a1 = horn.playLoop(hornTrigger, hornLatch, hornLoopBegin, hornLoopEnd); // Loop, if trigger still present
#else
	a1 = horn.play(hornLatch);
#endif
	// Siren
#ifdef SIREN_LOOP
	a2 = siren.playLoop(sirenTrigger, sirenLatch, sirenLoopBegin, sirenLoopEnd);
#else
	a2 = siren.play(sirenLatch);
#endif

	//*************************************************************************
	//* Group "b" (other sounds)
	//*************************************************************************
	b0 = sound1.play(sound1Trigger);
#ifdef SOUND2
	bs2 = sound2.play(sound2Trigger);
#endif
#ifdef SOUND3
	bs3 = sound2.play(sound3Trigger);
#endif

	// Reversing beep sound "b1"
#if reversingSampleCount > 10
	if (engineRunning && escInReverse)
		reversingTrigger = true;
	b1 = reversing.play(reversingTrigger);
#endif

	// Indicator tick sound "b2"
// #ifndef NO_INDICATOR_SOUND
#if indicatorSampleCount > 10
	b2 = indicator.play(indicatorSoundOn);
#endif

	// Wastegate (blowoff) sound, triggered after rapid throttle drop
#if wastegateSampleCount > 10
	b3 = wastegate.play(wastegateTrigger);
#endif

	// Air brake release sound, triggered after stop
#if brakeSampleCount > 10
	b4 = airBrake.play(airBrakeTrigger);
#endif

	// Air parking brake attaching sound, triggered after engine off
#if parkingBrakeSampleCount > 10
	b5 = parkingBrake.play(parkingBrakeTrigger);
#endif

	// Pneumatic gear shifting sound, triggered while shifting the TAMIYA 3 speed transmission ------
#if shiftingSampleCount > 10
	if (engineRunning && !automatic && !doubleClutch)
		b6 = shifting.play(shiftingTrigger);
#endif

#if not defined EXCAVATOR_MODE
	// Trailer coupling sound, triggered by switch -----------------------------------------------
#ifdef COUPLING_SOUND
	b8 = coupling.play(couplingTrigger);
	b9 = uncoupling.play(uncouplingTrigger);
#endif
#endif

	//*************************************************************************
	// Group "c" (excavator sounds)
	//*************************************************************************
#if defined EXCAVATOR_MODE
	// Hydraulic fluid flow sound
	c1 = hydraulicFlow.play();
	// Track rattle sound
	c2 = trackRattle.play();
	// Bucket rattle sound
	c3 = bucketRattle.play(bucketRattleTrigger);
	/*
	// Hydraulic fluid flow sound
	if (curHydraulicFlowSample < hydraulicFlowSampleCount - 1)
	{
		c1 = (hydraulicFlowSamples[curHydraulicFlowSample] * hydraulicFlowVolumePercentage / 100 * hydraulicFlowVolume / 100);
		curHydraulicFlowSample++;
	}
	else
		curHydraulicFlowSample = 0;
	// Track rattle sound
	if (curTrackRattleSample < trackRattleSampleCount - 1)
	{
		c2 = (trackRattleSamples[curTrackRattleSample] * trackRattleVolumePercentage / 100 * trackRattleVolume / 100);
		curTrackRattleSample++;
	}
	else
		curTrackRattleSample = 0;
	*/
#endif

	// Mixing sounds together
	a = a1 + a2;  // Horn & siren
	b = b0 + bs2; // Other sounds
	// b = b0 * 5 + bs2; 	// Other sounds
	// b = b0 * 5 + b1 + b2 / 2 + b3 + b4 + b5 + b6 + b7 + b8 + b9; // Other sounds
	// c = c1 + c2 + c3;											 // Excavator sounds
	// d = d1 + d2;												 // Additional sounds

	// DAC output (groups mixed together)
	// Mix signals, add 128 offset, write result to DAC
	dacWrite(DAC2, constrain((((a * 8 / 10) + b + c + d) * masterVolume / 100) + dacOffset, 0, 255));
	// dacWrite(DAC2, constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255));
	// dacWrite(DAC2, constrain( a2 * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
}

#pragma endregion SOUND_FUNCTIONS

// DIYGuy999 
// =======================================================================================================
// INTERRUPT FOR VARIABLE SPEED PLAYBACK (Engine sound, turbo sound)
// =======================================================================================================
/* void IRAM_ATTR variablePlaybackTimer()
{

	// coreId = xPortGetCoreID(); // Running on core 1

	static uint32_t attenuatorMillis = 0;
	static uint32_t curEngineSample = 0;		  // Index of currently loaded engine sample
	static uint32_t curRevSample = 0;			  // Index of currently loaded engine rev sample
	static uint32_t curTurboSample = 0;			  // Index of currently loaded turbo sample
	static uint32_t curFanSample = 0;			  // Index of currently loaded fan sample
	static uint32_t curChargerSample = 0;		  // Index of currently loaded charger sample
	static uint32_t curStartSample = 0;			  // Index of currently loaded start sample
	static uint32_t curJakeBrakeSample = 0;		  // Index of currently loaded jake brake sample
	static uint32_t curHydraulicPumpSample = 0;	  // Index of currently loaded hydraulic pump sample
	static uint32_t curTrackRattleSample = 0;	  // Index of currently loaded train track rattle sample
	static uint32_t lastDieselKnockSample = 0;	  // Index of last Diesel knock sample
	static uint16_t attenuator = 0;				  // Used for volume adjustment during engine switch off
	static uint16_t speedPercentage = 0;		  // slows the engine down during shutdown
	static int32_t a, a1, a2, a3, b, c, d, e = 0; // Input signals for mixer: a = engine, b = additional sound, c = turbo sound, d = fan sound, e = supercharger sound
	static int32_t f = 0;						  // Input signals for mixer: f = hydraulic pump
	static int32_t g = 0;						  // Input signals for mixer: g = train track rattle
	uint8_t a1Multi = 0;						  // Volume multipliers

	// portENTER_CRITICAL_ISR(&variableTimerMux); // disables C callable interrupts (on the current core) and locks the mutex by the current core.

	switch (engineState)
	{

	case OFF:													  // Engine off -----------------------------------------------------------------------
		variableTimerTicks = 4000000 / startSampleRate;			  // our fixed sampling rate
		timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

		a = 0; // volume = zero
		if (engineOn)
		{
			engineState = STARTING;
			engineStart = true;
		}
		break;

	case STARTING:												  // Engine start --------------------------------------------------------------------
		variableTimerTicks = 4000000 / startSampleRate;			  // our fixed sampling rate
		timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

		if (curStartSample < startSampleCount - 1)
		{
#if defined STEAM_LOCOMOTIVE_MODE
			a = (startSamples[curStartSample] * startVolumePercentage / 100);
#else
			a = (startSamples[curStartSample] * throttleDependentVolume / 100 * startVolumePercentage / 100);
#endif
			curStartSample++;
		}
		else
		{
			curStartSample = 0;
			engineState = RUNNING;
			engineStart = false;
			engineRunning = true;
			airBrakeTrigger = true;
		}
		break;

	case RUNNING: // Engine running ------------------------------------------------------------------

		// Engine idle & revving sounds (mixed together according to engine rpm, new in v5.0)
		variableTimerTicks = engineSampleRate;					  // our variable idle sampling rate!
		timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

		if (!engineJakeBraking && !blowoffTrigger)
		{
			if (curEngineSample < sampleCount - 1)
			{
				a1 = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100); // Idle sound
				a3 = 0;
				curEngineSample++;

				// Optional rev sound, recorded at medium rpm. Note, that it needs to represent the same number of ignition cycles as the
				// idle sound. For example 4 or 8 for a V8 engine. It also needs to have about the same length. In order to adjust the length
				// or "revSampleCount", change the "Rate" setting in Audacity until it is about the same.
#ifdef REV_SOUND
				a2 = (revSamples[curRevSample] * throttleDependentRevVolume / 100 * revVolumePercentage / 100); // Rev sound
				if (curRevSample < revSampleCount)
					curRevSample++;
#endif

				// Trigger throttle dependent Diesel ignition "knock" sound (played in the fixed sample rate interrupt)
				if (curEngineSample - lastDieselKnockSample > (sampleCount / dieselKnockInterval))
				{
					dieselKnockTrigger = true;
					dieselKnockTriggerFirst = false;
					lastDieselKnockSample = curEngineSample;
				}
			}
			else
			{
				curEngineSample = 0;
				if (jakeBrakeRequest)
					engineJakeBraking = true;
#ifdef REV_SOUND
				curRevSample = 0;
#endif
				lastDieselKnockSample = 0;
				dieselKnockTrigger = true;
				dieselKnockTriggerFirst = true;
			}
			curJakeBrakeSample = 0;
		}
		else
		{ // Jake brake sound ----
#ifdef JAKE_BRAKE_SOUND
			a3 = (jakeBrakeSamples[curJakeBrakeSample] * rpmDependentJakeBrakeVolume / 100 * jakeBrakeVolumePercentage / 100); // Jake brake sound
			a2 = 0;
			a1 = 0;
			if (curJakeBrakeSample < jakeBrakeSampleCount - 1)
				curJakeBrakeSample++;
			else
			{
				curJakeBrakeSample = 0;
				if (!jakeBrakeRequest)
					engineJakeBraking = false;
			}

			curEngineSample = 0;
			curRevSample = 0;
#endif
		}

		// Engine sound mixer ----
#ifdef REV_SOUND
		// Mixing the idle and rev sounds together, according to engine rpm
		// Below the "revSwitchPoint" target, the idle volume precentage is 90%, then falling to 0% @ max. rpm.
		// The total of idle and rev volume percentage is always 100%

		if (currentRpm > revSwitchPoint)
			a1Multi = map(currentRpm, idleEndPoint, revSwitchPoint, 0, idleVolumeProportionPercentage);
		else
			a1Multi = idleVolumeProportionPercentage; // 90 - 100% proportion
		if (currentRpm > idleEndPoint)
			a1Multi = 0;

		a1 = a1 * a1Multi / 100;		 // Idle volume
		a2 = a2 * (100 - a1Multi) / 100; // Rev volume

		a = a1 + a2 + a3; // Idle and rev sounds mixed together
#else
		a = a1 + a3; // Idle sound only
#endif

		// Turbo sound ----------------------------------
		if (curTurboSample < turboSampleCount - 1)
		{
			c = (turboSamples[curTurboSample] * throttleDependentTurboVolume / 100 * turboVolumePercentage / 100);
			curTurboSample++;
		}
		else
		{
			curTurboSample = 0;
		}

		// Fan sound -----------------------------------
		if (curFanSample < fanSampleCount - 1)
		{
			d = (fanSamples[curFanSample] * throttleDependentFanVolume / 100 * fanVolumePercentage / 100);
			curFanSample++;
		}
		else
		{
			curFanSample = 0;
		}
#if defined GEARBOX_WHINING
		if (neutralGear)
			d = 0; // used for gearbox whining simulation, so not active in gearbox neutral
#endif

		// Supercharger sound --------------------------
		if (curChargerSample < chargerSampleCount - 1)
		{
			e = (chargerSamples[curChargerSample] * throttleDependentChargerVolume / 100 * chargerVolumePercentage / 100);
			curChargerSample++;
		}
		else
		{
			curChargerSample = 0;
		}

		// Hydraulic pump sound -----------------------
#if defined EXCAVATOR_MODE
		if (curHydraulicPumpSample < hydraulicPumpSampleCount - 1)
		{
			f = (hydraulicPumpSamples[curHydraulicPumpSample] * hydraulicPumpVolumePercentage / 100 * hydraulicPumpVolume / 100);
			curHydraulicPumpSample++;
		}
		else
		{
			curHydraulicPumpSample = 0;
		}
#endif

#if defined STEAM_LOCOMOTIVE_MODE
		// Track rattle sound -----------------------
		if (curTrackRattleSample < trackRattleSampleCount - 1)
		{
			g = (trackRattleSamples[curTrackRattleSample] * trackRattleVolumePercentage / 100 * trackRattleVolume / 100);
			curTrackRattleSample++;
		}
		else
		{
			curTrackRattleSample = 0;
		}
#endif

		if (!engineOn)
		{
			speedPercentage = 100;
			attenuator = 1;
			engineState = STOPPING;
			engineStop = true;
			engineRunning = false;
		}
		break;

	case STOPPING:														   // Engine stop --------------------------------------------------------------------
		variableTimerTicks = 4000000 / sampleRate * speedPercentage / 100; // our fixed sampling rate
		timerAlarmWrite(variableTimer, variableTimerTicks, true);		   // // change timer ticks, autoreload true

		if (curEngineSample < sampleCount - 1)
		{
			a = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100 / attenuator);
			curEngineSample++;
		}
		else
		{
			curEngineSample = 0;
		}

		// fade engine sound out
		if (millis() - attenuatorMillis > 100)
		{ // Every 50ms
			attenuatorMillis = millis();
			attenuator++;		   // attenuate volume
			speedPercentage += 20; // make it slower (10)
		}

		if (attenuator >= 50 || speedPercentage >= 500)
		{ // 50 & 500
			a = 0;
			speedPercentage = 100;
			parkingBrakeTrigger = true;
			engineState = PARKING_BRAKE;
			engineStop = false;
		}
		break;

	case PARKING_BRAKE: // parking brake bleeding air sound after engine is off ----------------------------

		if (!parkingBrakeTrigger)
		{
			engineState = OFF;
		}
		break;

	} // end of switch case

	// DAC output (groups a, b, c mixed together) ************************************************************************

	dacWrite(DAC1, constrain(((a * 8 / 10) + (b / 2) + (c / 5) + (d / 5) + (e / 5) + f + g) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write  to DAC
																																		// dacWrite(DAC1, constrain(a * masterVolume / 100 + dacOffset, 0, 255));
																																		// dacWrite(DAC1, constrain(a + 128, 0, 255));

	// portEXIT_CRITICAL_ISR(&variableTimerMux);
}
*/

// =======================================================================================================
// INTERRUPT FOR FIXED SPEED PLAYBACK (Horn etc., played in parallel with engine sound)
// =======================================================================================================
/* void IRAM_ATTR fixedPlaybackTimer()
{

	// coreId = xPortGetCoreID(); // Running on core 1

	static uint32_t curHornSample = 0;							  // Index of currently loaded horn sample
	static uint32_t curSirenSample = 0;							  // Index of currently loaded siren sample
	static uint32_t curSound1Sample = 0;						  // Index of currently loaded sound1 sample
	static uint32_t curReversingSample = 0;						  // Index of currently loaded reversing beep sample
	static uint32_t curIndicatorSample = 0;						  // Index of currently loaded indicator tick sample
	static uint32_t curWastegateSample = 0;						  // Index of currently loaded wastegate sample
	static uint32_t curBrakeSample = 0;							  // Index of currently loaded brake sound sample
	static uint32_t curParkingBrakeSample = 0;					  // Index of currently loaded brake sound sample
	static uint32_t curShiftingSample = 0;						  // Index of currently loaded shifting sample
	static uint32_t curDieselKnockSample = 0;					  // Index of currently loaded Diesel knock sample
	static uint32_t curCouplingSample = 0;						  // Index of currently loaded trailer coupling sample
	static uint32_t curUncouplingSample = 0;					  // Index of currently loaded trailer uncoupling sample
	static uint32_t curHydraulicFlowSample = 0;					  // Index of currently loaded hydraulic flow sample
	static uint32_t curTrackRattleSample = 0;					  // Index of currently loaded track rattle sample
	static uint32_t curBucketRattleSample = 0;					  // Index of currently loaded bucket rattle sample
	static uint32_t curTireSquealSample = 0;					  // Index of currently loaded tire squeal sample
	static uint32_t curOutOfFuelSample = 0;						  // Index of currently loaded out of fuel sample
	static int32_t a, a1, a2 = 0;								  // Input signals "a" for mixer
	static int32_t b, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9 = 0; // Input signals "b" for mixer
	static int32_t c, c1, c2, c3 = 0;							  // Input signals "c" for mixer
	static int32_t d, d1, d2 = 0;								  // Input signals "d" for mixer
	static boolean knockSilent = 0;								  // This knock will be more silent
	static boolean knockMedium = 0;								  // This knock will be medium
	static uint8_t curKnockCylinder = 0;						  // Index of currently ignited zylinder

	// portENTER_CRITICAL_ISR(&fixedTimerMux);

	// Group "a" (horn & siren) ******************************************************************

	if (hornTrigger || hornLatch)
	{
		fixedTimerTicks = 4000000 / hornSampleRate;			// our fixed sampling rate
		timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

		if (curHornSample < hornSampleCount - 1)
		{
			a1 = (hornSamples[curHornSample] * hornVolumePercentage / 100);
			curHornSample++;
#ifdef HORN_LOOP // Optional "endless loop" (points to be defined manually in horn file)
			if (hornTrigger && curHornSample == hornLoopEnd)
				curHornSample = hornLoopBegin; // Loop, if trigger still present
#endif
		}
		else
		{ // End of sample
			curHornSample = 0;
			a1 = 0;
			hornLatch = false;
		}
	}

	if (sirenTrigger || sirenLatch)
	{
		fixedTimerTicks = 4000000 / sirenSampleRate;		// our fixed sampling rate
		timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

#if defined SIREN_STOP
		if (!sirenTrigger)
		{
			sirenLatch = false;
			curSirenSample = 0;
			a2 = 0;
		}
#endif

		if (curSirenSample < sirenSampleCount - 1)
		{
			a2 = (sirenSamples[curSirenSample] * sirenVolumePercentage / 100);
			curSirenSample++;
#ifdef SIREN_LOOP // Optional "endless loop" (points to be defined manually in siren file)
			if (sirenTrigger && curSirenSample == sirenLoopEnd)
				curSirenSample = sirenLoopBegin; // Loop, if trigger still present
#endif
		}
		else
		{ // End of sample
			curSirenSample = 0;
			a2 = 0;
			sirenLatch = false;
		}
	}
	if (curSirenSample > 10 && curSirenSample < 500)
		cannonFlash = true; // Tank cannon flash triggering in TRACKED_MODE
	else
		cannonFlash = false;

	// Group "b" (other sounds) **********************************************************************

	// Sound 1 "b0" ----
	if (sound1trigger)
	{
		fixedTimerTicks = 4000000 / sound1SampleRate;		// our fixed sampling rate
		timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

		if (curSound1Sample < sound1SampleCount - 1)
		{
			b0 = (sound1Samples[curSound1Sample] * sound1VolumePercentage / 100);
			curSound1Sample++;
		}
		else
		{
			sound1trigger = false;
		}
	}
	else
	{
		curSound1Sample = 0; // ensure, next sound will start @ first sample
		b0 = 0;
	}

	// Reversing beep sound "b1" ----
	if (engineRunning && escInReverse)
	{
		fixedTimerTicks = 4000000 / reversingSampleRate;	// our fixed sampling rate
		timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

		if (curReversingSample < reversingSampleCount - 1)
		{
			b1 = (reversingSamples[curReversingSample] * reversingVolumePercentage / 100);
			curReversingSample++;
		}
		else
		{
			curReversingSample = 0;
		}
	}
	else
	{
		curReversingSample = 0; // ensure, next sound will start @ first sample
		b1 = 0;
	}

	// Indicator tick sound "b2" ----------------------------------------------------------------------
#if not defined NO_INDICATOR_SOUND
	if (indicatorSoundOn)
	{
		fixedTimerTicks = 4000000 / indicatorSampleRate;	// our fixed sampling rate
		timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

		if (curIndicatorSample < indicatorSampleCount - 1)
		{
			b2 = (indicatorSamples[curIndicatorSample] * indicatorVolumePercentage / 100);
			curIndicatorSample++;
		}
		else
		{
			indicatorSoundOn = false;
		}
	}
	else
	{
		curIndicatorSample = 0; // ensure, next sound will start @ first sample
		b2 = 0;
	}
#endif

	// Wastegate (blowoff) sound, triggered after rapid throttle drop -----------------------------------
	if (wastegateTrigger)
	{
		if (curWastegateSample < wastegateSampleCount - 1)
		{
			b3 = (wastegateSamples[curWastegateSample] * rpmDependentWastegateVolume / 100 * wastegateVolumePercentage / 100);
			curWastegateSample++;
		}
		else
		{
			wastegateTrigger = false;
		}
	}
	else
	{
		b3 = 0;
		curWastegateSample = 0; // ensure, next sound will start @ first sample
	}

	// Air brake release sound, triggered after stop -----------------------------------------------
	if (airBrakeTrigger)
	{
		if (curBrakeSample < brakeSampleCount - 1)
		{
			b4 = (brakeSamples[curBrakeSample] * brakeVolumePercentage / 100);
			curBrakeSample++;
		}
		else
		{
			airBrakeTrigger = false;
		}
	}
	else
	{
		b4 = 0;
		curBrakeSample = 0; // ensure, next sound will start @ first sample
	}

	// Air parking brake attaching sound, triggered after engine off --------------------------------
	if (parkingBrakeTrigger)
	{
		if (curParkingBrakeSample < parkingBrakeSampleCount - 1)
		{
			b5 = (parkingBrakeSamples[curParkingBrakeSample] * parkingBrakeVolumePercentage / 100);
			curParkingBrakeSample++;
		}
		else
		{
			parkingBrakeTrigger = false;
		}
	}
	else
	{
		b5 = 0;
		curParkingBrakeSample = 0; // ensure, next sound will start @ first sample
	}

	// Pneumatic gear shifting sound, triggered while shifting the TAMIYA 3 speed transmission ------
	if (shiftingTrigger && engineRunning && !automatic && !doubleClutch)
	{
		if (curShiftingSample < shiftingSampleCount - 1)
		{
			b6 = (shiftingSamples[curShiftingSample] * shiftingVolumePercentage / 100);
			curShiftingSample++;
		}
		else
		{
			shiftingTrigger = false;
		}
	}
	else
	{
		b6 = 0;
		curShiftingSample = 0; // ensure, next sound will start @ first sample
	}

	// Diesel ignition "knock" is played in fixed sample rate section, because we don't want changing pitch! ------
	if (dieselKnockTriggerFirst)
	{
		dieselKnockTriggerFirst = false;
		curKnockCylinder = 0;
	}

	if (dieselKnockTrigger)
	{
		dieselKnockTrigger = false;
		curKnockCylinder++; // Count ignition sequence
		curDieselKnockSample = 0;
	}

#ifdef V8 // (former ADAPTIVE_KNOCK_VOLUME, rename it in your config file!)
	// Ford or Scania V8 ignition sequence: 1 - 5 - 4 - 2* - 6 - 3 - 7 - 8* (* = louder knock pulses, because 2nd exhaust in same manifold after 90°)
	if (curKnockCylinder == 4 || curKnockCylinder == 8)
		knockSilent = false;
	else
		knockSilent = true;
#endif

#ifdef V8_MEDIUM // (former ADAPTIVE_KNOCK_VOLUME, rename it in your config file!)
	// This is EXPERIMENTAL!! TODO
	if (curKnockCylinder == 5 || curKnockCylinder == 1)
		knockMedium = false;
	else
		knockMedium = true;
#endif

#ifdef V8_468 // (Chevy 468, containing 16 ignition pulses)
	// 1th, 5th, 9th and 13th are the loudest
	// Ignition sequence: 1 - 8 - 4* - 3 - 6 - 5 - 7* - 2
	if (curKnockCylinder == 1 || curKnockCylinder == 5 || curKnockCylinder == 9 || curKnockCylinder == 13)
		knockSilent = false;
	else
		knockSilent = true;
#endif

#ifdef V2
	// V2 engine: 1st and 2nd knock pulses (of 4) will be louder
	if (curKnockCylinder == 1 || curKnockCylinder == 2)
		knockSilent = false;
	else
		knockSilent = true;
#endif

#ifdef R6
	// R6 inline 6 engine: 6th knock pulse (of 6) will be louder
	if (curKnockCylinder == 6)
		knockSilent = false;
	else
		knockSilent = true;
#endif

#ifdef R6_2
	// R6 inline 6 engine: 6th and 3rd knock pulse (of 6) will be louder
	if (curKnockCylinder == 6 || curKnockCylinder == 3)
		knockSilent = false;
	else
		knockSilent = true;
#endif

	if (curDieselKnockSample < knockSampleCount)
	{
#if defined RPM_DEPENDENT_KNOCK // knock volume also depending on engine rpm
		b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * rpmDependentKnockVolume / 100);
#elif defined EXCAVATOR_MODE // knock volume also depending on hydraulic load
		b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * hydraulicDependentKnockVolume / 100);
#else						 // Just depending on throttle
		b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100);
#endif
		curDieselKnockSample++;
		if (knockSilent && !knockMedium)
			b7 = b7 * dieselKnockAdaptiveVolumePercentage / 100; // changing knock volume according to engine type and cylinder!
		if (knockMedium)
			b7 = b7 * dieselKnockAdaptiveVolumePercentage / 75;
	}

#if not defined EXCAVATOR_MODE
	// Trailer coupling sound, triggered by switch -----------------------------------------------
#ifdef COUPLING_SOUND
	if (couplingTrigger)
	{
		if (curCouplingSample < couplingSampleCount - 1)
		{
			b8 = (couplingSamples[curCouplingSample] * couplingVolumePercentage / 100);
			curCouplingSample++;
		}
		else
		{
			couplingTrigger = false;
		}
	}
	else
	{
		b8 = 0;
		curCouplingSample = 0; // ensure, next sound will start @ first sample
	}

	// Trailer uncoupling sound, triggered by switch -----------------------------------------------
	if (uncouplingTrigger)
	{
		if (curUncouplingSample < uncouplingSampleCount - 1)
		{
			b9 = (uncouplingSamples[curUncouplingSample] * couplingVolumePercentage / 100);
			curUncouplingSample++;
		}
		else
		{
			uncouplingTrigger = false;
		}
	}
	else
	{
		b9 = 0;
		curUncouplingSample = 0; // ensure, next sound will start @ first sample
	}
#endif
#endif

	// Group "c" (excavator sounds) **********************************************************************

#if defined EXCAVATOR_MODE

	// Hydraulic fluid flow sound -----------------------
	if (curHydraulicFlowSample < hydraulicFlowSampleCount - 1)
	{
		c1 = (hydraulicFlowSamples[curHydraulicFlowSample] * hydraulicFlowVolumePercentage / 100 * hydraulicFlowVolume / 100);
		curHydraulicFlowSample++;
	}
	else
	{
		curHydraulicFlowSample = 0;
	}

	// Track rattle sound -----------------------
	if (curTrackRattleSample < trackRattleSampleCount - 1)
	{
		c2 = (trackRattleSamples[curTrackRattleSample] * trackRattleVolumePercentage / 100 * trackRattleVolume / 100);
		curTrackRattleSample++;
	}
	else
	{
		curTrackRattleSample = 0;
	}

	// Bucket rattle sound -----------------------
	if (bucketRattleTrigger)
	{
		if (curBucketRattleSample < bucketRattleSampleCount - 1)
		{
			c3 = (bucketRattleSamples[curBucketRattleSample] * bucketRattleVolumePercentage / 100);
			curBucketRattleSample++;
		}
		else
		{
			bucketRattleTrigger = false;
		}
	}
	else
	{
		c3 = 0;
		curBucketRattleSample = 0; // ensure, next sound will start @ first sample
	}
#endif

	// Group "d" (additional sounds) **********************************************************************

#if defined TIRE_SQUEAL
	// Tire squeal sound -----------------------
	if (curTireSquealSample < tireSquealSampleCount - 1)
	{
		d1 = (tireSquealSamples[curTireSquealSample] * tireSquealVolumePercentage / 100 * tireSquealVolume / 100);
		curTireSquealSample++;
	}
	else
	{
		d1 = 0;
		curTireSquealSample = 0;
	}
#endif

#if defined BATTERY_PROTECTION
	// Out of fuel sound, triggered by battery voltage -----------------------------------------------
	if (outOfFuelMessageTrigger)
	{
		if (curOutOfFuelSample < outOfFuelSampleCount - 1)
		{
			d2 = (outOfFuelSamples[curOutOfFuelSample] * outOfFuelVolumePercentage / 100);
			curOutOfFuelSample++;
		}
		else
		{
			outOfFuelMessageTrigger = false;
		}
	}
	else
	{
		d2 = 0;
		curOutOfFuelSample = 0; // ensure, next sound will start @ first sample
	}
#endif

	// Mixing sounds together **********************************************************************
	a = a1 + a2; // Horn & siren
	// if (a < 2 && a > -2) a = 0; // Remove noise floor TODO, experimental
	b = b0 * 5 + b1 + b2 / 2 + b3 + b4 + b5 + b6 + b7 + b8 + b9; // Other sounds
	c = c1 + c2 + c3;											 // Excavator sounds
	d = d1 + d2;												 // Additional sounds

	// DAC output (groups mixed together) ****************************************************************************

	// dacDebug = constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255); // Mix signals, add 128 offset, write result to DAC
	dacWrite(DAC2, constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
																											   // dacWrite(DAC2, constrain( a2 * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC
																											   // dacWrite(DAC2, 0);

	// portEXIT_CRITICAL_ISR(&fixedTimerMux);
}
*/

//
// EOF
//