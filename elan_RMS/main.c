/*
 * elan_RMS.c
 *
 * Created: 10/28/2019 11:24:39 PM
 * Author : Aleksij Kraljic
 */ 

#include <avr/io.h>
#include "AK_MPU6050_lib.h"
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include <stdlib.h>
#include "i2c_master.h"
#include <math.h>

void statusLED(uint8_t status);
void debug_state_change(uint8_t state_num);

/* touch_ctn - count of switch touch events */
volatile uint8_t touch_ctn = 0;

int main(void)
{
	/* ================== INITIALIZATION ===================*/
	
	/* state variable */
	uint8_t state = 0;
	
	/*  acc   - current accelerometer readings array {X,Y,Z}
		p_acc - previous accelerometer readings array {X,Y,Z}
		th    - motion threshold
	*/
	int16_t acc[3], p_acc[3], th = 150;
	
	/* total_ctn  - total count of iterations when detecting motion or nonmotion
	   motion_ctn - count of motion events
	   static_ctn - count of static events
	   motion_sum - sum of motion events
	   static_sum - sum of static events
	*/
	uint16_t total_ctn = 0, motion_ctn = 0, static_ctn = 0;
	uint16_t motion_sum = 0, static_sum;
	
	/* length of motion or static arrays
	Program loop period ~ 20ms, therefore the length is determined as:
		X_length = (X_period in miLLiseconds)/(20ms)
	motion_length = 5000ms/20ms = 250
	static_length = 5000ms/20ms = 250
	*/
	uint16_t motion_length = 100, motion_array[motion_length];
	uint16_t static_length = 50, static_array[static_length];
	
	/* glitch_count - maximum allowed number of events that belong to other class (static event when counting motion events) */
	uint8_t glitch_count = 50;
	
	/* T - period of blinking, N - number of blinks */
	uint8_t T_blink_nomotion = 35, T_blink_return = 100;
	uint8_t N_blink_nomotion = 50, N_blink_return = 3;
	
	/* DEBUG: serial char array output
	char debug_str[16]; */
	
	/* set status LED as output on PORTB4 and status indicator LED as output on PORTB2*/
	DDRB |= (1<<DDB4) | (1<<DDB2) | (1<<DDB3);
	PORTB |= (1<<PINB3);
	
	initUSART();
	
	/* set clock to 16MHz */
	clock_prescale_set(clock_div_1);
	
	/* disable ADC */
	ADCSRA = 0;
	PRR |= (1<<PRUSART0) | (1<<PRADC) | (1<<PRSPI);
	
	/* printLine("Always good times"); */
	
	MPU6050_init();
	
	/* interrupt settings */
	EICRA |= (1<<ISC10);
	PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT1);
	
	/* test MPU6050 IMU */
	if (MPU6050_test_I2C()) {
		/* printLine("=== IMU working properly ==="); */
		statusLED(1);
	}
	else {
		statusLED(1);
		/* printLine("=== IMU ERROR ==="); */
		for(uint8_t i = 0; i < 50; i++){
			statusLED(0);
			_delay_ms(50);
			statusLED(1);
			_delay_ms(50);
			/* printString("."); */
		}
	}
	
	/*
	MPU6050_set_accelFS() set the accelerometer full scale range.
	accelFS:
	0 - 2g, 16384 LSB/g
	1 - 4g, 8192 LSB/g
	2 - 8g, 4096 LSB/g <- SELECTED
	3 - 16g, 2048 LSB/g
	*/
	MPU6050_set_accelFS(2);
	
	statusLED(0);
	
	/* global interrupt enable */
	sei();
	
	/* initialization blinking */
	for(uint8_t i=0;i<30;i++){
		statusLED(1);
		_delay_ms(50);
		statusLED(0);
		_delay_ms(50);
	}
	
	/* ================= MAIN PROGRAM LOOP =================*/
    while (1) 
    {
		switch(state) {
			case 0:
				/* STATE 0: no lights, waiting for button presses, if >3 presses move to state 1*/
				/* printLine("state = 0"); */
				if (touch_ctn >= 3) {
					touch_ctn = 0;
					state = 1;
					debug_state_change(1);
				}
				break;
			case 1:
				/* STATE 1: 5s of quick blinking indicating no motion then move to STATE 2*/
				/* printLine("state = 1"); */
				for(uint8_t i=0;i<N_blink_nomotion;i++){
					statusLED(1);
					_delay_ms(T_blink_nomotion);
					statusLED(0);
					_delay_ms(T_blink_nomotion);
				}
				touch_ctn = 0;
				state = 2;
				debug_state_change(2);
				/* assign 0 to all elements in the motion array */
				for(uint16_t i=0;i<motion_length;i++){
					motion_array[i] = 0;
				}
				break;
			case 2:
				/* STATE 2: detect motion, if motion is longer than T_moving seconds, move to STATE 3 */
				/* printLine("state = 2"); */
				
				if (touch_ctn >= 3) {
					touch_ctn = 0;
					state = 0;
					debug_state_change(0);
					for(uint8_t i=0;i<N_blink_return;i++){
						statusLED(1);
						_delay_ms(T_blink_return);
						statusLED(0);
						_delay_ms(T_blink_return);
					}
				}
				
				MPU6050_get_accel(p_acc);
				_delay_ms(10);
				MPU6050_get_accel(acc);
				
				if (abs(acc[0]-p_acc[0])>th || abs(acc[1]-p_acc[1])>th || abs(acc[2]-p_acc[2])>th) {
					/* printLine("Motion Detected"); */
					motion_array[total_ctn] = 1;
					motion_ctn += 1;
					} else {
					/* printLine("No Motion"); */
					motion_array[total_ctn] = 0;
					static_ctn += 1;
				}
				
				total_ctn += 1;
				
				if (static_ctn >= glitch_count) {
					motion_ctn = 0;
					static_ctn = 0;
					total_ctn = 0;
					/* assign 0 to all elements in the motion array */
					for(uint16_t i=0;i<motion_length;i++){
						motion_array[i] = 0;
					}
				}
				
				if (total_ctn >= motion_length) {
					/* sum motion events */
					for(uint16_t i=0;i<motion_length;i++){
						motion_sum += motion_array[i];
					}
					if (motion_sum > motion_length-glitch_count) {
						/* motion period detected */
						state = 3;
						debug_state_change(3);
						/* printLine("3"); */
						motion_sum = 0;
						motion_ctn = 0;
						static_ctn = 0;
						total_ctn = 0;
						touch_ctn = 0;
						/* assign 0 to all elements in the static array */
						for(uint16_t i=0;i<static_length;i++){
							static_array[i] = 0;
						}
					} else {
						/* motion period not detected */
						motion_sum = 0;
						motion_ctn = 0;
						static_ctn = 0;
						total_ctn = 0;
					}
				}
				
				/*
				itoa(motion_ctn,debug_str,10);
				printLine(debug_str);
				*/
				break;
			case 3:
				/* STATE 3: detect static period, if static for T_static seconds move to STATE 1 */
				
				if (touch_ctn >= 3) {
					touch_ctn = 0;
					state = 0;
					debug_state_change(0);
					for(uint8_t i=0;i<N_blink_return;i++){
						statusLED(1);
						_delay_ms(T_blink_return);
						statusLED(0);
						_delay_ms(T_blink_return);
					}
				}
				
				MPU6050_get_accel(p_acc);
				_delay_ms(10);
				MPU6050_get_accel(acc);
				
				if (abs(acc[0]-p_acc[0])>th || abs(acc[1]-p_acc[1])>th || abs(acc[2]-p_acc[2])>th) {
					/* printLine("Motion Detected"); */
					static_array[total_ctn] = 0;
					motion_ctn += 1;
					} else {
					/* printLine("No Motion"); */
					static_array[total_ctn] = 1;
					static_ctn += 1;
				}
				
				total_ctn += 1;
				
				if (motion_ctn >= glitch_count) {
					motion_ctn = 0;
					static_ctn = 0;
					total_ctn = 0;
					/* assign 0 to all elements in the static array */
					for(uint16_t i=0;i<static_length;i++){
						static_array[i] = 0;
					}
				}
				
				if (total_ctn >= static_length) {
					/* sum static events */
					for(uint16_t i=0;i<static_length;i++){
						static_sum += static_array[i];
					}
					if (static_sum > static_length-glitch_count) {
						/* static period detected */
						state = 1;
						debug_state_change(1);
						static_sum = 0;
						static_ctn = 0;
						motion_ctn = 0;
						total_ctn = 0;
						touch_ctn = 0;
						/* assign 0 to all elements in the static array */
						for(uint16_t i=0;i<static_length;i++){
							static_array[i] = 0;
						}
					} else {
						/* static period not detected */
						static_sum = 0;
						static_ctn = 0;
						motion_ctn = 0;
						total_ctn = 0;
						/* assign 0 to all elements in the static array */
						for(uint16_t i=0;i<static_length;i++){
							static_array[i] = 0;
						}
					}
				}
				
				/*
				itoa(static_ctn,debug_str,10);
				printLine(debug_str);
				*/
				break;
		}
		_delay_ms(10);
    }
}

ISR(PCINT0_vect) {
	touch_ctn += 1;
	_delay_ms(5);
}

void statusLED(uint8_t status)
{
	if (status) {
		PORTB |= (1<<PORTB4);
	}
	else {
		PORTB &= ~(1<<PORTB4);
	}
	
}

void debug_state_change(uint8_t state_num)
{
	for(uint8_t i=0;i<state_num;i++) {
		PORTB |= (1<<PORTB2);
		_delay_ms(300);
		PORTB &= ~(1<<PORTB2);
		_delay_ms(300);
	}
}