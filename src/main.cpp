/*
 * Copyright (c) 2024 Kevin Köster.
 *
 * This file is part of betaflight-beeper 
 * (see https://github.com/Smeat/betaflight-beeper).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <util/delay.h>

#include <string.h>

// in case we compile as C code. But compiling with g++ saves a few bytes
#ifndef __cplusplus
#define bool uint8_t
#define true 1
#define false 0
#endif

#define STATIC_TEST static

#define SENSE_DDR DDRB
#define SENSE_PORT PORTB
#define SENSE_PIN PB4

#define BEEPER_IN_DDR DDRB
#define BEEPER_IN_PORT PORTB
#define BEEPER_IN_PIN PB3

#define BEEPER_OUT_DDR DDRB
#define BEEPER_OUT_PORT PORTB
#define BEEPER_OUT_PIN PB0

#define BUTTON_DDR DDRB
#define BUTTON_PORT PORTB
#define BUTTON_PIN PB1

#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PB2

#define BEEP_PATTERN_SIZE 2
#define BEEP_PATTERN_NUM 2

#define STAGE1_TRIGGER_TIME 15 * 60 * 1000UL
#define STAGE1_SLEEP_CYLCLES 5

#define STAGE2_TRIGGER_TIME 120 * 60 * 1000UL
#define STAGE2_SLEEP_CYLCLES 10

#define internal_to_ms(x) x*16
#define ms_to_internal(x) x/16

#define FREQ_TO_COMPARE_VAL(x) (F_CPU/(2*x)-1)

#define SIGNAL_ACTIVE_LOW

#define BEEP_DELAY_TIME ms_to_internal(60*1000UL)

enum state {
	POWER_OFF,
	CONNECTED,
	DISCONNECTED,
};

// We are starting as disconnected
volatile uint8_t state = DISCONNECTED;
// flag to stop sleep cycle. used to avoid long delays, when recovering the quad
volatile bool stop_sleep = false;
bool turn_off_signal_received = false;

// XXX: this costs so much flash -.-
// disconnect variables
// since we only count in 16ms increments this this is saved as 16ms.
// overflows in 50 days. so plenty
uint32_t total_time = 0;

#define USE_RUNTIME_FREQ_SET

#ifdef USE_RUNTIME_FREQ_SET
uint8_t base_freq = 21;
#else
#define base_freq 21
#endif


// Alternating on/off times
// 2^(i+1) = cylces
// time = cycles/2^(10) * 16
// or use WDTO_ defines
// -1 = end of cycle

typedef struct pattern_s{
	uint8_t on_time;
	int8_t freq_offset;
	uint8_t off_time;
	uint8_t repeat;
} pattern_t;

// max vol = 21
pattern_t beep_pattern[BEEP_PATTERN_NUM][BEEP_PATTERN_SIZE] = {{{WDTO_250MS, 0, WDTO_250MS, 4}, {WDTO_250MS, 0, WDTO_250MS, 0}},
																{{WDTO_1S, 0, WDTO_1S, 4}, {WDTO_250MS, 0, WDTO_250MS, 2}}};
uint8_t current_pattern = 0;

// saves space -.-
STATIC_TEST inline uint32_t convert_wdto_to_16ms(uint8_t wdto_time){
	// 2^x = 16ms
	return (1UL << (wdto_time));
}

STATIC_TEST inline void disable_pin_int(){
	GIMSK &= ~(1 << PCIE);
}

STATIC_TEST bool is_turn_off_condition_set(){
	return (total_time > ms_to_internal(2000) && total_time < ms_to_internal(8000));
}

STATIC_TEST inline void enable_pin_int(){
	GIMSK |= (1 << PCIE);
}

STATIC_TEST bool is_beeper_input_set(){
#ifdef SIGNAL_ACTIVE_LOW
	return !(PINB & (1 << BEEPER_IN_PIN));
#else
	return PINB & (1 << BEEPER_IN_PIN);
#endif
}

STATIC_TEST inline bool is_button_pressed(){
	// button is active low
	return !(PINB & (1 << BUTTON_PIN));
}

STATIC_TEST inline void set_watchdog(uint8_t time){
	wdt_reset();
	WDTCR = (1 << WDCE) | (1 << WDTIE) | (time & 0x08 ? (1 << WDP3) : 0x00) | (time & 0x07);
}

STATIC_TEST inline void led_enable(){
	LED_PORT |= (1 << LED_PIN);
}

STATIC_TEST inline void led_disable() {
	LED_PORT &= ~(1 << LED_PIN);
}

STATIC_TEST void sleep(uint8_t time){
	// TODO: disable stuff?

	//enable wdt
	set_watchdog(time);
	// Entering sleep (this sets and clears the flag)
	sleep_mode();
	total_time += convert_wdto_to_16ms(time);
	// TODO: do we need this?
	//wdt_disable();
}

STATIC_TEST bool is_button_long_pressed(){
	if(is_button_pressed()){
		led_enable();
		disable_pin_int();
		sleep(WDTO_2S);
		enable_pin_int();
		led_disable();
	}
	return is_button_pressed();
}

// never tested
#ifdef ENABLE_PATTERN_EDIT
STATIC_TEST inline void record_pattern(uint8_t num){
	bool initial_state = is_beeper_input_set();
	for(int i = 0; i < BEEP_PATTERN_SIZE; ++i){
		// sleep for all possible times until the state changes
		for(uint8_t j = 0; j < WDTO_8S+1; ++j){
			sleep(j);
			// end sequence if button was pressed
			if(is_button_pressed()){
				//eeprom_update_byte((uint8_t*)&beep_pattern[num][i],WDTO_8S + 1);
				beep_pattern[num][i] = WDTO_8S + 1;
				break;
			}
			else if(is_beeper_input_set() != initial_state){
				// state changed or max time, save
				//eeprom_update_byte((uint8_t*)&beep_pattern[num][i],j);
				beep_pattern[num][i] = j;
				break;
			}
		}
	}
}
#endif

STATIC_TEST void sleep_cycles(uint8_t time, uint8_t cycles){
	stop_sleep = false;
	while(cycles && !stop_sleep){
		sleep(time);
		--cycles;
	}
}

STATIC_TEST inline bool is_connected(){
	return PINB & (1 << SENSE_PIN);
}

// Splitting them saves a few bytes
STATIC_TEST void beep_enable(uint8_t freq){
	// set sleep mode to idle to enable pwm
	set_sleep_mode(SLEEP_MODE_IDLE);
	TCNT0 = 254;
	//BEEPER_OUT_PORT |= (1 << BEEPER_OUT_PIN);
	OCR0A = freq;
	// non-inverting mode and ctc
	TCCR0A |= (1 << COM0A0);
}

STATIC_TEST inline void beep_disable() {
	TCCR0A &= ~(1 << COM0A0);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	//OCR0A = 0;
	//BEEPER_OUT_PORT &= ~(1 << BEEPER_OUT_PIN);
	//TCNT0 = 0;
}

/**
 * "Totally" power off the µC. Can only be waked by button press or beep line. Used when user requests power off manually
 */
STATIC_TEST inline void power_off() {
	// Play confirm pattern
	beep_enable(15);
	sleep(WDTO_120MS);
	beep_disable();
	sleep(WDTO_120MS);
	beep_enable(15);
	sleep(WDTO_120MS);

	// disable beeper
	beep_disable();
	led_disable();
	// disable watchdog
	wdt_disable();
	// enter sleep
	// should be around 0.5µA. So a fet to power itself off would be useless
	// but consumes more space
	sleep_mode();
}

STATIC_TEST inline void on_connect(){
	total_time = 0; // reset time on connect event
}

STATIC_TEST void connected_action(){
	static uint8_t signal_time[4] = {0,0,0,0};
	static bool last_beep_status = false;

	// just beep if input signal
	if(is_beeper_input_set()) {
		beep_enable(base_freq);
	}
	else {
		beep_disable();
	}

	if(is_turn_off_condition_set() || is_beeper_input_set() || turn_off_signal_received) {
		led_enable();
	}
	else {
		led_disable();
	}

	// shift on overflow or signal change
	if(last_beep_status != is_beeper_input_set() || signal_time[0] == 254){
		signal_time[3] = signal_time[2];
		signal_time[2] = signal_time[1];
		signal_time[1] = signal_time[0];
		signal_time[0] = 0;
		last_beep_status = is_beeper_input_set();
	}

	// Turn off if betaflight sends continous beeps 100ms on 100ms off
	turn_off_signal_received = signal_time[3] > ms_to_internal(70) && signal_time[3] < ms_to_internal(130) &&
			signal_time[2] > ms_to_internal(70) && signal_time[2] < ms_to_internal(130) &&
			signal_time[1] > ms_to_internal(70) && signal_time[1] < ms_to_internal(130);
			//&&	last_beep_status;

#ifdef USE_RUNTIME_FREQ_SET
	if(is_button_pressed()){
		if(is_button_long_pressed()){
			--base_freq;
		} else {
			++base_freq;
		}
	}
#endif

	// sleep a little bit to keep the timer running. avoids using another timer
	sleep(WDTO_15MS);

#ifdef ENABLE_PATTERN_EDIT
	static uint8_t edit_num = 0;
	// this is the number of loops 1=16ms. so overflows after about 4 sec
	static uint8_t edit_time = 0;
	// TODO: use loop counter to detect button press length
	if(is_button_pressed()){
		edit_time = 0;
		++edit_num;
		if(edit_num > BEEP_PATTERN_NUM){
			edit_num = 1;
		}
		disable_pin_int();
		// Beep edit_num times to confirm position
		for(int i = 1; i < edit_num+1; ++i){
			beep_enable();
			sleep(WDTO_120MS);
			beep_disable();
			sleep(WDTO_120MS);
		}
		// TODO: determine sleep time
		sleep(WDTO_500MS);
		enable_pin_int();
	}
	// button was pressed to enable edit and first signal is there
	if(edit_num != 0){
		if(is_beeper_input_set()){
			record_pattern(edit_num);
			goto set_pattern;
		}
		// Timeout in 254*15ms = 3810ms
		if(edit_time == 254){
			set_pattern:
			current_pattern = edit_num;
			edit_num = 0;
		}
	}
	++edit_time;
#endif
	++signal_time[0];
}

STATIC_TEST inline void on_disconnect(){
	// disable beeper in case it was beeping
	beep_disable();
	led_disable();

	if( is_turn_off_condition_set() || turn_off_signal_received){
		// power off, if bepper is set, while pulling the power <- not a good idea
		// or total power on time is less than 8 seconds (this might impose problems, when a battery is ejected)
		// or power on time is > 2sec and <8 sec
		state = POWER_OFF;
	}
	// power down all unneeded features?

	// reset time on disconnect, as we don't need a separate variable
	total_time = 0;
}

STATIC_TEST inline void disconnected_action(){
	// wait until BEEP_DELAY_TIME to start beeping
	if(total_time > BEEP_DELAY_TIME) {
		led_disable();
		for(uint8_t i = 0; i < BEEP_PATTERN_SIZE; ++i){
			//uint8_t pattern = eeprom_read_byte((const uint8_t*)&beep_pattern[current_pattern][i]);
			pattern_t pattern = beep_pattern[current_pattern][i];
			// repeat pattern j times
			for(uint8_t j = 0; j < pattern.repeat; ++j){
				// Beep enable part
				beep_enable(base_freq + pattern.freq_offset);
				// sleep instead of simple delay
				sleep(pattern.on_time);

				// off time part
				beep_disable();
				sleep(pattern.off_time);
				// in case we are pressing the button while beeping
				if(stop_sleep){
					stop_sleep = false;
					goto disconnected_check_button;
				}
			}
		}

		// cycle through all patterns
		if(++current_pattern >= BEEP_PATTERN_NUM){
			current_pattern = 0;
		}
	} else {
		// Very low noise beep to indicate armed status
		beep_enable(5);
		sleep(WDTO_15MS);
		beep_disable();
	}

	// increase delay when beeping for a longer time
	if(total_time < STAGE1_TRIGGER_TIME){
		// STAGE 0
		sleep_cycles(WDTO_2S, 1);
	}
	else if(total_time < STAGE2_TRIGGER_TIME){
		// stage 1
		sleep_cycles(WDTO_8S, STAGE1_SLEEP_CYLCLES);
	}
	else {
		// stage 2
		sleep_cycles(WDTO_8S, STAGE2_SLEEP_CYLCLES);
	}

disconnected_check_button:
	if(is_button_long_pressed()){
		// Button still pressed after 2 sec. so shutdown
		// if button is not pressed after 4 sec we are pretty sure it is a human interaction
		sleep(WDTO_2S);
		if(!is_button_pressed()){
			state = POWER_OFF;
		}
	}

}

/**
 * Possible state transitions
 *
 * power off -> connected <->  disconnected -> power off
 */
STATIC_TEST inline void check_status(){
	if(is_connected()){
		// triggered on power on or disconnected -> connected
		if(state != CONNECTED){
			state = CONNECTED;
			on_connect();
		}
	}
	else {
		if(state == CONNECTED){
			state = DISCONNECTED;
			on_disconnect();
		}
	}

	if(state == CONNECTED){
		connected_action();
	}
	else if(state == DISCONNECTED){
		disconnected_action();
	}
	else {
		power_off();
	}

}

ISR(PCINT0_vect){
	// Wakeup from power down
	// no need to do anything
	// Set stop sleep flag for sleep_cycle
	stop_sleep = true;
}

ISR(WDT_vect){
	//TODO: do we need to disable the watchdog?
	MCUSR &= ~(1 << WDRF);
	WDTCR |= (1 << WDTIE);
}

int main(){
	DDRB = 0; // all input
	PORTB = 0xFF; // enable all pullups (saves power)
	
	// enable LED out
	LED_DDR |= (1 << LED_PIN);

	// enable beeper output
	BEEPER_OUT_DDR |= (1 << BEEPER_OUT_PIN);
	BEEPER_OUT_PORT &= ~(1 << BEEPER_OUT_PIN);

	TCCR0A = (1 << WGM01);

	// no pwm prescaler
	TCCR0B = (1 << CS00);

	// pulldown not installed and signal inverted
#ifndef SIGNAL_ACTIVE_LOW
	BEEPER_IN_PORT &= ~(1 << BEEPER_IN_PIN);
#endif

	SENSE_PORT &= ~(1 << SENSE_PIN);

	// Enable interrupt on any change on int0 and pcint
	enable_pin_int();
	// enable int on beeper_in, button and sense
	PCMSK |= (1 << PCINT1) | (1 << PCINT4);

	// enable int0. not needed for now, since we are using pcint
	////MCUCR |= (1 << ISC01) | (1 << ISC00);

	WDTCR = (1<<WDCE);
	//Enable watchdog for int only
	WDTCR = (1<<WDTIE);
	
	// set sleep to power down
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	// Disable adc
	ADCSRA &= ~(1 << ADEN);
	ACSR |= (1 << ACD);
	// disable timer0 and adc (doesn't do anything on attiny13)
	power_all_disable();

	sei();

	while(42){
		check_status();
	}
}
