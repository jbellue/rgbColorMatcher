/*
 * Based on https://github.com/paulboardman/avr/tree/master/rgb_color_matcher
 * three potentionmeters:
 *      ADC1: G
 *      ADC2: B
 *      ADC3: R
 *
 * a strip of two WS2812b:
 *      PB0
 *
 * two push buttons:
 *      RST-GND (for hardware reset)
 *      PB1-GND (to select difficulty)
 *                         ┌ ─ ─ ─ ┐
 *   push button - RST  1  |°      |  8  VCC
 *      Red pot - ADC3  2  |       |  7  ADC1 - Green pot
 *     Blue pot - ADC2  3  |       |  6  PB1 - push button
 *                 GND  4  |       |  5  PB0 - WS2812b
 *                         └ ─ ─ ─ ┘
 * 
 * TODO:
 *      - Add a difficulty selection on PB1. It could cycle through like 5 difficulty levels, and show them as a scale from green to red on the LEDs
 *      - Program a better light show on victory
 */

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "strip_handler.h"
extern "C" {
    #include "debounce.h"
}


#define F_CPU 8000000UL

#define GREEN_POT 0b00000001
#define BLUE_POT  0b00000010
#define RED_POT 0b00000011

#define DIFFICULTY 20
#define LED_COUNT 2
rgb_color leds[LED_COUNT];
#define MIN_RGB_LEVEL 50

#define DIFFICULTY_STEP 15
#define DEFAULT_DIFFICULTY 30

uint8_t difficulty = DEFAULT_DIFFICULTY;

/*
 * Initialise both the target led (and ignore values that are too dark)
 * and the user's led to black
 */
void initStartLed() {
    uint8_t totalLight;
    while (totalLight < MIN_RGB_LEVEL) {
        // shift 7 bits to the right to divide by 128 and get a value 0-255
        leds[0].r = (uint8_t)(rand() >> 7);
        leds[0].g = (uint8_t)(rand() >> 7);
        leds[0].b = (uint8_t)(rand() >> 7);
        totalLight = leds[0].r + leds[0].g + leds[0].b;
    }
    leds[1].r = 0;
    leds[1].g = 0;
    leds[1].b = 0;
}

/*
 * Use a variable stored in EEPROM to ensure the random color
 * sequence changes from one game to the next.
 */
void initRand() {
	uint8_t current_seed = eeprom_read_word(0);
	srand(++current_seed);              // increment and use value as seed
	eeprom_write_word(0, current_seed); //store the new seed for next time
}

void initADC() {
    ADMUX =
        (1 << ADLAR) |    // left shift result
        (0 << REFS1) |    // Sets ref. voltage to VCC, bit 1
        (0 << REFS0);     // Sets ref. voltage to VCC, bit 0

    ADCSRA =
        (1 << ADEN)  |    // Enable ADC 
        (1 << ADPS2) |    // set prescaler to 64, bit 2
        (1 << ADPS1) |    // set prescaler to 64, bit 1
        (0 << ADPS0);     // set prescaler to 64, bit 0

    DIDR0 = (1 << ADC2D);     // disable digital input on analog input channel to conserve power 

}

/*
 * Use timer0 to update the user's LED.
 */
void initTimer() {
    cli();                  // stop interrupts
    TCNT0 = 0;              // initialize counter value to 0

    OCR0A = 129;            // Should give 8000000 / 1024 / (129+1) = 60Hz

    TCCR0A = (1 << WGM01);  // set CTC mode

    TCCR0B = (1 << CS00) |
             (1 << CS02);   // 1024 prescaler

    TIMSK |= (1 << OCIE0A); // Output Compare Match A Interrupt Enable

    sei();                  // enable interrupts
}

uint8_t readADC(const uint8_t channel) {
    ADMUX &= 0xF0;                 // clear the previous channel selected
    ADMUX |= channel;

    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete
    return ADCH;
}

void showVictory() {
    cli();  // prevent interrupt from ruining the light show

    for(uint8_t i = 0; i < 5 ; ++i) {
        leds[0].r = 0;
        leds[0].g = 0;
        leds[0].b = 0;
        leds[1].r = 0;
        leds[1].g = 0;
        leds[1].b = 0;
        led_strip_write(leds, LED_COUNT);

        _delay_ms(200);

        leds[0].r = 200;
        leds[0].g = 200;
        leds[0].b = 200;
        leds[1].r = 200;
        leds[1].g = 200;
        leds[1].b = 200;
        led_strip_write(leds, LED_COUNT);

        _delay_ms(200);
    }
    sei();  // restore interrupts
}

void restartFromScratch() {
    initStartLed();
}

void compareLedValues() {
    if (abs(leds[0].r - leds[1].r) > difficulty ||
        abs(leds[0].g - leds[1].g) > difficulty ||
        abs(leds[0].b - leds[1].b) > difficulty) {
        // loose, better luck next loop
        return;
    }

    showVictory();
    restartFromScratch();
}

void initButtonInput() {
    DDRB &= ~(1 << PB1);    // Set PB1 as a digital input pin
    PORTB |= (1 << PB1);    // Enable its pull-up resistor
}

int main(void) {
    initADC();
    initTimer();
    initRand();
    initStartLed();
    initButtonInput();

    while(1) {
        leds[1].r = readADC(RED_POT);
        leds[1].g = readADC(GREEN_POT);
        leds[1].b = readADC(BLUE_POT);
        _delay_ms(10);

        compareLedValues();
    }

    return 0;
}

ISR(TIMER0_COMPA_vect) {
    led_strip_write(leds, LED_COUNT);
    if (debounce(PB1)) {
        difficulty += DIFFICULTY_STEP;
        if (difficulty > 65) {
            difficulty = 5;
        }
    }
}
