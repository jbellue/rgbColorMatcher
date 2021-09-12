#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "strip_handler.h"


//define the processor speed (if it's not already been defined by the compiler)
#ifndef F_CPU
	#define F_CPU 8000000UL
#endif

#define RED_POT   0b00000000
#define GREEN_POT 0b00000001
#define BLUE_POT  0b00000010
#define DIFFICULTY_POT 0b00000011

#define LED_COUNT 2
rgb_color leds[LED_COUNT];


#define MIN_RGB_LEVEL 50

/*
 * four potentionmeters:
 *     ADC0: R
 *     ADC1: G
 *     ADC2: B
 *     ADC3: difficulty (needed accuracy)
 *
 * a strip of two neopixels
 *     PWM
 *
 *
 *              ┌ ─ ─ ─ ┐
 *     ADC0  1  |°      |  8  VCC
 *     ADC3  2  |       |  7  ADC1
 *     ADC2  3  |       |  6  PB1 (as software reset?)
 *     GND   4  |       |  5  PWM
 *              └ ─ ─ ─ ┘
 */

// extern "C" void output_grb(uint8_t * ptr, uint16_t count);
// struct cRGB leds[2];
// uint8_t leds[6];    // grb-grb

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
void initRand()
{
	uint8_t current_seed = eeprom_read_word(0);
	srand(++current_seed);              // increment and use value as seed
	eeprom_write_word(0, current_seed); //store the new seed for next time
}

/*
 * We don't care about 10-bits precision, so only use 8 bits. That'll also
 * make reading the values simpler.
 */
void initADC() {
    ADMUX = (1 << ADLAR) |  // left shift result for 8-bit values
            (1 << REFS0);   // use VCC as reference

    ADCSRA =(1 << ADEN)  |  // enable ADC
            (1 << ADPS1) |  // set ADC clock to 125khz (division factor 8)
            (1 << ADPS0);   // set ADC clock to 125khz (division factor 8)
}

/*
 * Use timer0 to update the user's LED.
 */
void initTimer() {
    // set up Timer 0
    TCCR0A = (1 << WGM01) | // CTC mode
            (1 << COM0A0);  // Toggle OC0A on Compare Match
    TCCR0B = (1 << CS00) |
            (1 << CS02);    // 1024 prescaler
    OCR0A = 128;            // Should give 8000000 / 1024 / 128 = 61Hz

    sei();                  // enable interrupts
}

uint8_t readADC(uint8_t channel) {
    ADMUX &= 11111000;              // clear the previous channel selected
    ADMUX |= channel;

    while(!(ADCSRA & (1 << ADIF))); // wait for conversion to finish
    ADCSRA |= (1 << ADSC);          // start the conversion
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

void compareLedValues(uint8_t difficulty) {
    if (abs(leds[0].r - leds[1].r) > difficulty ||
        abs(leds[0].g - leds[1].g) > difficulty ||
        abs(leds[0].b - leds[1].b) > difficulty) {
        // loose, better luck next loop
        return;
    }

    showVictory();
    restartFromScratch();
}

int main(void) {
    initADC();
    initTimer();
    initRand();
    initStartLed();

    while(1) {
        leds[1].r = readADC(GREEN_POT);
        leds[1].g = readADC(RED_POT);
        leds[1].b = readADC(BLUE_POT);
        _delay_ms(10);

        compareLedValues(readADC(DIFFICULTY_POT));
    }
}

ISR(TIMER0_COMPA_vect) {
    led_strip_write(leds, LED_COUNT);
}
