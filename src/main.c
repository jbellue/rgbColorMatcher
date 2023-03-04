/*
 * Based on https://github.com/paulboardman/avr/tree/master/rgb_color_matcher
 * Four potentionmeters:
 *      ADC0: Difficulty - 10kΩ, with detents
 *      ADC1: G          - 10kΩ
 *      ADC2: B          - 10kΩ
 *      ADC3: R          - 10kΩ
 *
 * a strip of two WS2812b:
 *      PB0
 *
 * a push button:
 *      PB1-GND: software reset
 *                              ┌───────┐
 *   Diffficulty pot - ADC0  1 ─┤°      ├─ 8  VCC
 *           Red pot - ADC3  2 ─┤       ├─ 7  ADC1 - Green pot
 *          Blue pot - ADC2  3 ─┤       ├─ 6  PB1 - Software reset
 *                      GND  4 ─┤       ├─ 5  PB0 - WS2812b
 *                              └───────┘
 *
 * TODO:
 *  - Tweak code for RSTDSBL
 *  - Tweak difficulty
 */

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "strip_handler.h"
#include "debounce.h"

#define DIFFICULTY_POT_MUX  0b00000000
#define GREEN_POT_MUX       0b00000001
#define BLUE_POT_MUX        0b00000010
#define RED_POT_MUX         0b00000011

#define MIN_DIFFICULTY  85
#define MAX_DIFFICULTY  5
#define LED_COUNT 2
rgb_color leds[LED_COUNT];

#define MIN_RGB_LEVEL 50

static volatile uint8_t updateLEDstripFlag = 0;
static volatile uint8_t restartFromScratchFlag = 0;

/*
 * Initialise both the target led (and ignore values that are too dark)
 * and the user's led to black
 */
void initStartLed() {
    uint8_t totalLight = 0;
    while (totalLight < MIN_RGB_LEVEL) {
        // shift 7 bits to the right to divide by 128 and get a value 0-255
        leds[0].r = (uint8_t)(rand() >> 7);
        leds[0].g = (uint8_t)(rand() >> 7);
        leds[0].b = (uint8_t)(rand() >> 7);
        totalLight = leds[0].r + leds[0].g + leds[0].b;
    }
    leds[1] = (rgb_color) {.r=0, .g=0, .b=0};    // black
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

    for(uint8_t i = 0; i < 100 ; ++i) {
        float frequency = 0.3 * i;
        uint8_t r = (sin(frequency    ) + 1) * 128;
        uint8_t g = (sin(frequency + 2) + 1) * 128;
        uint8_t b = (sin(frequency + 4) + 1) * 128;
        leds[0] = (rgb_color) {.r=r, .g=g, .b=b};
        leds[1] = (rgb_color) {.r=r, .g=g, .b=b};
        led_strip_write(leds, LED_COUNT);

        _delay_ms(20);
    }
    sei();  // restore interrupts
}

void restartFromScratch() {
    initStartLed();
}

void compareLedValues(const uint8_t difficulty) {
    if (abs(leds[0].r - leds[1].r) > difficulty ||
        abs(leds[0].g - leds[1].g) > difficulty ||
        abs(leds[0].b - leds[1].b) > difficulty) {
        // lose, better luck next loop
        return;
    }

    showVictory();
    restartFromScratch();
}

void initButtonInput() {
    DDRB &= ~(1 << PB1);    // Set PB1 as a digital input pin
    PORTB |= (1 << PB1);    // Enable its pull-up resistor
}

uint8_t map(const uint8_t x, const uint8_t in_min, const uint8_t in_max, const uint8_t out_min, const uint8_t out_max) {
    if (x <= out_min) return out_min;
    if (x >= out_max) return out_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t setDifficultyFromADCValue(const uint8_t ADCvalue) {
    return map(ADCvalue, 120, 255, MAX_DIFFICULTY, MIN_DIFFICULTY);
}

int main(void) {
    initADC();
    initTimer();
    initRand();
    initStartLed();
    initButtonInput();

    while(1) {
        leds[1].r = readADC(RED_POT_MUX);
        leds[1].g = readADC(GREEN_POT_MUX);
        leds[1].b = readADC(BLUE_POT_MUX);

        compareLedValues(setDifficultyFromADCValue(readADC(DIFFICULTY_POT_MUX)));

        if (updateLEDstripFlag) {
            led_strip_write(leds, LED_COUNT);
            updateLEDstripFlag = 0;
        }
        if(restartFromScratchFlag) {
            restartFromScratch();
            restartFromScratchFlag = 0;
        }
    }

    return 0;
}

ISR(TIMER0_COMPA_vect) {
    updateLEDstripFlag = 1;
    if (!(PINB & (1 << PB1))) {
        restartFromScratchFlag = 1;
    }
}
