#ifndef STRIP_HANDLER_H
#define STRIP_HANDLER_H

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define LED_STRIP_PORT PORTB
#define LED_STRIP_DDR  DDRB
#define LED_STRIP_PIN  0

typedef struct rgb_color {
    uint8_t r, g, b;
} rgb_color;

void led_strip_write(rgb_color * colors, uint16_t count) __attribute__((noinline));

#endif