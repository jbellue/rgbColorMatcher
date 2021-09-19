#ifndef DEBOUNCE_H
#define DEBOUNCE_H

// heavily inspired from https://www.avrfreaks.net/sites/default/files/forum_attachments/debounce.pdf
#include <stdint.h>

static inline uint8_t debounce(const uint8_t pin) {
    static uint8_t count = 0;
    static uint8_t previous_state = 0;

    const uint8_t current_state = (~PINB & (1 << pin)) != 0;
    if (current_state != previous_state) {
        if (++count >= 3) { // No bounce in three checks, update state
            previous_state = current_state;
            count = 0;
            if (current_state != 0) {
                return 1;
            }
        }
    } else {
        // Reset counter
        count = 0;
    }
    return 0;
}

#endif
