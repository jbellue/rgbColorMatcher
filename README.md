# rgbColorMatcher

A game based on [Paul Boardman's rgb_color_matcher](https://github.com/paulboardman/avr/tree/master/rgb_color_matcher)

##  Goal

Match a randomly selected color using three potentiometers for the red, green and blue values.

You can tweak the difficulty with a fourth potentiometer.

## Hardware
- Four potentionmeters:
  - ADC0: Difficulty - 10kΩ, with detents
  - ADC1: G          - 10kΩ
  - ADC2: B          - 10kΩ
  - ADC3: R          - 10kΩ
- a strip of two WS2812b (on PB0), for the goal LED and the user's
- a push button (PB1-GND), for software reset
 
 ```
                            ┌───────┐
 Diffficulty pot - ADC0  1 ─┤°      ├─ 8  VCC
         Red pot - ADC3  2 ─┤       ├─ 7  ADC1 - Green pot
        Blue pot - ADC2  3 ─┤       ├─ 6  PB1 -  Software reset
                    GND  4 ─┤       ├─ 5  PB0 -  WS2812b
                            └───────┘
```

 ## TODO

- Tweak code for RSTDSBL
- Tweak difficulty
