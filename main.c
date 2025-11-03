#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdbool.h>

#define CLK_DIV 125 // PWM clock divider
#define TOP 999 // PWM counter top value

#define SW_R 7 // right button - decreases brightness
#define SW_M 8 // middle button - light switch
#define SW_L 9 // left button - increases brightness
#define BUTTONS_SIZE 3 // how many buttons

#define LED_R 22 // right LED
#define LED_M 21 // middle LED
#define LED_L 20 // left LED
#define LEDS_SIZE 3 // how many LEDs

#define BR_RATE 50 // step size for brightness changes
#define BR_MID (TOP / 2) // 50% brightness level
#define MAX_BR (TOP + 1)

void ini_buttons(const uint *buttons); // Initialize buttons
void ini_leds(const uint *leds); // Initialize LED pins
bool light_switch(const uint *leds, uint brightness, bool on); // Turn lights on/off
void set_brightness(const uint *leds, uint brightness); // Increase/decrease lighting
uint clamp(int br); // returns value between 0 and TOP

int main() {
    const uint buttons[] = {SW_R, SW_M, SW_L};
    const uint leds[] = {LED_R, LED_M, LED_L};
    uint brightness = BR_MID; // LEDs brightness value

    // Initialize chosen serial port
    stdio_init_all();
    // Initialize buttons
    ini_buttons(buttons);
    // Initialize LED pins
    ini_leds(leds);

    bool lightsOn = false;
    bool SW1_unpressed = true; // SW1 has pull-up, so unpressed = true

    while (true) {
        // SW1 state: true = not pressed, false = pressed
        const bool sw1_state = gpio_get(SW_M);

        // Detect button press (transition from released to pressed)
        if (SW1_unpressed && !sw1_state) {
            // Turn lights on
            if (!lightsOn) {
                lightsOn = light_switch(leds, brightness, true);
            }
            else {
                // If LEDs are on and brightness is 0%, restore to 50%
                if (brightness <= 0) {
                    brightness = BR_MID;
                    set_brightness(leds, BR_MID);
                }
                // Otherwise turn lights off
                else {
                    lightsOn = light_switch(leds, 0, false);
                }
            }
        }

        if (lightsOn) {
            // Increase lighting
            if (!gpio_get(SW_L)) {
                brightness = clamp((int)brightness + BR_RATE);
                set_brightness(leds, brightness);
            }
            // Decrease lighting
            if (!gpio_get(SW_R)) {
                brightness = clamp((int)brightness - BR_RATE);
                set_brightness(leds, brightness);
            }
        }
        // 100 ms polling delay:
        //  - filters out unintended rapid toggles
        //  - reduces CPU usage
        sleep_ms(100);
        // Store current SW1 state for next loop iteration
        // used for edge detection and double-press prevention
        SW1_unpressed = sw1_state;
    }
}

void ini_buttons(const uint *buttons) {
    for (int i = 0; i < BUTTONS_SIZE; i++) {
        gpio_init(buttons[i]); // Initialize GPIO pin
        gpio_set_dir(buttons[i], GPIO_IN); // Set as input
        gpio_pull_up(buttons[i]); // Enable internal pull-up resistor (button reads high = true when not pressed)
    }
}

void ini_leds(const uint *leds) {
    // Track which PWM slices (0-7) have been initialized
    bool slice_ini[8] = {false};

    // Get default PWM configuration
    pwm_config config = pwm_get_default_config();
    // Set clock divider
    pwm_config_set_clkdiv_int(&config, CLK_DIV);
    // Set wrap (TOP)
    pwm_config_set_wrap(&config, TOP);

    for (int i = 0; i < LEDS_SIZE; i++) {
        // Get slice and channel for your GPIO pin
        const uint slice = pwm_gpio_to_slice_num(leds[i]);
        const uint chan = pwm_gpio_to_channel(leds[i]);

        // Disable PWM while configuring
        pwm_set_enabled(slice, false);

        // Initialize each slice once (sets divider and TOP for both A/B)
        if (!slice_ini[slice]) {
            pwm_init(slice, &config, false); // Do not start yet
            slice_ini[slice] = true;
        }

        // Set compare value (CC) to define duty cycle
        pwm_set_chan_level(slice, chan, 0);
        // Select PWM model for your pin
        gpio_set_function(leds[i], GPIO_FUNC_PWM);
        // Start PWM
        pwm_set_enabled(slice, true);
    }
}

bool light_switch(const uint *leds, const uint brightness, const bool on) {
    if (on) {
        set_brightness(leds, brightness);
        return true;
    }
    set_brightness(leds, 0);
    return false;
}

void set_brightness(const uint *leds, const uint brightness) {
    // Set PWM duty cycle for each LED to match the desired brightness
    for (int i = 0; i < LEDS_SIZE; i++) {
        const uint slice = pwm_gpio_to_slice_num(leds[i]);  // Get PWM slice for LED pin
        const uint chan  = pwm_gpio_to_channel(leds[i]); // Get PWM channel (A/B)
        pwm_set_chan_level(slice, chan, brightness); // Update duty cycle value
    }
}

uint clamp(const int br) {
    // Limit brightness value to valid PWM range [0, TOP]
    if (br < 0) return 0; // Lower bound
    if (br > MAX_BR) return MAX_BR; // Upper bound
    return br; // Within range
}