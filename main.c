#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdbool.h>

#define CLK_DIV 125 // PWM clock divider
#define TOP 999 // PWM counter top value

// Stepper motor control pins
#define IN1 2
#define IN2 3
#define IN3 6
#define IN4 13
#define INS_SIZE 4

// Optical sensor input with pull-up
#define SENSOR 28

void ini_coil_pins(const uint *ins); // Initialize motor output pins
void ini_sensor(); // Initialize optical sensor input
int calibrate(const uint *ins, const int half_step[8][4], int max, int revolution_steps[3]); // Measure steps per revolution using the optical sensor
void step_motor(const uint *ins, int step, const int half_step[8][4]); // Perform one half-step
int get_avg(const int revolution_steps[3]); // Calculate average of three edge intervals

int main() {
    const uint coil_pins[] = {IN1, IN2, IN3, IN4};
    const int safe_max = 20480; // Safety limit: 5 * 4096
    int revolution_steps[3]; // Array to store step counts between four consecutive edges
    // Half-step sequence for unipolar stepper motor
    const int half_step[8][4] = {
        {1, 0, 0, 0}, // Step 1: A
        {1, 1, 0, 0}, // Step 2: A + B
        {0, 1, 0, 0}, // Step 3: B
        {0, 1, 1, 0}, // Step 4: B + C
        {0, 0, 1, 0}, // Step 5: C
        {0, 0, 1, 1}, // Step 6: C + D
        {0, 0, 0, 1}, // Step 7: D
        {1, 0, 0, 1}  // Step 8: D + A
    };

    // Initialize chosen serial port
    stdio_init_all();
    // Initialize motor control pins
    ini_coil_pins(coil_pins);
    // Initialize optical sensor input (with internal pull-up)
    ini_sensor();

    // Run calibration to measure average steps per revolution
    const int result = calibrate(coil_pins, half_step, safe_max, revolution_steps);
    if (result > 0) {
        printf("Calibration completed.\r\n");
        printf("Average: %d\r\n", result);
    }
    else
        printf("Calibration failed.\n\r");
    return 0;
}

void ini_coil_pins(const uint *ins) {
    for (int i = 0; i < INS_SIZE; i++) {
        gpio_init(ins[i]);
        gpio_set_dir(ins[i], GPIO_OUT);
        // Ensure all coils are off at startup
        gpio_put(ins[i], 0);
    }
}

void ini_sensor() {
    gpio_init(SENSOR);
    gpio_set_dir(SENSOR, GPIO_IN);
    // Internal pull-up: SENSOR reads HIGH (1) when not blocked, LOW (0) when blocked
    gpio_pull_up(SENSOR);
}

int calibrate(const uint *ins, const int half_step[8][4], const int max, int revolution_steps[3]) {
    int count = 0; // Number of detected falling edges
    int step = 0; // Global step counter
    int edge_step = 0; // Steps between consecutive edges (starts after first edge)
    bool first_edge_found = false;
    bool continue_loop = true;
    bool prev_state = gpio_get(SENSOR); // true = no obstacle, false = obstacle

    do {
        // Advance the motor by one half-step
        step_motor(ins, step, half_step);
        sleep_ms(3);
        step++;

        // Start counting steps between edges after the first edge has been found
        if (first_edge_found)
            edge_step++;

        const bool sensor_state = gpio_get(SENSOR);
        // Detect falling edge: HIGH → LOW transition (no obstacle → obstacle)
        if (prev_state && !sensor_state) {
            if (!first_edge_found) {
                printf("First low edge found\r\n");
                first_edge_found = true;
            }
            else {
                // Store steps between two consecutive falling edges
                revolution_steps[count-1] = edge_step;
                printf("%d. round steps: %d\r\n", count, edge_step);
                edge_step = 0;
            }
            count++;
        }
        // Stop after four edges (three valid intervals) or when safety limit is reached
        if (count >= 4 || step > max)
            continue_loop = false;
        prev_state = sensor_state;

    } while (continue_loop);

    // At least four falling edges are required to calculate three intervals
    if (count >= 4) {
        const int avg = get_avg(revolution_steps);
        return avg;
    }
    return 0; // Calibration failed
}

void step_motor(const uint *ins, const int step, const int half_step[8][4]) {
    // Determines which step phase (0–7) the motor is currently in
    // Bitwise AND preserves only the three lowest bits.
    // This means that phase is always between 0 and 7.
    // As step increases, phase cycles through 0–7 → 0–7 → 0–7…
    // Each phase corresponds to one row in half_step[8][4]
    // defining which coils (IN1–IN4) are energized at this moment.
    const int phase = step & 7;
    for (int i = 0; i < INS_SIZE; i++) {
        gpio_put(ins[i], half_step[phase][i]);
    }
}

int get_avg(const int revolution_steps[3]) {
    int sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += revolution_steps[i];
    }
    const int avg = sum / 3;
    return avg;
}