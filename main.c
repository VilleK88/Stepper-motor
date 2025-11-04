#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

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

#define INPUT_LENGTH 200

void ini_coil_pins(const uint *coil_pins); // Initialize motor output pins
void ini_sensor(); // Initialize optical sensor input
int calibrate(const uint *coil_pins, const int half_step[8][4], int max, int revolution_steps[3]); // Measure steps per revolution using the optical sensor
void step_motor(const uint *coil_pins, int step, const int half_step[8][4]); // Perform one half-step
int get_avg(const int revolution_steps[3]); // Calculate average of three edge intervals
void run_motor(const uint *coil_pins, const int half_step[8][4], int count);
char *handle_input();
bool get_input(char *user_input);
void trim_line(char *user_input);
bool compare(const char *user_input, const char *cmp_value);
bool check_if_nums(const char *string);
int get_nums_from_a_string(const char *string);

int main() {
    const uint coil_pins[] = {IN1, IN2, IN3, IN4};
    const int safe_max = 20480; // Safety limit: 5 * 4096
    int revolution_steps[3] = {0, 0, 0}; // Array to store step counts between four consecutive edges
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

    while (true) {
        const char *user_input = handle_input();

        if (compare(user_input, "calib")) {
            // Run calibration to measure average steps per revolution
            const int result = calibrate(coil_pins, half_step, safe_max, revolution_steps);
            if (result > 0) {
                printf("Calibration completed.\r\n");
                printf("Average: %d\r\n", result);
            }
            else
                printf("Calibration failed.\n\r");
        }
        else if (compare(user_input, "status")) {
            if (revolution_steps[0] == 0) {
                printf("Not available\r\n");
            }
            else {
                for (int i = 0; i < 3; i++) {
                    printf("%d. revolution: %d\r\n", i+1, revolution_steps[i]);
                }
            }
        }

        char word_out[4];
        memcpy(word_out, user_input, 3);
        word_out[3] = '\0';
        if (compare(word_out, "run")) {
            printf("Run works\r\n");
            if (strlen(user_input) >= 4) {
                if (check_if_nums(user_input + 4)) {
                    const int num_out = get_nums_from_a_string(user_input + 4);
                    printf("Num out: %d\r\n", num_out);
                    run_motor(coil_pins, half_step, num_out);
                }
            }
            else if (strlen(user_input) == 3) {
                if (strlen(user_input) == strlen(word_out))
                    run_motor(coil_pins, half_step, 8);
            }
        }
    }
}

void ini_coil_pins(const uint *coil_pins) {
    for (int i = 0; i < INS_SIZE; i++) {
        gpio_init(coil_pins[i]);
        gpio_set_dir(coil_pins[i], GPIO_OUT);
        // Ensure all coils are off at startup
        gpio_put(coil_pins[i], 0);
    }
}

void ini_sensor() {
    gpio_init(SENSOR);
    gpio_set_dir(SENSOR, GPIO_IN);
    // Internal pull-up: SENSOR reads HIGH (1) when not blocked, LOW (0) when blocked
    gpio_pull_up(SENSOR);
}

int calibrate(const uint *coil_pins, const int half_step[8][4], const int max, int revolution_steps[3]) {
    int count = 0; // Number of detected falling edges
    int step = 0; // Global step counter
    int edge_step = 0; // Steps between consecutive edges (starts after first edge)
    bool first_edge_found = false;
    bool continue_loop = true;
    bool prev_state = gpio_get(SENSOR); // true = no obstacle, false = obstacle

    do {
        // Advance the motor by one half-step
        step_motor(coil_pins, step, half_step);
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

void step_motor(const uint *coil_pins, const int step, const int half_step[8][4]) {
    // Determines which step phase (0–7) the motor is currently in
    // Bitwise AND preserves only the three lowest bits.
    // This means that phase is always between 0 and 7.
    // As step increases, phase cycles through 0–7 → 0–7 → 0–7…
    // Each phase corresponds to one row in half_step[8][4]
    // defining which coils (IN1–IN4) are energized at this moment.
    const int phase = step & 7;
    for (int i = 0; i < INS_SIZE; i++) {
        gpio_put(coil_pins[i], half_step[phase][i]);
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

void run_motor(const uint *coil_pins, const int half_step[8][4], const int count) {
    const int i_count = count * 512;  // 4096 / 8 = 512
    for (int i = 0; i < i_count; i++) {
        step_motor(coil_pins, i, half_step);
        sleep_ms(3);
    }
}

char *handle_input() {
    static char string[INPUT_LENGTH];
    bool stop_loop = false;
    while (!stop_loop) {
        printf("Enter cmd: ");
        fflush(stdout);
        stop_loop = get_input(string);
    }
    return string;
}

bool get_input(char *user_input) {
    if(fgets(user_input, INPUT_LENGTH, stdin)) {
        if (strchr(user_input, '\n') == NULL) {
            int c = 0;
            while ((c = getchar()) != '\n' && c != EOF) {}
            printf("Input too long (max %d characters).\r\n", INPUT_LENGTH-2);
            return false;
        }
        trim_line(user_input);
        if (user_input[0] == '\0') {
            printf("Empty input.\r\n");
            return false;
        }
        return true;
    }
    return false;
}

void trim_line(char *user_input) {
    int len = (int)strlen(user_input);
    while (len > 0 && (user_input[len - 1] == '\n' || user_input[len - 1] == '\r')) {
        user_input[--len] = '\0';
    }
}

bool compare(const char *user_input, const char *cmp_value) {
    if (strcmp(user_input, cmp_value) == 0)
        return true;
    return false;
}

bool check_if_nums(const char *string) {
    const int len = (int)strlen(string);
    for (int i = 0; i < len; i++) {
        if (!isdigit((unsigned char)string[i])) {
            return false;
        }
    }
    return true;
}

int get_nums_from_a_string(const char *string) {
    const int len = (int)strlen(string);
    char num_char[len + 1];
    int j = 0;

    for (int i = 0; i < len; i++) {
        if (isdigit((unsigned)string[i])) {
            num_char[j++] = string[i];
        }
    }

    if (j > 0) {
        num_char[j] = '\0';
        return atoi(num_char);
    }
    return 0;
}