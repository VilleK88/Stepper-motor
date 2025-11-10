#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

// Stepper motor control pins
#define IN1 2
#define IN2 3
#define IN3 6
#define IN4 13
#define INS_SIZE 4

#define SENSOR 28 // Optical sensor input with pull-up

#define INPUT_LENGTH 200 // Maximum input line length

void ini_coil_pins(const uint *coil_pins); // Initialize motor coil output pins as outputs
void ini_sensor(); // Initialize optical sensor input with internal pull-up
int calibrate(const uint *coil_pins, const int half_step[8][4], int max, int revolution_steps[3]); // Measure steps per revolution using the optical sensor
void step_motor(const uint *coil_pins, int step, const int half_step[8][4]); // Perform one half-step of the stepper motor
int get_avg(const int revolution_steps[3]); // Calculate the average of three revolution step counts
void run_motor(const uint *coil_pins, const int half_step[8][4], int count, int steps_per_rev); // Run the motor for N * (1/8) revolutions using the calibrated steps per revolution
char *handle_input(); // Read a single non-empty command from user input
bool get_input(char *user_input); // Read a line from stdin, validate it, and remove newline characters
void trim_line(char *user_input); // Remove '\n' and '\r' characters from the end of a string
bool check_if_nums(const char *string); // Return true if the string contains only digits (0–9)
int get_nums_from_a_string(const char *string); // Extract digits from a string, form an integer (rejects leading zeros)
bool validate_run_input(const char *user_input); // Validate that "run" command has a proper numeric argument ("run N")

int main() {
    // Stepper motor coil pins
    const uint coil_pins[] = {IN1, IN2, IN3, IN4};
    // Safety limit to prevent infinite rotation during calibration
    const int safe_max = 20480; // Safety limit: 5 * 4096 steps
    int steps_per_rev = 4096; // Default steps per revolution before calibration
    int avg = 0;
    int revolution_steps[3] = {0, 0, 0}; // Array to store step counts between four consecutive edges

    // Half-step sequence for unipolar stepper motor
    // Each row defines which coils (IN1–IN4) are energized for each step
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
    // Initialize stepper motor pins
    ini_coil_pins(coil_pins);
    // Initialize optical sensor input (with internal pull-up)
    ini_sensor();

    while (true) {
        // Read one user command (status / calib / run [N])
        const char *user_input = handle_input();

        // status command: print system state
        if (strcmp(user_input, "status") == 0) {
            if (avg > 0) {
                // Calibration completed, display step count per revolution
                printf("Calibrated: yes\r\n");
                printf("Steps per revolution: %d\r\n", steps_per_rev);
            }
            else {
                // Calibration not yet performed
                printf("Calibrated: no\r\n");
                printf("Not available\r\n");
            }
        }
        // calib command: perform calibration
        else if (strcmp(user_input, "calib") == 0) {
            // Run calibration and compute average from 3 rotations
            avg = calibrate(coil_pins, half_step, safe_max, revolution_steps);
            if (avg > 0) {
                // Update step count per revolution
                steps_per_rev = avg;
                printf("Calibration completed\r\n");
            }
            else {
                // Calibration failed (too few edges detected)
                printf("Calibration failed\r\n");
            }
        }
        // run command: "run" or "run N"
        else if (strncmp(user_input, "run", 3) == 0) {
            // If command is in form "run N"
            if (validate_run_input(user_input)) {
                // Parse numeric argument after "run "
                const int num_out = get_nums_from_a_string(user_input + 4);
                // Run only if N > 0
                if (num_out > 0)
                    run_motor(coil_pins, half_step, num_out, steps_per_rev);
            }
            // If command is plain "run" → rotate one full revolution (8 * 1/8)
            else if (strlen(user_input) == 3) {
                run_motor(coil_pins, half_step, 8, steps_per_rev);
            }
        }
    }
}

void ini_coil_pins(const uint *coil_pins) {
    // Initialize all coil pins as outputs and set them LOW at startup
    for (int i = 0; i < INS_SIZE; i++) {
        gpio_init(coil_pins[i]);
        gpio_set_dir(coil_pins[i], GPIO_OUT);
        gpio_put(coil_pins[i], 0); // Ensure coils are off at startup
    }
}

void ini_sensor() {
    // Initialize the optical sensor input with internal pull-up resistor
    gpio_init(SENSOR);
    gpio_set_dir(SENSOR, GPIO_IN);
    // Internal pull-up: SENSOR reads HIGH (1) when not blocked, LOW (0) when blocked
    gpio_pull_up(SENSOR);
}

int calibrate(const uint *coil_pins, const int half_step[8][4], const int max, int revolution_steps[3]) {
    int count = 0; // Number of falling edges detected
    int step = 0; // total half-steps taken
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
        // Detect falling edge: HIGH -> LOW transition (no obstacle -> obstacle)
        if (prev_state && !sensor_state) {
            if (!first_edge_found) {
                // First falling edge - start counting after this point
                printf("First low edge found\r\n");
                first_edge_found = true;
            }
            else {
                // Store number of steps between consecutive edges
                revolution_steps[count-1] = edge_step;
                printf("%d. round steps: %d\r\n", count, edge_step);
                edge_step = 0;
            }
            count++;
        }
        // Stop after 4 falling edges (3 intervals) or reaching safety limit
        if (count >= 4 || step > max)
            continue_loop = false;
        prev_state = sensor_state;

    } while (continue_loop);

    // At least 4 edges are required to compute 3 intervals (1 revolution)
    if (count >= 4) {
        const int avg = get_avg(revolution_steps);
        return avg;
    }
    return 0; // Calibration failed
}

void step_motor(const uint *coil_pins, const int step, const int half_step[8][4]) {
    // Determines which step phase (0–7) the motor is currently in
    // Bitwise AND preserves only the three lowest bits.
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

void run_motor(const uint *coil_pins, const int half_step[8][4], const int count, const int steps_per_rev) {
    // Calculate total number of half-steps:
    const int i_count = count * (steps_per_rev / 8);
    for (int i = 0; i < i_count; i++) {
        step_motor(coil_pins, i, half_step);
        sleep_ms(3);
    }
}

char *handle_input() {
    // Static buffer for user input
    static char string[INPUT_LENGTH];
    bool stop_loop = false;
    // Keep prompting until valid input is entered
    while (!stop_loop) {
        printf("Enter cmd: ");
        fflush(stdout);
        stop_loop = get_input(string);
    }
    return string;
}

bool get_input(char *user_input) {
    // Read one line from stdin
    if(fgets(user_input, INPUT_LENGTH, stdin)) {
        // If no newline found, input exceeded buffer size -> discard remainder
        if (strchr(user_input, '\n') == NULL) {
            int c = 0;
            while ((c = getchar()) != '\n' && c != EOF) {}
            printf("Input too long (max %d characters).\r\n", INPUT_LENGTH-2);
            return false;
        }
        // Remove trailing newline characters
        trim_line(user_input);
        // Reject empty input
        if (user_input[0] == '\0') {
            printf("Empty input.\r\n");
            return false;
        }
        return true;
    }
    return false;
}

void trim_line(char *user_input) {
    // Remove '\n' and '\r' characters from the end of the line
    int len = (int)strlen(user_input);
    while (len > 0 && (user_input[len - 1] == '\n' || user_input[len - 1] == '\r')) {
        user_input[--len] = '\0';
    }
}

bool check_if_nums(const char *string) {
    // Return true if all characters in string are digits
    const int len = (int)strlen(string);
    for (int i = 0; i < len; i++) {
        if (!isdigit((unsigned char)string[i])) {
            return false;
        }
    }
    return true;
}

int get_nums_from_a_string(const char *string) {
    // Reject immediately if string starts with '0' (leading zeros not allowed)
    if (string[0] != '0') {
        const int len = (int)strlen(string);
        char num_char[len + 1]; // buffer for numeric substring + null terminator
        int j = 0;
        // Collect digits into num_char
        for (int i = 0; i < len; i++) {
            if (isdigit((unsigned)string[i])) {
                num_char[j++] = string[i];
            }
        }
        // If at least one digit found, convert to integer
        if (j > 0) {
            num_char[j] = '\0';
            return atoi(num_char);
        }
    }
    return 0; // If no valid number found return 0
}

bool validate_run_input(const char *user_input) {
    // Accept only form "run N"
    // - at least 4 characters long
    // - 4th character is a space
    // - after the space only digits are allowed
    if (strlen(user_input) >= 4 && check_if_nums(user_input + 4) && user_input[3] == ' ')
        return true;
    return false;
}