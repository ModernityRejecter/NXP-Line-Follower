#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_pwm.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "hbridge.h"
#include "fsl_common.h"
#include "Config.h"
#include "fsl_gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define DEBUG_LF 1

//GLOBAL CALIBRATION CONSTANTS

// Steering control (PID)
float Kp = 0.04f;
float Kd = 0.250f;

// Dynamic speed and slew rate
int16_t MAX_SPEED = 100;
int16_t MIN_CURVE_SPEED = 30;     // Speed on sharp curves
int32_t STRAIGHT_THRESHOLD = 500;

// Acceleration ramp
int16_t ACCEL_STEP = 5;

// Active breaking system
float Kd_Brake = 0.040f;
int16_t EMERGENCY_BRAKE_SPEED = -30;

// Speed used strictly for blind pivoting
int16_t PIVOT_SPEED = 15;


// UTILITIES

#ifndef SystemCoreClock
extern uint32_t SystemCoreClock;
#endif

static inline void delay_ms_blocking(uint32_t ms) {
    SDK_DelayAtLeastUs(ms * 1000U, SystemCoreClock);
}

// SENSORS
static inline uint8_t raw_S10(void) { return GPIO_PinRead(BOARD_INITPINS_S10_GPIO, BOARD_INITPINS_S10_GPIO_PIN); }
static inline uint8_t raw_S11(void) { return GPIO_PinRead(BOARD_INITPINS_S11_GPIO, BOARD_INITPINS_S11_GPIO_PIN); }
static inline uint8_t raw_S12(void) { return GPIO_PinRead(BOARD_INITPINS_S12_GPIO, BOARD_INITPINS_S12_GPIO_PIN); }
static inline uint8_t raw_S13(void) { return GPIO_PinRead(BOARD_INITPINS_S13_GPIO, BOARD_INITPINS_S13_GPIO_PIN); }
static inline uint8_t raw_S14(void) { return GPIO_PinRead(BOARD_INITPINS_S14_GPIO, BOARD_INITPINS_S14_GPIO_PIN); }
static inline uint8_t raw_S15(void) { return GPIO_PinRead(BOARD_INITPINS_S15_GPIO, BOARD_INITPINS_S15_GPIO_PIN); }
static inline uint8_t raw_S16(void) { return GPIO_PinRead(BOARD_INITPINS_S16_GPIO, BOARD_INITPINS_S16_GPIO_PIN); }
static inline uint8_t raw_S17(void) { return GPIO_PinRead(BOARD_INITPINS_S17_GPIO, BOARD_INITPINS_S17_GPIO_PIN); }

uint8_t read_sensors(void) {
    uint8_t sensors = 0;
    if (raw_S10()) sensors |= (1 << 0);
    if (raw_S11()) sensors |= (1 << 1);
    if (raw_S12()) sensors |= (1 << 2);
    if (raw_S13()) sensors |= (1 << 3);
    if (raw_S14()) sensors |= (1 << 4);
    if (raw_S15()) sensors |= (1 << 5);
    if (raw_S16()) sensors |= (1 << 6);
    if (raw_S17()) sensors |= (1 << 7);
    return sensors;
}

void binprintf(uint8_t v) {
    uint8_t mask = 1 << 7;
    while(mask) {
        PRINTF("%d", (v & mask ? 1 : 0));
        mask >>= 1;
    }
}

// POSITION CALCULATION

int32_t calculate_position(uint8_t sensors) {
    if (sensors == 0) return -1;

    int32_t sum = 0;
    int32_t active_sensors = 0;

    for (int i = 0; i < 8; i++) {
        if ((sensors >> i) & 1) {
            sum += i * 1000;
            active_sensors++;
        }
    }
    return sum / active_sensors;
}

// SPEED LIMITING

static inline int16_t clamp_speed(int16_t speed) {
    if (speed > 100) return 100;
    if (speed < -100) return -100;
    return speed;
}

// MAIN

int main(void) {
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL, CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U, GPIO0, 27U);

    delay_ms_blocking(100);

#if DEBUG_LF
    PRINTF("Line Follower SLEW RATE + ANTI-INERTIA Start\r\n");
#endif

    int32_t position = 3500;
    int32_t last_position = 3500;
    int32_t error = 0;
    int32_t previous_error = 0;
    int16_t actual_base_speed = 0;

    bool has_seen_line = false;

    while (1) {
        uint8_t sensors = read_sensors();
        position = calculate_position(sensors);

        int16_t speedL = 0;
        int16_t speedR = 0;
        int32_t correction = 0;
        int16_t target_speed = 0;

        if (position == -1) {
            // Case 1: line lost
            if (!has_seen_line) {
                speedL = 0;
                speedR = 0;
            } else {
                // Completely cut case speed so it doesn't floor it when finding the line
                actual_base_speed = 0;

                if (last_position < 3500) {
                    speedL = -PIVOT_SPEED;
                    speedR = PIVOT_SPEED;
                } else {
                    speedL = PIVOT_SPEED;
                    speedR = -PIVOT_SPEED;
                }
            }
        } else {
            // Case 2: on track
            if (!has_seen_line) {
                error = position - 3500;
                previous_error = error;
                // Start with a soft speed to avoid shocks
                actual_base_speed = MIN_CURVE_SPEED;
                has_seen_line = true;
            }

            last_position = position;
            error = position - 3500;

            int32_t abs_error = abs(error);
            int32_t delta_error = error - previous_error;
            int32_t abs_delta = abs(delta_error);

            // Target speed calculation
            if (abs_error <= STRAIGHT_THRESHOLD) {
                target_speed = MAX_SPEED;
            } else {
                int32_t error_above_threshold = abs_error - STRAIGHT_THRESHOLD;
                int32_t remaining_interval = 3500 - STRAIGHT_THRESHOLD;
                target_speed = MAX_SPEED - (error_above_threshold * (MAX_SPEED - MIN_CURVE_SPEED) / remaining_interval);

                if (target_speed < MIN_CURVE_SPEED) {
                    target_speed = MIN_CURVE_SPEED;
                }
            }

            // Emergency braking applied to target speed
            int16_t brake_penalty = (int16_t)(Kd_Brake * (float)abs_delta);
            target_speed -= brake_penalty;

            if (target_speed < EMERGENCY_BRAKE_SPEED) {
                target_speed = EMERGENCY_BRAKE_SPEED;
            }

            if (target_speed > actual_base_speed) {
                // Gradual acceleration
                actual_base_speed += ACCEL_STEP;
                // Cap the speed on overshoots
                if (actual_base_speed > target_speed) {
                    actual_base_speed = target_speed;
                }
            } else {
                // If the target is lower cut power
                actual_base_speed = target_speed;
            }

            // Steering correction calculations
            float P = Kp * (float)error;
            float D = Kd * (float)delta_error;
            correction = (int32_t)(P + D);

            previous_error = error;

            speedL = actual_base_speed + correction;
            speedR = actual_base_speed - correction;
        }

        speedL = clamp_speed(speedL);
        speedR = clamp_speed(speedR);

        HbridgeSpeed(&g_hbridge, speedR, speedL);

#if DEBUG_LF
        if (has_seen_line) {
            PRINTF("S=");
            binprintf(sensors);
            if (position == -1) {
                PRINTF(" [LOST] Pivot L:%d R:%d\r\n", speedL, speedR);
            } else {
                PRINTF(" V_Real:%d Target:%d Cor:%d L:%d R:%d\r\n", actual_base_speed, target_speed, correction, speedL, speedR);
            }
        }
#endif

        delay_ms_blocking(2);
    }
}
