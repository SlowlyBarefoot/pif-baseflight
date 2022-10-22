/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

typedef struct drv_pwm_config_t {
    bool enableInput;
    bool usePPM;
    bool useServos;
    bool extraServos;    // configure additional 4 channels in PPM mode as servos, not motors
    bool airplane;       // fixed wing hardware config, lots of servos etc
    uint8_t pwmFilter;   // PWM ICFilter value for jittering input
    uint8_t adcChannel;  // steal one RC input for current sensor
    uint16_t motorPwmRate;
    uint16_t servoPwmRate;
    uint16_t idlePulse;  // PWM value to use when initializing the driver. set this to either PULSE_1MS (regular pwm),
    // some higher value (used by 3d mode), or 0, for brushed pwm drivers.
    uint16_t servoCenterPulse;
    uint16_t failsafeThreshold;

    // OUT parameters, filled by driver
    uint8_t numServos;
} drv_pwm_config_t;

// This indexes into the read-only hardware definition structure in drv_pwm.c, as well as into pwmPorts[] structure with dynamic data.
enum {
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM9,
    PWM10,
    PWM11,
    PWM12,
    PWM13,
    PWM14,
    MAX_PORTS
};

#ifdef __cplusplus
extern "C" {
#endif

bool pwmInit(drv_pwm_config_t *init); // returns whether driver is asking to calibrate throttle or not

#ifdef __cplusplus
}
#endif
