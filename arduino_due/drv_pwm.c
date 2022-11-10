/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "mw.h"

#include "drv_pwm.h"

#include "core/pif_log.h"
#include "core/pif_pulse.h"

/*
    Configuration maps:

    1) multirotor PPM input
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    2) multirotor PPM input with more servos
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for servos

    2) multirotor PWM input
    PWM1..8 used for input
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    3) airplane / flying wing w/PWM
    PWM1..8 used for input
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos

    4) airplane / flying wing with PPM
    PWM1 used for PPM
    PWM5..8 used for servos
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos
*/

typedef struct {
    uint16_t period;

    // for input only
    uint8_t channel;
    uint8_t port;
    PifPulse pulse;
} pwmPortData_t;

enum {
    TYPE_IP = 0x10,
    TYPE_IW = 0x20,
    TYPE_M = 0x40,
    TYPE_S = 0x80
};

typedef struct {
	uint8_t pwm;
	uint8_t type;
	uint8_t port;
} hardwareMaps_t;

typedef void (*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors

static pwmPortData_t pwmPorts[MAX_PORTS];
static uint16_t captures[MAX_PPM_INPUTS];    // max out the captures array, just in case...
static pwmPortData_t *motors[MAX_MOTORS];
static pwmPortData_t *servos[MAX_SERVOS];
static pwmWriteFuncPtr pwmWritePtr = NULL;
static uint8_t numMotors = 0;
static uint8_t numServos = 0;
static uint8_t numInputs = 0;
static uint8_t pwmFilter = 0;
static uint16_t failsafeThreshold = 985;
// external vars (ugh)
extern int16_t failsafeCnt;

static const hardwareMaps_t multiPPM[] = {
    { PWM1, TYPE_IP, 37 },     // PPM input
	{ PWM9, TYPE_M, 2 },      // Swap to servo if needed
	{ PWM10, TYPE_M, 3 },     // Swap to servo if needed
	{ PWM11, TYPE_M, 5 },
	{ PWM12, TYPE_M, 6 },
	{ PWM13, TYPE_M, 7 },
	{ PWM14, TYPE_M, 8 },
	{ PWM5, TYPE_M, 9 },      // Swap to servo if needed
	{ PWM6, TYPE_M, 11 },      // Swap to servo if needed
	{ PWM7, TYPE_M, 12 },      // Swap to servo if needed
	{ PWM8, TYPE_M, 13 },      // Swap to servo if needed
	{ 0xFF, 0, 0 }
};

static const hardwareMaps_t multiPWM[] = {
    { PWM1, TYPE_IW, 37 },     // input #1
	{ PWM2, TYPE_IW, 38 },
	{ PWM3, TYPE_IW, 36 },
	{ PWM4, TYPE_IW, 39 },
	{ PWM5, TYPE_IW, 40 },
	{ PWM6, TYPE_IW, 41 },
	{ PWM7, TYPE_IW, 42 },
	{ PWM8, TYPE_IW, 43 },     // input #8
	{ PWM9, TYPE_M, 2 },      // motor #1 or servo #1 (swap to servo if needed)
	{ PWM10, TYPE_M, 3 },     // motor #2 or servo #2 (swap to servo if needed)
	{ PWM11, TYPE_M, 5 },     // motor #1 or #3
	{ PWM12, TYPE_M, 6 },
	{ PWM13, TYPE_M, 7 },
	{ PWM14, TYPE_M, 8 },     // motor #4 or #6
	{ 0xFF, 0, 0 }
};

static const hardwareMaps_t airPPM[] = {
    { PWM1, TYPE_IP, 37 },     // PPM input
	{ PWM9, TYPE_M, 2 },      // motor #1
	{ PWM10, TYPE_M, 3 },     // motor #2
	{ PWM11, TYPE_S, 5 },     // servo #1
	{ PWM12, TYPE_S, 6 },
	{ PWM13, TYPE_S, 7 },
	{ PWM14, TYPE_S, 8 },     // servo #4
	{ PWM5, TYPE_S, 9 },      // servo #5
	{ PWM6, TYPE_S, 10 },
	{ PWM7, TYPE_S, 11 },
	{ PWM8, TYPE_S, 12 },      // servo #8
	{ 0xFF, 0, 0 }
};

static const hardwareMaps_t airPWM[] = {
    { PWM1, TYPE_IW, 37 },     // input #1
	{ PWM2, TYPE_IW, 38 },
	{ PWM3, TYPE_IW, 36 },
	{ PWM4, TYPE_IW, 39 },
	{ PWM5, TYPE_IW, 40 },
	{ PWM6, TYPE_IW, 41 },
	{ PWM7, TYPE_IW, 42 },
	{ PWM8, TYPE_IW, 43 },     // input #8
	{ PWM9, TYPE_M, 2 },      // motor #1
	{ PWM10, TYPE_M, 3 },     // motor #2
	{ PWM11, TYPE_S, 5 },     // servo #1
	{ PWM12, TYPE_S, 6 },
	{ PWM13, TYPE_S, 7 },
	{ PWM14, TYPE_S, 8 },     // servo #4
	{ 0xFF, 0, 0 }
};

static const hardwareMaps_t *const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

#define PWM_TIMER_MHZ 1
#define PWM_TIMER_8_MHZ 8

static void failsafeCheck(uint8_t channel, uint16_t pulse)
{
    static uint8_t goodPulses;

    if (channel < 4 && pulse > failsafeThreshold)
        goodPulses |= (1 << channel);       // if signal is valid - mark channel as OK
    if (goodPulses == 0x0F) {               // If first four chanells have good pulses, clear FailSafe counter
        goodPulses = 0;
        if (failsafeCnt > 20)
            failsafeCnt -= 20;
        else
            failsafeCnt = 0;
    }
}

static void _isrPulsePosition()
{
    PifPulse* p_pulse = &pwmPorts[0].pulse;

    if (pifPulse_sigTick(p_pulse, (*pif_act_timer1us)())) {
        failsafeCheck(p_pulse->_channel, captures[p_pulse->_channel]);
    }
}

static void _isrPulseWidth(uint8_t index)
{
	pifPulse_sigEdge(&pwmPorts[index].pulse, digitalRead(pwmPorts[index].port) ? PE_RISING : PE_FALLING, (*pif_act_timer1us)());
	captures[index] = pifPulse_GetHighWidth(&pwmPorts[index].pulse);
    if (captures[index]) {
        failsafeCheck(pwmPorts[index].channel, captures[index]);
    }
}

static void _isrPulseWidth1()
{
	_isrPulseWidth(0);
}

static void _isrPulseWidth2()
{
	_isrPulseWidth(1);
}

static void _isrPulseWidth3()
{
	_isrPulseWidth(2);
}

static void _isrPulseWidth4()
{
	_isrPulseWidth(3);
}

static void _isrPulseWidth5()
{
	_isrPulseWidth(4);
}

static void _isrPulseWidth6()
{
	_isrPulseWidth(5);
}

static void _isrPulseWidth7()
{
	_isrPulseWidth(6);
}

static void _isrPulseWidth8()
{
	_isrPulseWidth(7);
}

static void pwmWriteBrushed(uint8_t index, uint16_t value)
{
	analogWrite(motors[index]->port, (value - 1000) * motors[index]->period / 1000);
}

static void pwmWriteStandard(uint8_t index, uint16_t value)
{
	analogWrite(motors[index]->port, value * 4096L / 2500L);
}

bool pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const hardwareMaps_t *setup;
    uint16_t period;
    pwmPortData_t *p;

    // to avoid importing cfg/mcfg
    failsafeThreshold = init->failsafeThreshold;
    // pwm filtering on input
    pwmFilter = init->pwmFilter;

    // this is pretty hacky shit, but it will do for now. array of 4 config maps, [ multiPWM multiPPM airPWM airPPM ]
    if (init->airplane)
        i = 2; // switch to air hardware config
    if (init->usePPM)
        i++; // next index is for PPM

    setup = hardwareMaps[i];

    for (i = 0; i < MAX_PORTS; i++) {
        uint8_t pwm = setup[i].pwm;
        uint8_t type = setup[i].type;

        if (pwm == 0xFF) // terminator
            break;

        // hacks to allow current functionality
        if ((type & (TYPE_IP | TYPE_IW)) && !init->enableInput)
        	type = 0;

        if (type & TYPE_IP) {
            p = &pwmPorts[pwm];
            if (pifPulse_Init(&p->pulse, PIF_ID_AUTO)) {
                pifPulse_SetPositionMode(&p->pulse, 8, 2700, captures);
                pifPulse_SetValidRange(&p->pulse, PIF_PMM_TICK_POSITION, PULSE_MIN, PULSE_MAX);
                p->port = setup[i].port;
            	pinMode(p->port, INPUT_PULLUP);
               	attachInterrupt(p->port, _isrPulsePosition, RISING);
                numInputs = 8;
            }
        } else if (type & TYPE_IW) {
            p = &pwmPorts[pwm];
            if (pifPulse_Init(&p->pulse, PIF_ID_AUTO)) {
                pifPulse_SetMeasureMode(&p->pulse, PIF_PMM_EDGE_HIGH_WIDTH);
                pifPulse_SetValidRange(&p->pulse, PIF_PMM_EDGE_HIGH_WIDTH, PULSE_MIN, PULSE_MAX);
                p->port = setup[i].port;
            	pinMode(p->port, INPUT_PULLUP);
            	switch (i) {
            	case 0:
                	attachInterrupt(p->port, _isrPulseWidth1, CHANGE);
                	break;
            	case 1:
                	attachInterrupt(p->port, _isrPulseWidth2, CHANGE);
                	break;
            	case 2:
                	attachInterrupt(p->port, _isrPulseWidth3, CHANGE);
                	break;
            	case 3:
                	attachInterrupt(p->port, _isrPulseWidth4, CHANGE);
                	break;
            	case 4:
                	attachInterrupt(p->port, _isrPulseWidth5, CHANGE);
                	break;
            	case 5:
                	attachInterrupt(p->port, _isrPulseWidth6, CHANGE);
                	break;
            	case 6:
                	attachInterrupt(p->port, _isrPulseWidth7, CHANGE);
                	break;
            	case 7:
                	attachInterrupt(p->port, _isrPulseWidth8, CHANGE);
                	break;
            	}
                numInputs++;
            }
        } else if (type & TYPE_M) {
            uint32_t hz, mhz;

        	pinMode(setup[i].port, OUTPUT);

            if (init->motorPwmRate > 500)
                mhz = PWM_TIMER_8_MHZ;
            else
                mhz = PWM_TIMER_MHZ;

            hz = mhz * 1000000;

            period = hz / init->motorPwmRate;

            pwmPorts[pwm].period = period;
            pwmPorts[pwm].port = setup[i].port;
            motors[numMotors++] = &pwmPorts[pwm];
        } else if (type & TYPE_S) {
        	pinMode(setup[i].port, OUTPUT);

        	pwmPorts[pwm].period = 1000000 / init->servoPwmRate;
            pwmPorts[pwm].port = setup[i].port;
            servos[numServos++] = &pwmPorts[pwm];
        }
    }

    // determine motor writer function
    pwmWritePtr = pwmWriteStandard;
    if (init->motorPwmRate > 500)
        pwmWritePtr = pwmWriteBrushed;

    // set return values in init struct
    init->numServos = numServos;

    return false;
}

void actPwmWriteMotor(uint8_t index, uint16_t value)
{
    if (index < numMotors)
        pwmWritePtr(index, value);
}

void actPwmWriteServo(uint8_t index, uint16_t value)
{
    if (index < numServos)
    	analogWrite(servos[index]->port, value);
}

uint16_t actPwmRead(uint8_t channel)
{
    return captures[channel];
}
