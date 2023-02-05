/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "main.h"
#include "board.h"
#include "link_driver.h"

#include "drv_pwm.h"

#include "rc/pif_rc_ppm.h"
#include "rc/pif_rc_pwm.h"

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

static union {
	PifRcPwm pwm;
	PifRcPpm ppm;
} s_rc;
static pwmPortData_t pwmPorts[MAX_PORTS];
static uint16_t captures[MAX_PPM_INPUTS];    // max out the captures array, just in case...
static pwmPortData_t *motors[MAX_MOTORS];
static pwmPortData_t *servos[MAX_SERVOS];
static pwmWriteFuncPtr pwmWritePtr = NULL;
static uint8_t numMotors = 0;
static uint8_t numServos = 0;
static uint8_t numInputs = 0;
static uint8_t pwmFilter = 0;
static bool syncPWM = false;
static uint16_t failsafeThreshold = 985;
// external vars (ugh)
extern int16_t failsafeCnt;

static const hardwareMaps_t multiPPM[] = {
    { PWM1, TYPE_IP, 37 },     // PPM input
	{ PWM9, TYPE_M, TIM_CHANNEL_1 },      // Swap to servo if needed
	{ PWM10, TYPE_M, TIM_CHANNEL_2 },     // Swap to servo if needed
	{ PWM11, TYPE_M, TIM_CHANNEL_3 },
	{ PWM12, TYPE_M, TIM_CHANNEL_4 },
	{ 0xFF, 0, 0 }
};

static const hardwareMaps_t multiPWM[] = {
    { PWM1, TYPE_IW, 38 },     // input #1
	{ PWM2, TYPE_IW, 39 },
	{ PWM3, TYPE_IW, 40 },
	{ PWM4, TYPE_IW, 41 },
	{ PWM5, TYPE_IW, 42 },
	{ PWM6, TYPE_IW, 43 },
	{ PWM7, TYPE_IW, 44 },
	{ PWM8, TYPE_IW, 45 },     // input #8
	{ PWM9, TYPE_M, TIM_CHANNEL_1 },      // motor #1 or servo #1 (swap to servo if needed)
	{ PWM10, TYPE_M, TIM_CHANNEL_2 },     // motor #2 or servo #2 (swap to servo if needed)
	{ PWM11, TYPE_M, TIM_CHANNEL_3 },     // motor #1 or #3
	{ PWM12, TYPE_M, TIM_CHANNEL_4 },
	{ 0xFF, 0, 0 }
};

static const hardwareMaps_t airPPM[] = {
    { PWM1, TYPE_IP, 37 },     // PPM input
	{ PWM9, TYPE_M, TIM_CHANNEL_1 },      // motor #1
	{ PWM10, TYPE_M, TIM_CHANNEL_2 },     // motor #2
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
    { PWM1, TYPE_IW, 38 },     // input #1
	{ PWM2, TYPE_IW, 39 },
	{ PWM3, TYPE_IW, 40 },
	{ PWM4, TYPE_IW, 41 },
	{ PWM5, TYPE_IW, 42 },
	{ PWM6, TYPE_IW, 43 },
	{ PWM7, TYPE_IW, 44 },
	{ PWM8, TYPE_IW, 45 },     // input #8
	{ PWM9, TYPE_M, TIM_CHANNEL_1 },      // motor #1
	{ PWM10, TYPE_M, TIM_CHANNEL_2 },     // motor #2
	{ PWM11, TYPE_S, 3 },     // servo #1
	{ PWM12, TYPE_S, 4 },
	{ PWM13, TYPE_S, 5 },
	{ PWM14, TYPE_S, 6 },     // servo #4
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

void pwmReadRc()
{
	uint16_t value = pifRcPpm_sigTick(&s_rc.ppm, (*pif_act_timer1us)());
    if (value) {
        failsafeCheck(s_rc.ppm._channel, value);
    }
}

static void _evtRcReceive(PifRc* p_owner, uint16_t* p_channel, PifIssuerP p_issuer)
{
    PifTask* p_task = (PifTask*)p_issuer;
    int i;

	for (i = 0; i < p_owner->_channel_count; i++) {
		captures[i] = p_channel[i];
	}
    pifTask_SetTrigger(p_task);
}

static void pwmWriteStandard(uint8_t index, uint16_t value)
{
	switch (motors[index]->port) {
	case TIM_CHANNEL_1:	htim3.Instance->CCR1 = value; break;
	case TIM_CHANNEL_2:	htim3.Instance->CCR2 = value; break;
	case TIM_CHANNEL_3:	htim3.Instance->CCR3 = value; break;
	case TIM_CHANNEL_4:	htim3.Instance->CCR4 = value; break;
	}
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

    syncPWM = init->syncPWM;

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
            numInputs = 8;
        } else if (type & TYPE_IW) {
/*
            p = &pwmPorts[pwm];
			pinMode(setup[i].port, INPUT_PULLUP);
			switch (i) {
			case 0:
				attachInterrupt(setup[i].port, _isrPulseWidth1, CHANGE);
				break;
			case 1:
				attachInterrupt(setup[i].port, _isrPulseWidth2, CHANGE);
				break;
			case 2:
				attachInterrupt(setup[i].port, _isrPulseWidth3, CHANGE);
				break;
			case 3:
				attachInterrupt(setup[i].port, _isrPulseWidth4, CHANGE);
				break;
			case 4:
				attachInterrupt(setup[i].port, _isrPulseWidth5, CHANGE);
				break;
			case 5:
				attachInterrupt(setup[i].port, _isrPulseWidth6, CHANGE);
				break;
			case 6:
				attachInterrupt(setup[i].port, _isrPulseWidth7, CHANGE);
				break;
			case 7:
				attachInterrupt(setup[i].port, _isrPulseWidth8, CHANGE);
				break;
			}
			numInputs++;
*/
        } else if (type & TYPE_M) {
            uint32_t hz, mhz;

            if (init->motorPwmRate > 500 || init->fastPWM)
                mhz = PWM_TIMER_8_MHZ;
            else
                mhz = PWM_TIMER_MHZ;

            hz = mhz * 1000000;

            if (init->fastPWM)
                period = hz / 4000;
            else
                period = hz / init->motorPwmRate;

            pwmPorts[pwm].period = period;
            pwmPorts[pwm].port = setup[i].port;
            HAL_TIM_PWM_Start(&htim3, pwmPorts[pwm].port);
            motors[numMotors++] = &pwmPorts[pwm];
        } else if (type & TYPE_S) {
        	pwmPorts[pwm].period = 1000000 / init->servoPwmRate;
            servos[numServos++] = &pwmPorts[pwm];
        }
    }

    if (init->enableInput) {
		if (init->usePPM) {
			if (pifRcPpm_Init(&s_rc.ppm, PIF_ID_AUTO, numInputs, 2700)) {
				pifRcPpm_SetValidRange(&s_rc.ppm, PULSE_MIN, PULSE_MAX);
				pifRc_AttachEvtReceive(&s_rc.ppm.parent, _evtRcReceive, g_task_compute_rc);
			}
		}
		else {
			if (pifRcPwm_Init(&s_rc.pwm, PIF_ID_AUTO, numInputs)) {
				pifRcPwm_SetValidRange(&s_rc.pwm, PULSE_MIN, PULSE_MAX);
				pifRc_AttachEvtReceive(&s_rc.pwm.parent, _evtRcReceive, g_task_compute_rc);
			}
		}
    }

    // determine motor writer function
    pwmWritePtr = pwmWriteStandard;

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
//    if (index < numServos)
//    	analogWrite(servos[index]->port, value);
}

uint16_t actPwmRead(uint8_t channel)
{
    return captures[channel];
}
