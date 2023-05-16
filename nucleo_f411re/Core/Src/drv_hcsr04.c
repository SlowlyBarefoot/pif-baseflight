#include "main.h"
#include "drv_hcsr04.h"

#include "sensor/pif_hc_sr04.h"

#ifdef SONAR

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When trigged it sends out a series of 40KHz ultrasonic pulses and receives
 * echo froman object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

static PifHcSr04 s_hcsr04;
static SWITCH s_echo_state;

static sonarDistanceFuncPtr funcSonarDistance;


static void _actHcSr04Trigger(SWITCH state)
{
	HAL_GPIO_WritePin(SONAR_TRIGGER_GPIO_Port, SONAR_TRIGGER_Pin, state);
    s_echo_state = 0;
}

void hcsr04Echo()
{
	s_echo_state ^= 1;
	pifHcSr04_sigReceiveEcho(&s_hcsr04, s_echo_state);
}

static void _evtHcSr04Distance(int32_t distance)
{
    float temp = (*funcSonarDistance)(distance);
    static float pretemp = 0;

    if (temp != pretemp) {
        pifHcSr04_SetTemperature(&s_hcsr04, temp);
        pretemp = temp;
    }
}

BOOL hcsr04Init(uint16_t period, sonarDistanceFuncPtr func)
{
	if (!pifHcSr04_Init(&s_hcsr04, PIF_ID_AUTO)) return FALSE;
	s_hcsr04.act_trigger = _actHcSr04Trigger;
	s_hcsr04.evt_read = _evtHcSr04Distance;
	if (!pifHcSr04_StartTrigger(&s_hcsr04, period)) return FALSE;

	funcSonarDistance = func;
    return TRUE;
}

#endif
