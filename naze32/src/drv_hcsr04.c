#include "board.h"

#include "drv_gpio.h"
#include "drv_hcsr04.h"

#ifdef __PIF_DEBUG__
	#include "core/pif_log.h"
#endif
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

//#define HC_SR05_PWM
#define HC_SR05_RC

static uint16_t trigger_pin;
static uint16_t echo_pin;
static uint32_t exti_line;
static uint8_t exti_pin_source;
static IRQn_Type exti_irqn;

static PifHcSr04 s_hcsr04;
static SWITCH s_echo_state;

static sonarDistanceFuncPtr funcSonarDistance;


static void _actHcSr04Trigger(SWITCH state)
{
    if (state) {
        digitalHi(GPIOB, trigger_pin);
    }
    else {
        digitalLo(GPIOB, trigger_pin);
    }
    s_echo_state = 0;
}

void ECHO_EXTI_IRQHandler(void)
{
	s_echo_state ^= 1;
	pifHcSr04_sigReceiveEcho(&s_hcsr04, s_echo_state);

    EXTI_ClearITPendingBit(exti_line);
}

void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

static void _evtHcSr04Distance(int32_t distance)
{
    float temp = (*funcSonarDistance)(distance);
    static float pretemp = 0;

    if (temp != pretemp) {
        pifHcSr04_SetTemperature(&s_hcsr04, temp);
#ifdef __PIF_DEBUG__
        pifLog_Printf(LT_INFO, "Temp=%f", temp);
#endif
        pretemp = temp;
    }
}

BOOL hcsr04Init(uint16_t period, sonarDistanceFuncPtr func)
{
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTIInit;

	if (!pifHcSr04_Init(&s_hcsr04, PIF_ID_AUTO)) return FALSE;
	s_hcsr04.act_trigger = _actHcSr04Trigger;
	s_hcsr04.evt_read = _evtHcSr04Distance;
	if (!pifHcSr04_StartTrigger(&s_hcsr04, period)) return FALSE;

	funcSonarDistance = func;

    // enable AFIO for EXTI support - already done is drv_system.c
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph, ENABLE);

#ifdef HC_SR05_PWM
    trigger_pin = Pin_8;   // PWM5 (PB8) - 5v tolerant
    echo_pin = Pin_9;      // PWM6 (PB9) - 5v tolerant
    exti_line = EXTI_Line9;
    exti_pin_source = GPIO_PinSource9;
    exti_irqn = EXTI9_5_IRQn;
#endif

#ifdef HC_SR05_RC
    trigger_pin = Pin_0;   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
    echo_pin = Pin_1;      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
    exti_line = EXTI_Line1;
    exti_pin_source = GPIO_PinSource1;
    exti_irqn = EXTI1_IRQn;
#endif

    // tp - trigger pin
    gpio.pin = trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOB, &gpio);

    // ep - echo pin
    gpio.pin = echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOB, &gpio);

    // setup external interrupt on echo pin
    gpioExtiLineConfig(GPIO_PortSourceGPIOB, exti_pin_source);

    EXTI_ClearITPendingBit(exti_line);

    EXTIInit.EXTI_Line = exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_EnableIRQ(exti_irqn);
    return TRUE;
}

#endif
