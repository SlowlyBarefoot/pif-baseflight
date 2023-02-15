#include "board.h"
#include "link_driver.h"


PifI2cPort g_i2c_port;
PifTimerManager g_timer_1ms;
PifTask* g_task_compute_imu;
PifTask* g_task_compute_rc;
PifTask* g_task_gps;


PIF_WEAK void systemReset(bool toBootloader)
{
	(void)toBootloader;
}

PIF_WEAK void failureMode(uint8_t mode)
{
	(void)mode;
}

PIF_WEAK void actLed0State(BOOL state)
{
	(void)state;
}

PIF_WEAK void actLed0Toggle()
{
}

PIF_WEAK void actLed1State(BOOL state)
{
	(void)state;
}

PIF_WEAK void actLed1Toggle()
{
}

PIF_WEAK void actInvState(BOOL state)
{
	(void)state;
}

PIF_WEAK void actBuzzerAction(PifId id, BOOL action)
{
	(void)id;
	(void)action;
}

PIF_WEAK uint16_t actGetAdcChannel(uint8_t channel)
{
	(void)channel;
	return 0;
}

PIF_WEAK float actGetBatteryVoltage()
{
	return 0.0;
}

PIF_WEAK uint32_t actGetBatteryCurrent()
{
	return 0UL;
}

PIF_WEAK void actPwmWriteMotor(uint8_t index, uint16_t value)
{
	(void)index;
	(void)value;
}

PIF_WEAK void actPwmWriteServo(uint8_t index, uint16_t value)
{
	(void)index;
	(void)value;
}

PIF_WEAK uint16_t actPwmRead(uint8_t channel)
{
	(void)channel;
	return 0;
}

PIF_WEAK serialPort_t *uartOpen(int port, uint32_t baudRate, portMode_t mode, uint8_t period)
{
	(void)port;
	(void)baudRate;
	(void)mode;
	(void)period;
	return NULL;
}

PIF_WEAK BOOL serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
	(void)instance;
	(void)baudRate;
	return TRUE;
}

PIF_WEAK BOOL serialStartReceiveFunc(PifComm* p_comm)
{
	(void)p_comm;
	return TRUE;
}

PIF_WEAK BOOL serialStopReceiveFunc(PifComm* p_comm)
{
	(void)p_comm;
	return TRUE;
}
