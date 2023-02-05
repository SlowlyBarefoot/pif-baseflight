#pragma once

#include "core/pif_comm.h"
#include "core/pif_i2c.h"
#include "core/pif_timer.h"
#include "sensor/pif_imu_sensor.h"
#include "sensor/pif_sensor_event.h"
#include "storage/pif_storage.h"


#define PIF_ID_UART(N)              (0x100 + (N))   // N : 0=UART1, 1=UART2, 2=UART3
#define PIF_ID_UART_2_IDX(N)        ((N) - 0x100)   // N : 0x100=UART1, 0x101=UART2, 0x102=UART3

#define PIF_ID_MSP(N)               (0x110 + (N))   // N : 0=main port, 1=flex port
#define PIF_ID_MSP_2_IDX(N)         ((N) - 0x110)   // N : 0x110=main port, 0x111=flex port

#define PIF_ID_LED(N)               (0x120 + (N))   // N : 0=LED0, 1=LED1
#define PIF_ID_LED_2_IDX(N)         ((N) - 0x120)   // N : 0x120=LED0, 0x121=LED1

#define PIF_ID_BUZZER               0x130

#define DISALLOW_YIELD_ID_I2C       1

#define PULSE_1MS   (1000)      // 1ms pulse width
#define PULSE_MIN   (750)       // minimum PWM pulse width which is considered valid
#define PULSE_MAX   (2250)      // maximum PWM pulse width which is considered valid


typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;

/*********** RC alias *****************/
enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

typedef enum {
    ADC_BATTERY = 0,
    ADC_EXTERNAL_PAD = 1,
    ADC_EXTERNAL_CURRENT = 2,
    ADC_RSSI = 3,
    ADC_CHANNEL_MAX = 4
} AdcChannel;

typedef enum {
	UART_PORT_NONE = 0,
	UART_PORT_1,
	UART_PORT_2,
	UART_PORT_3,
} UartPort;


struct sensorSet_t;
typedef struct sensorSet_t sensorSet_t;

typedef BOOL (*sensorInitFuncPtr)(sensorSet_t* p_sensor_set, PifImuSensorAlign align);// sensor init prototype
typedef BOOL (*sensorReadFuncPtr)(sensorSet_t* p_sensor_set, int16_t *data);          // sensor read and align prototype

struct sensorSet_t {
    PifImuSensor imu_sensor;
	struct {
	    const char* hardware;
	    sensorInitFuncPtr init;                         // initialize function
	    sensorReadFuncPtr read;                         // read 3 axis data function
		sensorReadFuncPtr temperature;                  // read temperature if available
		uint16_t lpf;
		float scale;                                    // scalefactor (currently used for gyro only, todo for accel)
	} gyro;
	struct {
	    const char* hardware;
	    sensorInitFuncPtr init;                         // initialize function
	    sensorReadFuncPtr read;                         // read 3 axis data function
	    uint16_t acc_1G;
	} acc;
	struct {
	    const char* hardware;
	    sensorInitFuncPtr init;                         // initialize function
	    sensorReadFuncPtr read;                         // read 3 axis data function
		PifTask* p_task;
		float declination;       						// calculated at startup from config
	} mag;
#ifdef BARO
	struct {
	    const char* hardware;
	    sensorInitFuncPtr init;                         // initialize function
	    PifEvtBaroRead evt_read;
   	    PifTask* p_task;
	} baro;
#endif
};

typedef bool (*sensorDetectFuncPtr)(sensorSet_t *p_sensor_set, void* p_param);

typedef struct sensor_detect_t {
	sensorDetectFuncPtr p_func;
	void* p_param;
} sensorDetect_t;

// sonar
typedef enum {
    SF_NONE 	    = 0,
    SF_AVERAGE	    = 1,
    SF_NOISE_CANCEL	= 2
} sonar_filter_t;

typedef float (*sonarDistanceFuncPtr)(int32_t distance);						// 반환값은 기온 (도)
typedef BOOL (*sonarInitFuncPtr)(uint16_t period, sonarDistanceFuncPtr func);	// period unit : ms

// Serial Port
typedef enum portMode_t {
    MODE_RX = 1 << 0,
    MODE_TX = 1 << 1,
    MODE_RXTX = MODE_RX | MODE_TX,
    MODE_SBUS = 1 << 2
} portMode_t;

typedef struct serialPort {
    portMode_t mode;
    PifComm comm;

    void* p_param;
} serialPort_t;


extern PifI2cPort g_i2c_port;
extern PifTimerManager g_timer_1ms;
extern PifTask* g_task_compute_rc;
extern PifTask* g_task_compute_imu;
extern PifTask* g_task_gps;


#ifdef __cplusplus
extern "C" {
#endif

// bootloader/IAP
void systemReset(bool toBootloader);

// failure
void failureMode(uint8_t mode);

// led
void actLed0State(BOOL state);
void actLed0Toggle();
void actLed1State(BOOL state);
void actLed1Toggle();

void actInvState(BOOL state);

// buzzer
void actBuzzerAction(PifId id, BOOL action);

// adc sensor
uint16_t actGetAdcChannel(uint8_t channel);
float actGetBatteryVoltage();
uint32_t actGetBatteryCurrent();

// pwm
void actPwmWriteMotor(uint8_t index, uint16_t value);
void actPwmWriteServo(uint8_t index, uint16_t value);
uint16_t actPwmRead(uint8_t channel);

// uart
serialPort_t *uartOpen(int port, uint32_t baudRate, portMode_t mode);
BOOL serialSetBaudRate(serialPort_t *instance, uint32_t baudRate);
BOOL serialStartReceiveFunc(PifComm* p_comm);
BOOL serialStopReceiveFunc(PifComm* p_comm);

#ifdef __cplusplus
}
#endif
