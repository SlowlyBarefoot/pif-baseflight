// Do not remove the include below
#include "baseflight.h"
#include "board.h"
#include "buzzer.h"
#include "mw.h"
#ifdef TELEMETRY
#include "telemetry_common.h"
#endif

#include "drv_adc.h"
#include "drv_gy86.h"
#include "drv_hcsr04.h"
#include "drv_i2c.h"
#include "drv_pwm.h"
#include "drv_system.h"
#include "drv_uart.h"

#include "core/pif_log.h"

#include <DueFlashStorage.h>
#ifdef USE_I2C_WIRE
	#include <Wire.h>
#endif


#define EFC_ACCESS_MODE_128 	0
#define FLASH_ACCESS_MODE_128 	EFC_ACCESS_MODE_128
#define EFC 					EFC0


const char g_board_name[] = "Arduino Due";

core_t core;
int hw_revision = 0;

static sensorDetect_t gyro_detect[] = { { (sensorDetectFuncPtr)gy86Detect, NULL }, { NULL, NULL } };
static sensorDetect_t* acc_detect = NULL;
static sensorDetect_t* baro_detect = NULL;
static sensorDetect_t* mag_detect = NULL;


static uint32_t _GetArduinoDueUid(uint32_t* unique_id)
{
	uint32_t ul_rc;

	ul_rc = efc_init(EFC, FLASH_ACCESS_MODE_128, 4);
	if (ul_rc != 0) {
		return ul_rc;
	}

	return flash_read_unique_id(unique_id, 4);
}

static void featureDefault(void)
{
    featureSet(FEATURE_VBAT);
}

#ifdef __PIF_DEBUG__

void actTaskSignal(BOOL state)
{
	digitalWrite(13, state);
}

#endif

extern "C" {
	int sysTickHook()
	{
		pif_sigTimer1ms();
		pifTimerManager_sigTick(&g_timer_1ms);
		return 0;
	}
}

//The setup function is called once at startup of the sketch
void setup()
{
    uint8_t i;
#ifdef __PIF_DEBUG__
    int line;
	#define FAIL { line = __LINE__; goto fail; }
#else
	#define FAIL { goto fail; }
#endif
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;
    bool sensorsOK = false;
    PifTask* p_task;

	analogReadResolution(12);
	analogWriteResolution(12);

    g_crystal_clock = F_CPU;
    g_core_clock = SystemCoreClock;

    _GetArduinoDueUid(g_unique_id);

#ifndef __PIF_NO_LOG__
	Serial.begin(115200);
#endif

#ifdef USE_I2C_WIRE
	Wire.begin();
	Wire.setClock(400000);
#else
	I2C_Init(I2C_CLOCK_400KHz);
#endif

    pif_Init(micros);

    if (!pifTaskManager_Init(20)) FAIL;

#ifdef __PIF_DEBUG__
    pif_act_task_signal = actTaskSignal;

    logOpen();
#endif

    if (!pifTimerManager_Init(&g_timer_1ms, PIF_ID_AUTO, 1000, 3)) FAIL;		        // 1000us

#ifdef __PIF_DEBUG__
    pifLog_Printf(LT_INFO, "Start Baseflight: %d\n", sizeof(master_t));
#endif

    if (!buzzerInit()) FAIL;

    // make sure (at compile time) that config struct doesn't overflow allocated flash pages
    ct_assert(sizeof(mcfg) < STORAGE_VOLUME);

    g_featureDefault = featureDefault;

    if (!pifI2cPort_Init(&g_i2c_port, PIF_ID_AUTO, 5, I2C_TRANSFER_SIZE)) FAIL;
    g_i2c_port.act_read = actI2cRead;
    g_i2c_port.act_write = actI2cWrite;

    if (!initEEPROM(storageInit())) FAIL;
    if (!checkFirstTime(false)) FAIL;
    readEEPROM();

    systemInit();

    // sleep for 100ms
    pif_Delay1ms(100);

    activateConfig();

    // configure rssi ADC
    if (mcfg.rssi_adc_channel > 0 && (mcfg.rssi_adc_channel == 1 || mcfg.rssi_adc_channel == 9 || mcfg.rssi_adc_channel == 5) && mcfg.rssi_adc_channel != mcfg.power_adc_channel)
        adc_params.rssiAdcChannel = mcfg.rssi_adc_channel;
    else {
        adc_params.rssiAdcChannel = 0;
        mcfg.rssi_adc_channel = 0;
    }

    adcInit(&adc_params);
    // Check battery type/voltage
    if (feature(FEATURE_VBAT))
        batteryInit();
    initBoardAlignment();

    // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
    sensorsOK = sensorsAutodetect(gyro_detect, acc_detect, baro_detect, mag_detect);
#ifdef __PIF_DEBUG__
    pifLog_Printf(LT_INFO, "Sensor: %lxh(%d)", sensorsMask(), sensorsOK);
#endif

    // if gyro was not detected due to whatever reason, we give up now.
    if (!sensorsOK)
        failureMode(3);

    actLed1State(ON);
    actLed0State(OFF);
    for (i = 0; i < 10; i++) {
        actLed1Toggle();
        actLed0Toggle();
        pif_Delay1ms(25);
        actBuzzerAction(PIF_ID_BUZZER, ON);
        pif_Delay1ms(25);
        actBuzzerAction(PIF_ID_BUZZER, OFF);
    }
    actLed0State(OFF);
    actLed1State(OFF);

    imuInit(); // Mag is initialized inside imuInit
    mixerInit(); // this will set core.useServo var depending on mixer type

    serialInit(UART_PORT_1, mcfg.serial_baudrate, UART_PORT_NONE);

    g_task_compute_rc = pifTaskManager_Add(TM_EXTERNAL_ORDER, 0, taskComputeRc, NULL, FALSE);
    if (!g_task_compute_rc) FAIL;
    g_task_compute_rc->name = "RC";

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_FLYING_WING || mcfg.mixerConfiguration == MULTITYPE_CUSTOM_PLANE)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SERIALRX); // disable inputs if using spektrum
    pwm_params.useServos = core.useServo;
    pwm_params.extraServos = cfg.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = mcfg.motor_pwm_rate;
    pwm_params.servoPwmRate = mcfg.servo_pwm_rate;
    pwm_params.pwmFilter = mcfg.pwm_filter;
    pwm_params.idlePulse = PULSE_1MS; // standard PWM for brushless ESC (default, overridden below)
    if (feature(FEATURE_3D))
        pwm_params.idlePulse = mcfg.neutral3d;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors
    pwm_params.servoCenterPulse = mcfg.midrc;
    pwm_params.failsafeThreshold = cfg.failsafe_detect_threshold;
    switch (mcfg.power_adc_channel) {
        case 1:
            pwm_params.adcChannel = PWM2;
            break;
        case 9:
            pwm_params.adcChannel = PWM8;
            break;
        default:
            pwm_params.adcChannel = 0;
            break;
    }

    pwmInit(&pwm_params);
    core.numServos = pwm_params.numServos;

    // configure PWM/CPPM read function and max number of channels. spektrum or sbus below will override both of these, if enabled
    for (i = 0; i < RC_CHANS; i++)
        rcData[i] = 1502;
    rcReadRawFunc = pwmReadRawRC;

    if (feature(FEATURE_SERIALRX)) {
        switch (mcfg.serialrx_type) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                spektrumInit(UART_PORT_3, &rcReadRawFunc);
                break;
            case SERIALRX_SBUS:
                // Configure hardware inverter on PB2. If not available, this has no effect.
                actInvState(ON);
                sbusInit(UART_PORT_3, &rcReadRawFunc);
                break;
            case SERIALRX_SUMD:
                sumdInit(UART_PORT_3, &rcReadRawFunc);
                break;
            case SERIALRX_MSP:
                mspInit(&rcReadRawFunc);
                break;
            case SERIALRX_IBUS:
                ibusInit(UART_PORT_3, &rcReadRawFunc);
                break;
        }
    }

    // Optional GPS - available in both PPM, PWM and serialRX input mode, in PWM input, reduces number of available channels by 2.
    // gpsInit will return if FEATURE_GPS is not enabled.
    if (feature(FEATURE_GPS)) {
        gpsInit(UART_PORT_2, mcfg.gps_baudrate);
    }

#ifdef SONAR
    // sonar stuff only works with PPM
    if (feature(FEATURE_SONAR)) {
//      Sonar_init(hcsr04Init, SF_NONE);
//      Sonar_init(hcsr04Init, SF_AVERAGE);
        Sonar_init(hcsr04Init, SF_NOISE_CANCEL);
    }
#endif

    core.numAuxChannels = constrain((mcfg.rc_channel_count - 4), 4, 8);

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY))
        initTelemetry();
#endif

    if (mcfg.mixerConfiguration == MULTITYPE_GIMBAL)
        calibratingA = CALIBRATING_ACC_CYCLES;
    calibratingG = CALIBRATING_GYRO_CYCLES;
    calibratingB = CALIBRATING_BARO_CYCLES;             // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
    f.SMALL_ANGLE = 1;

    p_task = pifTaskManager_Add(TM_PERIOD_MS, 1, taskLoop, NULL, TRUE);         								// 1ms
    if (!p_task) FAIL;
    p_task->name = "Loop";

    if (mcfg.looptime) {
        g_task_compute_imu = pifTaskManager_Add(TM_PERIOD_US, mcfg.looptime, taskComputeImu, NULL, TRUE);
    }
    else {
        g_task_compute_imu = pifTaskManager_Add(TM_ALWAYS, 0, taskComputeImu, NULL, TRUE);
    }
    if (!g_task_compute_imu) FAIL;
    g_task_compute_imu->name = "IMU";
    g_task_compute_imu->disallow_yield_id = DISALLOW_YIELD_ID_I2C;

#ifdef MAG
    if (sensors(SENSOR_MAG)) {
        sensor_set.mag.p_task = pifTaskManager_Add(TM_PERIOD_MS, 100, taskMagGetAdc, NULL, TRUE);				// 100ms
        if (!sensor_set.mag.p_task) FAIL;
        sensor_set.mag.p_task->name = "Mag";
        sensor_set.mag.p_task->disallow_yield_id = DISALLOW_YIELD_ID_I2C;
    }
#endif

#ifdef BARO
    if (sensors(SENSOR_BARO)) {
        sensor_set.baro.p_task = pifTaskManager_Add(TM_EXTERNAL_ORDER, 0, taskGetEstimatedAltitude, NULL, FALSE);
        if (!sensor_set.baro.p_task) FAIL;
        sensor_set.baro.p_task->name = "Baro";
    }
#endif

#ifdef GPS
    g_task_gps = pifTaskManager_Add(TM_EXTERNAL_ORDER, 0, taskGpsNewData, NULL, FALSE);
    if (!g_task_gps) FAIL;
    g_task_gps->name = "GPS";
#endif

    p_task = pifTaskManager_Add(TM_PERIOD_MS, 50, taskLedState, NULL, TRUE);									// 50ms
    if (!p_task) FAIL;
    p_task->name = "Led";

	pifLog_Printf(LT_INFO, "Task=%d Timer1ms=%d\n", pifTaskManager_Count(),
			pifTimerManager_Count(&g_timer_1ms));
	return;

fail:
#ifdef __PIF_DEBUG__
	pifLog_Printf(LT_ERROR, "Error=%Xh Line=%u", pif_error, line);
#endif
	pifLog_SendAndExit();
}

// The loop function is called in an endless loop
void loop()
{
	pifTaskManager_Loop();
}
