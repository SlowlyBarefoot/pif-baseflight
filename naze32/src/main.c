/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "buzzer.h"
#include "mw.h"
#ifdef TELEMETRY
#include "telemetry_common.h"
#endif

#include "drv_adc.h"
#include "drv_adxl345.h"
#include "drv_ak8975.h"
#include "drv_bma280.h"
#include "drv_bmp280.h"
#include "drv_hcsr04.h"
#include "drv_hmc5883l.h"
#include "drv_i2c.h"
#include "drv_l3g4200d.h"
#include "drv_mma845x.h"
#include "drv_mpu.h"
#include "drv_pwm.h"
#include "drv_qmc5883.h"
#include "drv_serial.h"
#include "drv_softserial.h"
#include "drv_spi.h"
#include "drv_system.h"


// sync this with HardwareRevision in board.h
static const char *const hwNames[] = {
    "", "Naze 32", "Naze32 rev.5", "Naze32 SP", "Naze32 rev.6"
};

const char* g_board_name;

core_t core;
int hw_revision = 0;

static drv_mpu_config_t mpu_params;
static sensorDetect_t gyro_detect[] = { 
    { (sensorDetectFuncPtr)mpuDetect, (void*)&mpu_params }, 
#ifndef CJMCU
    { (sensorDetectFuncPtr)l3g4200dDetect, NULL },
#endif
    { NULL, NULL }
};

#ifndef CJMCU
    static drv_adxl345_config_t acc_params = {
        .useFifo = false,
        .dataRate = 800
    };
#endif
#if !defined(CJMCU) || defined(NAZE)
	static sensorDetect_t acc_detect[] = { 
#ifndef CJMCU
        { (sensorDetectFuncPtr)adxl345Detect, (void*)&acc_params }, 
#endif
#ifdef NAZE
        { (sensorDetectFuncPtr)mma8452Detect, NULL }, 
        { (sensorDetectFuncPtr)bma280Detect, NULL },
#endif
        { NULL, NULL }
    };
#else
	sensorDetect_t* acc_detect = NULL;
#endif

#ifdef BARO
	static sensorDetect_t baro_detect[] = { 
        { (sensorDetectFuncPtr)bmp280Detect, NULL }, 
//        { (sensorDetectFuncPtr)bmp085Detect, NULL }, 
//        { (sensorDetectFuncPtr)ms5611Detect, NULL },
        { NULL, NULL }
    };
#else    
	sensorDetect_t* baro_detect = NULL;
#endif

#ifdef MAG
	static sensorDetect_t mag_detect[] = { 
        { (sensorDetectFuncPtr)hmc5883lDetect, NULL }, 
        { (sensorDetectFuncPtr)ak8975detect, NULL },
        { (sensorDetectFuncPtr)qmc5883Detect, NULL },
        { NULL, NULL }
    };
#else    
	sensorDetect_t* msg_detect = NULL;
#endif

// from system_stm32f10x.c
void SetSysClock(bool overclock);

#ifdef USE_LAME_PRINTF
// gcc/GNU version
static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(core.mainport, c);
}
#else
// keil/armcc version
int fputc(int c, FILE *f)
{
    // let DMA catch up a bit when using set or dump, we're too fast.
    while (!isSerialTransmitBufferEmpty(core.mainport));
    serialWrite(core.mainport, c);
    return c;
}
#endif

static void featureDefault(void)
{
#ifdef CJMCU
    featureSet(FEATURE_PPM);
#else
    featureSet(FEATURE_VBAT);
#endif
}

int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;
    bool sensorsOK = false;
#ifdef SOFTSERIAL_LOOPBACK
    serialPort_t *loopbackPort1 = NULL;
    serialPort_t *loopbackPort2 = NULL;
#endif
    int spekUart = 0;
    PifTask* p_task;

    g_unique_id[0] = U_ID_0;
    g_unique_id[1] = U_ID_1;
    g_unique_id[2] = U_ID_2;

    g_crystal_clock = hse_value;
    g_core_clock = SystemCoreClock;

    pif_Init(micros);

    if (!pifTaskManager_Init(20)) goto bootloader;

    if (!pifTimerManager_Init(&g_timer_1ms, PIF_ID_AUTO, 1000, 3)) goto bootloader;		        // 1000us

    if (!buzzerInit()) goto bootloader;

    // make sure (at compile time) that config struct doesn't overflow allocated flash pages
    ct_assert(sizeof(mcfg) < CONFIG_SIZE);

    g_featureDefault = featureDefault;

    if (!initEEPROM(storageInit())) goto bootloader;
    if (!checkFirstTime(false)) goto bootloader;
    readEEPROM();

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(mcfg.emf_avoidance);

    // determine hardware revision
    if (hse_value == 8000000)
        hw_revision = NAZE32;
    else if (hse_value == 12000000)
        hw_revision = NAZE32_REV5;

    systemInit();
#ifdef USE_LAME_PRINTF
    init_printf(NULL, _putc);
#endif

    if (feature(FEATURE_SERIALRX)) {
        switch (mcfg.serialrx_type) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                // Spektrum satellite binding if enabled on startup.
                // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
                // The rest of Spektrum initialization will happen later - via spektrumInit()
                spekUart = spektrumBind(mcfg.spektrum_sat_on_flexport, mcfg.spektrum_sat_bind);
#ifndef HARDWARE_BIND_PLUG
                if (spekUart) {
                    // If we came here as a result of hard  reset (power up, with mcfg.spektrum_sat_bind set), then reset it back to zero and write config
                    // Don't reset if hardware bind plug is present
                    if (rccReadBkpDr() != BKP_SOFTRESET) {
                        mcfg.spektrum_sat_bind = 0;
                        writeEEPROM(1, true);
                    }
                }
#endif
                break;
        }
    }

    // sleep for 100ms
    pif_Delay1ms(100);

    activateConfig();

#ifndef CJMCU
    if (spiInit() == SPI_DEVICE_MPU && hw_revision == NAZE32_REV5)
        hw_revision = NAZE32_SP;
#endif

    if (hw_revision != NAZE32_SP) {
        i2cInit(I2C_DEVICE);

        if (!pifI2cPort_Init(&g_i2c_port, PIF_ID_AUTO, 5, 16)) goto bootloader;
        g_i2c_port.act_read = actI2cRead;
        g_i2c_port.act_write = actI2cWrite;
    }

    // configure power ADC
    if (mcfg.power_adc_channel > 0 && (mcfg.power_adc_channel == 1 || mcfg.power_adc_channel == 9 || mcfg.power_adc_channel == 5))
        adc_params.powerAdcChannel = mcfg.power_adc_channel;
    else {
        adc_params.powerAdcChannel = 0;
        mcfg.power_adc_channel = 0;
    }

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
    mpu_params.crystal_clock = g_crystal_clock;
    sensorsOK = sensorsAutodetect(gyro_detect, acc_detect, baro_detect, mag_detect);
    g_board_name = hwNames[hw_revision];

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

    serialInit(UART_PORT_1, mcfg.serial_baudrate, (hw_revision == NAZE32_SP && !mcfg.spektrum_sat_on_flexport) ? UART_PORT_3 : UART_PORT_NONE);

    g_task_compute_rc = pifTaskManager_Add(TM_EXTERNAL_ORDER, 0, taskComputeRc, NULL, FALSE);
    if (!g_task_compute_rc) goto bootloader;
    g_task_compute_rc->name = "RC";

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_FLYING_WING || mcfg.mixerConfiguration == MULTITYPE_CUSTOM_PLANE)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
    pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SERIALRX); // spektrum/sbus support uses UART too
    pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
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
    pwm_params.syncPWM = feature(FEATURE_SYNCPWM);
    pwm_params.fastPWM = feature(FEATURE_FASTPWM);
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
                spektrumInit(spekUart, &rcReadRawFunc);
                break;
            case SERIALRX_SBUS:
                // Configure hardware inverter on PB2. If not available, this has no effect.
                actInvState(ON);
                sbusInit(UART_PORT_2, &rcReadRawFunc);
                break;
            case SERIALRX_SUMD:
                sumdInit(UART_PORT_2, &rcReadRawFunc);
                break;
            case SERIALRX_MSP:
                mspInit(&rcReadRawFunc);
                break;
            case SERIALRX_IBUS:
                ibusInit(UART_PORT_2, &rcReadRawFunc);
                break;
        }
    }

#ifndef CJMCU
    // Optional GPS - available in both PPM, PWM and serialRX input mode, in PWM input, reduces number of available channels by 2.
    // gpsInit will return if FEATURE_GPS is not enabled.
    if (feature(FEATURE_GPS) && (!feature(FEATURE_SERIALRX) || mcfg.spektrum_sat_on_flexport)) {
        gpsInit(UART_PORT_2, mcfg.gps_baudrate);
    }
#endif

    if (feature(FEATURE_PPM)) {
#ifdef SONAR
        // sonar stuff only works with PPM
        if (feature(FEATURE_SONAR)) {
//          Sonar_init(hcsr04Init, SF_NONE);
//          Sonar_init(hcsr04Init, SF_AVERAGE);
            Sonar_init(hcsr04Init, SF_NOISE_CANCEL);
        }
#endif
    }

    core.numAuxChannels = constrain((mcfg.rc_channel_count - 4), 4, 8);

#ifndef CJMCU
    if (feature(FEATURE_SOFTSERIAL)) {
        //mcfg.softserial_baudrate = 19200; // Uncomment to override config value

        setupSoftSerialPrimary(mcfg.softserial_baudrate, mcfg.softserial_1_inverted);
        setupSoftSerialSecondary(mcfg.softserial_2_inverted);

#ifdef SOFTSERIAL_LOOPBACK
        loopbackPort1 = (serialPort_t *)(&softSerialPorts[0]));
        serialPrint(loopbackPort1, "SOFTSERIAL 1 - LOOPBACK ENABLED\r\n");

        loopbackPort2 = (serialPort_t *)(&softSerialPorts[1]));
        serialPrint(loopbackPort2, "SOFTSERIAL 2 - LOOPBACK ENABLED\r\n");
#endif
        //core.mainport = (serialPort_t*)&(softSerialPorts[0]); // Uncomment to switch the main port to use softserial.
    }

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY))
        initTelemetry();
#endif
#endif

    if (mcfg.mixerConfiguration == MULTITYPE_GIMBAL)
        calibratingA = CALIBRATING_ACC_CYCLES;
    calibratingG = CALIBRATING_GYRO_CYCLES;
    calibratingB = CALIBRATING_BARO_CYCLES;             // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
    f.SMALL_ANGLE = 1;

    p_task = pifTaskManager_Add(TM_PERIOD_MS, 10, taskLoop, NULL, TRUE);                            // 50ms
    if (!p_task) goto bootloader;
    p_task->name = "Loop";

    if (mcfg.looptime) {
        g_task_compute_imu = pifTaskManager_Add(TM_PERIOD_US, mcfg.looptime, taskComputeImu, NULL, TRUE);
    }
    else {
        g_task_compute_imu = pifTaskManager_Add(TM_RATIO, 100, taskComputeImu, NULL, TRUE);	        // 100%
    }
    if (!g_task_compute_imu) goto bootloader;
    g_task_compute_imu->name = "IMU";
    g_task_compute_imu->disallow_yield_id = DISALLOW_YIELD_ID_I2C;

#ifdef MAG
    if (sensors(SENSOR_MAG)) {
        sensor_set.mag.p_task = pifTaskManager_Add(TM_PERIOD_MS, 100, taskMagGetAdc, NULL, TRUE); // 100ms
        if (!sensor_set.mag.p_task) goto bootloader;
        sensor_set.mag.p_task->name = "Mag";
        sensor_set.mag.p_task->disallow_yield_id = DISALLOW_YIELD_ID_I2C;
    }
#endif

#ifdef BARO
    if (sensors(SENSOR_BARO)) {
        sensor_set.baro.p_task = pifTaskManager_Add(TM_EXTERNAL_ORDER, 0, taskGetEstimatedAltitude, NULL, FALSE);
        if (!sensor_set.baro.p_task) goto bootloader;
        sensor_set.baro.p_task->name = "Baro";
    }
#endif

#ifdef GPS
    g_task_gps = pifTaskManager_Add(TM_EXTERNAL_ORDER, 0, taskGpsNewData, NULL, FALSE);
    if (!g_task_gps) goto bootloader;
    g_task_gps->name = "GPS";
#endif

    p_task = pifTaskManager_Add(TM_PERIOD_MS, 50, taskLedState, NULL, TRUE);                        // 50ms
    if (!p_task) goto bootloader;
    p_task->name = "Led";

    // loopy
    while (1) {
        pifTaskManager_Loop();
#ifdef SOFTSERIAL_LOOPBACK
        if (loopbackPort1) {
            while (serialTotalBytesWaiting(loopbackPort1)) {
                uint8_t b = serialRead(loopbackPort1);
                serialWrite(loopbackPort1, b);
                //serialWrite(core.mainport, 0x01);
                //serialWrite(core.mainport, b);
            };
        }

        if (loopbackPort2) {
            while (serialTotalBytesWaiting(loopbackPort2)) {
                serialRead(loopbackPort2);
            };
        }
#endif
    }

bootloader:
    systemReset(true);
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(mcfg.mincommand);
    while (1);
}
