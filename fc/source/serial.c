/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "cli.h"
#include "mw.h"
#ifdef TELEMETRY
#include "telemetry_common.h"
#endif

#ifndef __PIF_NO_LOG__
	#include "core/pif_log.h"
#endif
#include "protocol/pif_msp.h"

// Multiwii Serial Protocol 0
#define MSP_VERSION              4
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)
#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FW_FLAPS                ((uint32_t)1 << 3)

#define MSP_IDENT                100    //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         8 servos
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan and more
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         altitude, variometer
#define MSP_ANALOG               110    //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113    //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114    //out message         powermeter trig
#define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119    //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120    //out message         Servo settings
#define MSP_NAV_STATUS           121    //out message         Returns navigation status
#define MSP_NAV_CONFIG           122    //out message         Returns navigation parameters
#define MSP_FW_CONFIG            123    //out message         Returns parameters specific to Flying Wing mode

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203    //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
#define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_SET_WP               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210    //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211    //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212    //in message          Servo settings
#define MSP_SET_MOTOR            214    //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215    //in message          Sets nav config parameters - write to the eeprom
#define MSP_SET_FW_CONFIG        216    //in message          Sets parameters specific to Flying Wing mode

// #define MSP_BIND                 240    //in message          no param

#define MSP_EEPROM_WRITE         250    //in message          no param

#define MSP_DEBUGMSG             253    //out message         debug string buffer
#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4

// Additional commands that are not compatible with MultiWii
#define MSP_UID                  160    //out message         Unique device ID
#define MSP_ACC_TRIM             240    //out message         get acc angle trim values
#define MSP_SET_ACC_TRIM         239    //in message          set acc angle trim values
#define MSP_GPSSVINFO            164    //out message         get Signal Strength (only U-Blox)
#define MSP_GPSDEBUGINFO         166    //out message         get GPS debugging data (only U-Blox)
#define MSP_SERVOMIX_CONF        241    //out message         Returns servo mixer configuration
#define MSP_SET_SERVOMIX_CONF    242    //in message          Sets servo mixer configuration

// Additional private MSP for baseflight configurator
#define MSP_RCMAP                64     //out message         get channel map (also returns number of channels total)
#define MSP_SET_RCMAP            65     //in message          set rc map, numchannels to set comes from MSP_RCMAP
#define MSP_CONFIG               66     //out message         baseflight-specific settings that aren't covered elsewhere
#define MSP_SET_CONFIG           67     //in message          baseflight-specific settings save
#define MSP_REBOOT               68     //in message          reboot settings
#define MSP_BUILDINFO            69     //out message         build date as well as some space for future expansion

#define INBUF_SIZE 128
#define MAX_SERIAL_INPUTS 8

typedef struct box_t {
    const uint8_t boxIndex;         // this is from boxnames enum
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      // Permanent ID for reporting to GUI
} box_t;

static const box_t boxes[] = {
    { BOXARM, "ARM;", 0 },
    { BOXANGLE, "ANGLE;", 1 },
    { BOXHORIZON, "HORIZON;", 2 },
    { BOXBARO, "BARO;", 3 },
    { BOXVARIO, "VARIO;", 4 },
    { BOXMAG, "MAG;", 5 },
    { BOXHEADFREE, "HEADFREE;", 6 },
    { BOXHEADADJ, "HEADADJ;", 7 },
    { BOXCAMSTAB, "CAMSTAB;", 8 },
    { BOXCAMTRIG, "CAMTRIG;", 9 },
    { BOXGPSHOME, "GPS HOME;", 10 },
    { BOXGPSHOLD, "GPS HOLD;", 11 },
    { BOXPASSTHRU, "PASSTHRU;", 12 },
    { BOXBEEPERON, "BEEPER;", 13 },
    { BOXLEDMAX, "LEDMAX;", 14 },
    { BOXLEDLOW, "LEDLOW;", 15 },
    { BOXLLIGHTS, "LLIGHTS;", 16 },
    { BOXCALIB, "CALIB;", 17 },
    { BOXGOV, "GOVERNOR;", 18 },
    { BOXOSD, "OSD SW;", 19 },
    { BOXTELEMETRY, "TELEMETRY;", 20 },
    { BOXSERVO1, "SERVO1;", 21 },
    { BOXSERVO2, "SERVO2;", 22 },
    { BOXSERVO3, "SERVO3;", 23 },
    { BOXGCRUISE, "CRUISE;", 24 },
    { CHECKBOXITEMS, NULL, 0xFF }
};

// this is calculated at startup based on enabled features.
static uint8_t availableBoxes[CHECKBOXITEMS];
// this is the number of filled indexes in above array
static uint8_t numberBoxItems = 0;
// from mixer.c
extern int16_t motor_disarmed[MAX_MOTORS];
// cause reboot after MSP processing complete
static bool pendReboot = false;

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

typedef  struct mspPortState_t {
    serialPort_t *port;
    PifMsp pif_msp;
    uint8_t* p_rx_packet;
    uint8_t tx_packet[128], *p_tx_packet;
} mspPortState_t;

static mspPortState_t ports[2];

static BOOL s_change_cli = FALSE;

static void evtMspReceive(PifMsp* p_owner, PifMspPacket* p_packet);
static void evtMspOtherPacket(PifMsp* p_owner, uint8_t data);

static uint8_t _Read8(mspPortState_t* p_port)
{
	uint8_t data = p_port->p_rx_packet[0];
	p_port->p_rx_packet++;
	return data;
}

static uint16_t _Read16(mspPortState_t* p_port)
{
	uint16_t data = p_port->p_rx_packet[0] + (p_port->p_rx_packet[1] << 8);
	p_port->p_rx_packet += 2;
	return data;
}

static uint16_t _Read32(mspPortState_t* p_port)
{
	uint32_t data = p_port->p_rx_packet[0] + (p_port->p_rx_packet[1] << 8) +
			((uint32_t)p_port->p_rx_packet[2] << 16) + ((uint32_t)p_port->p_rx_packet[3] << 24);
	p_port->p_rx_packet += 4;
	return data;
}

static void _Serialize8(mspPortState_t* p_port, uint8_t a)
{
    p_port->p_tx_packet[0] = a;
    p_port->p_tx_packet++;
}

static void _Serialize16(mspPortState_t* p_port, int16_t a)
{
    p_port->p_tx_packet[0] = a & 0xFF;
    p_port->p_tx_packet[1] = (a >> 8) & 0xFF;
    p_port->p_tx_packet += 2;
}

static void _Serialize32(mspPortState_t* p_port, uint32_t a)
{
    p_port->p_tx_packet[0] = a & 0xFF;
    p_port->p_tx_packet[1] = (a >> 8) & 0xFF;
    p_port->p_tx_packet[2] = (a >> 16) & 0xFF;
    p_port->p_tx_packet[3] = (a >> 24) & 0xFF;
    p_port->p_tx_packet += 4;
}

static void _SerializeStruct(mspPortState_t* p_port, uint8_t *cb, uint8_t siz)
{
    while (siz--)
        *p_port->p_tx_packet++ = *cb++;
}

static void _SerializeNames(mspPortState_t* p_port, const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        *p_port->p_tx_packet++ = *c;
}

static int _SerializeBoxNamesReply(mspPortState_t* p_port)
{
    int i, idx, j, flag = 1, count = 0, len;

reset:
    // in first run of the loop, we grab total size of junk to be sent
    // then come back and actually send it
    for (i = 0; i < numberBoxItems; i++) {
        idx = availableBoxes[i];
        len = strlen(boxes[idx].boxName);
        if (flag) {
            count += len;
        } else {
            for (j = 0; j < len; j++)
                *p_port->p_tx_packet++ = boxes[idx].boxName[j];
        }
    }

    if (flag) {
        flag = 0;
        goto reset;
    }
    return count;
}

void serialInit(uint8_t port, uint32_t baudrate, uint8_t flexport)
{
    int idx;

    core.mainport = uartOpen(port, baudrate, MODE_RXTX);
    ports[0].port = core.mainport;

    if (!pifMsp_Init(&ports[0].pif_msp, &g_timer_1ms, PIF_ID_MSP(0))) return;
    ports[0].pif_msp.evt_receive = evtMspReceive;
    ports[0].pif_msp.evt_other_packet = evtMspOtherPacket;
    pifMsp_AttachComm(&ports[0].pif_msp, &core.mainport->comm);

    // additional telemetry port available only if spektrum sat isn't already assigned there
    if (flexport) {
        core.flexport = uartOpen(flexport, baudrate, MODE_RXTX);
        ports[1].port = core.flexport;

        if (!pifMsp_Init(&ports[1].pif_msp, &g_timer_1ms, PIF_ID_MSP(1))) return;
        ports[1].pif_msp.evt_receive = evtMspReceive;
        ports[1].pif_msp.evt_other_packet = evtMspOtherPacket;
        pifMsp_AttachComm(&ports[1].pif_msp, &core.flexport->comm);
    }

    // calculate used boxes based on features and fill availableBoxes[] array
    memset(availableBoxes, 0xFF, sizeof(availableBoxes));

    idx = 0;
    availableBoxes[idx++] = BOXARM;
    if (sensors(SENSOR_ACC)) {
        availableBoxes[idx++] = BOXANGLE;
        availableBoxes[idx++] = BOXHORIZON;
    }
    if (sensors(SENSOR_BARO)) {
        availableBoxes[idx++] = BOXBARO;
        if (feature(FEATURE_VARIO))
            availableBoxes[idx++] = BOXVARIO;
    }
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        availableBoxes[idx++] = BOXMAG;
        availableBoxes[idx++] = BOXHEADFREE;
        availableBoxes[idx++] = BOXHEADADJ;
    }
    if (feature(FEATURE_SERVO_TILT))
        availableBoxes[idx++] = BOXCAMSTAB;
    if (feature(FEATURE_GPS)) {
        availableBoxes[idx++] = BOXGPSHOME;
        availableBoxes[idx++] = BOXGPSHOLD;
    }
    if (f.FIXED_WING) {
        availableBoxes[idx++] = BOXPASSTHRU;
        availableBoxes[idx++] = BOXGCRUISE;
    }
    availableBoxes[idx++] = BOXBEEPERON;
    if (feature(FEATURE_INFLIGHT_ACC_CAL))
        availableBoxes[idx++] = BOXCALIB;
    availableBoxes[idx++] = BOXOSD;
    if (feature(FEATURE_TELEMETRY && mcfg.telemetry_switch))
        availableBoxes[idx++] = BOXTELEMETRY;
    if (mcfg.mixerConfiguration == MULTITYPE_CUSTOM_PLANE) {
        availableBoxes[idx++] = BOXSERVO1;
        availableBoxes[idx++] = BOXSERVO2;
        availableBoxes[idx++] = BOXSERVO3;
    }

    numberBoxItems = idx;
}

void evtMspReceive(PifMsp* p_owner, PifMspPacket* p_packet)
{
    uint32_t i, j, tmp, junk;
#ifdef GPS
    uint8_t wp_no;
    int32_t lat = 0, lon = 0, alt = 0;
#endif
    const char *build = __DATE__;
    mspPortState_t* p_port = &ports[PIF_ID_MSP_2_IDX(p_owner->_id)];

    p_port->p_rx_packet = p_packet->p_data;
    p_port->p_tx_packet = p_port->tx_packet;
    switch (p_packet->command) {
    case MSP_SET_RAW_RC:
        for (i = 0; i < 8; i++)
            rcData[i] = _Read16(p_port);
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        mspFrameRecieve();
        break;
        
    case MSP_SET_ACC_TRIM:
        cfg.angleTrim[PITCH] = _Read16(p_port);
        cfg.angleTrim[ROLL]  = _Read16(p_port);
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

#ifdef GPS
    case MSP_SET_RAW_GPS:
        f.GPS_FIX = _Read8(p_port);
        GPS_numSat = _Read8(p_port);
        GPS_coord[LAT] = _Read32(p_port);
        GPS_coord[LON] = _Read32(p_port);
        GPS_altitude = _Read16(p_port);
        GPS_speed = _Read16(p_port);
        GPS_update |= 2;        // New data signalisation to GPS functions
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;
#endif

    case MSP_SET_PID:
        for (i = 0; i < PIDITEMS; i++) {
            cfg.P8[i] = _Read8(p_port);
            cfg.I8[i] = _Read8(p_port);
            cfg.D8[i] = _Read8(p_port);
#ifndef __PIF_NO_LOG__
            pifLog_Printf(LT_INFO, "S-PID:%d P=%u I=%u D=%u", i, cfg.P8[i], cfg.I8[i], cfg.D8[i]);
#endif
        }
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_SET_BOX:
        for (i = 0; i < numberBoxItems; i++)
            cfg.activate[availableBoxes[i]] = _Read16(p_port);
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_SET_RC_TUNING:
        cfg.rcRate8 = _Read8(p_port);
        cfg.rcExpo8 = _Read8(p_port);
        _Read8(p_port); // Legacy pitch-roll rate, read but not set.
        cfg.yawRate = _Read8(p_port);
        cfg.dynThrPID = _Read8(p_port);
        cfg.thrMid8 = _Read8(p_port);
        cfg.thrExpo8 = _Read8(p_port);
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_SET_MISC:
        tmp = _Read16(p_port);
        // sanity check
        if (tmp < 1600 && tmp > 1400)
            mcfg.midrc = tmp;
        mcfg.minthrottle = _Read16(p_port);
        mcfg.maxthrottle = _Read16(p_port);
        mcfg.mincommand = _Read16(p_port);
        cfg.failsafe_throttle = _Read16(p_port);
        mcfg.gps_type = _Read8(p_port);
        mcfg.gps_baudrate = _Read8(p_port);
        mcfg.gps_ubx_sbas = _Read8(p_port);
        mcfg.multiwiicurrentoutput = _Read8(p_port);
        mcfg.rssi_aux_channel = _Read8(p_port);
        _Read8(p_port);
        cfg.mag_declination = _Read16(p_port) * 10;
        mcfg.vbatscale = _Read8(p_port);           // actual vbatscale as intended
        mcfg.vbatmincellvoltage = _Read8(p_port);  // vbatlevel_warn1 in MWC2.3 GUI
        mcfg.vbatmaxcellvoltage = _Read8(p_port);  // vbatlevel_warn2 in MWC2.3 GUI
        mcfg.vbatwarningcellvoltage = _Read8(p_port); // vbatlevel when buzzer starts to alert
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_SET_MOTOR:
        for (i = 0; i < 8; i++)
            motor_disarmed[i] = _Read16(p_port);
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_SELECT_SETTING:
        if (!f.ARMED) {
            mcfg.current_profile = _Read8(p_port);
            if (mcfg.current_profile > 2)
                mcfg.current_profile = 0;
            // this writes new profile index and re-reads it
            writeEEPROM(0, false);
        }
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_SET_HEAD:
        magHold = _Read16(p_port);
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_IDENT:
        _Serialize8(p_port, VERSION);                    // multiwii version
        _Serialize8(p_port, mcfg.mixerConfiguration);    // type of multicopter
        _Serialize8(p_port, MSP_VERSION);                // MultiWii Serial Protocol Version
        _Serialize32(p_port, CAP_PLATFORM_32BIT | CAP_BASEFLIGHT_CONFIG | CAP_DYNBALANCE | CAP_FW_FLAPS); // "capability"
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 7);
        break;

    case MSP_STATUS:
        _Serialize16(p_port, cycleTime);
        _Serialize16(p_port, g_i2c_port.error_count);
        _Serialize16(p_port, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        // OK, so you waste all the fucking time to have BOXNAMES and BOXINDEXES etc, and then you go ahead and serialize enabled shit simply by stuffing all
        // the bits in order, instead of setting the enabled bits based on BOXINDEX. WHERE IS THE FUCKING LOGIC IN THIS, FUCKWADS.
        // Serialize the boxes in the order we delivered them, until multiwii retards fix their shit
        junk = 0;
        tmp = f.ANGLE_MODE << BOXANGLE | f.HORIZON_MODE << BOXHORIZON |
                f.BARO_MODE << BOXBARO | f.MAG_MODE << BOXMAG | f.HEADFREE_MODE << BOXHEADFREE | rcOptions[BOXHEADADJ] << BOXHEADADJ |
                rcOptions[BOXCAMSTAB] << BOXCAMSTAB | rcOptions[BOXCAMTRIG] << BOXCAMTRIG |
                f.GPS_HOME_MODE << BOXGPSHOME | f.GPS_HOLD_MODE << BOXGPSHOLD |
                f.CRUISE_MODE << BOXGCRUISE | f.PASSTHRU_MODE << BOXPASSTHRU |
                rcOptions[BOXBEEPERON] << BOXBEEPERON |
                rcOptions[BOXLEDMAX] << BOXLEDMAX |
                rcOptions[BOXLLIGHTS] << BOXLLIGHTS |
                rcOptions[BOXVARIO] << BOXVARIO |
                rcOptions[BOXCALIB] << BOXCALIB |
                rcOptions[BOXGOV] << BOXGOV |
                rcOptions[BOXOSD] << BOXOSD |
                rcOptions[BOXTELEMETRY] << BOXTELEMETRY |
                rcOptions[BOXSERVO1] << BOXSERVO1 |
                rcOptions[BOXSERVO2] << BOXSERVO2 |
                rcOptions[BOXSERVO3] << BOXSERVO3 |
                f.ARMED << BOXARM;
        for (i = 0; i < numberBoxItems; i++) {
            int flag = (tmp & (1 << availableBoxes[i]));
            if (flag)
                junk |= 1 << i;
        }
        _Serialize32(p_port, junk);
        _Serialize8(p_port, mcfg.current_profile);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 11);
        break;

    case MSP_RAW_IMU:
        // Retarded hack until multiwiidorks start using real units for sensor data
        if (acc_1G > 1024) {
            for (i = 0; i < 3; i++)
                _Serialize16(p_port, accSmooth[i] / 8);
        } else {
            for (i = 0; i < 3; i++)
                _Serialize16(p_port, accSmooth[i]);
        }
        for (i = 0; i < 3; i++)
            _Serialize16(p_port, gyroData[i]);
        for (i = 0; i < 3; i++)
            _Serialize16(p_port, magADC[i]);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 18);
        break;

    case MSP_SERVO:
        _SerializeStruct(p_port, (uint8_t *)&servo, 16);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 16);
        break;

    case MSP_SERVO_CONF:
        for (i = 0; i < MAX_SERVOS; i++) {
            _Serialize16(p_port, cfg.servoConf[i].min);
            _Serialize16(p_port, cfg.servoConf[i].max);
            _Serialize16(p_port, cfg.servoConf[i].middle);
            _Serialize8(p_port, cfg.servoConf[i].rate);
            _Serialize16(p_port, cfg.servoConf[i].direction);
        }
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, MAX_SERVOS * 9);
        break;

    case MSP_SET_SERVO_CONF:
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        for (i = 0; i < MAX_SERVOS; i++) {
            cfg.servoConf[i].min = _Read16(p_port);
            cfg.servoConf[i].max = _Read16(p_port);
            cfg.servoConf[i].middle = _Read16(p_port);
            cfg.servoConf[i].rate = _Read8(p_port);
            cfg.servoConf[i].direction = _Read16(p_port);
        }
        break;

    case MSP_SERVOMIX_CONF:
        for (i = 0; i < MAX_SERVO_RULES; i++) {
            _Serialize8(p_port, mcfg.customServoMixer[i].targetChannel);
            _Serialize8(p_port, mcfg.customServoMixer[i].fromChannel);
            _Serialize8(p_port, mcfg.customServoMixer[i].rate);
            _Serialize8(p_port, mcfg.customServoMixer[i].speed);
            _Serialize8(p_port, mcfg.customServoMixer[i].min);
            _Serialize8(p_port, mcfg.customServoMixer[i].max);
            _Serialize8(p_port, mcfg.customServoMixer[i].box);
        }
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, MAX_SERVO_RULES * sizeof(servoMixer_t));
        break;

    case MSP_SET_SERVOMIX_CONF:
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        for (i = 0; i < MAX_SERVO_RULES; i++) {
            mcfg.customServoMixer[i].targetChannel = _Read8(p_port);
            mcfg.customServoMixer[i].fromChannel = _Read8(p_port);
            mcfg.customServoMixer[i].rate = _Read8(p_port);
            mcfg.customServoMixer[i].speed = _Read8(p_port);
            mcfg.customServoMixer[i].min = _Read8(p_port);
            mcfg.customServoMixer[i].max = _Read8(p_port);
            mcfg.customServoMixer[i].box = _Read8(p_port);
        }
        loadCustomServoMixer();
        break;

    case MSP_FW_CONFIG:
        _Serialize8(p_port, mcfg.fw_althold_dir);
        // _Serialize8(p_port, cfg.fw_vector_thrust); // Future Gui setting?
        _Serialize16(p_port, cfg.fw_gps_maxcorr);
        _Serialize16(p_port, cfg.fw_gps_rudder);
        _Serialize16(p_port, cfg.fw_gps_maxclimb);
        _Serialize16(p_port, cfg.fw_gps_maxdive);
        _Serialize16(p_port, cfg.fw_climb_throttle);
        _Serialize16(p_port, cfg.fw_cruise_throttle);
        _Serialize16(p_port, cfg.fw_idle_throttle);
        _Serialize16(p_port, cfg.fw_scaler_throttle);
        _Serialize32(p_port, cfg.fw_roll_comp); // Float is Not compatible with Gui. Change to _Serialize8
        _Serialize8(p_port, cfg.fw_rth_alt);
        // next added for future use
        _Serialize32(p_port, 0);
        _Serialize32(p_port, 0);
        _Serialize32(p_port, 0);
        _Serialize32(p_port, 0);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 38);
        break;

    case MSP_SET_FW_CONFIG:
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        mcfg.fw_althold_dir = _Read8(p_port);
        // cfg.fw_vector_thrust = _Read8(p_port); // Future Gui setting?
        cfg.fw_gps_maxcorr = _Read16(p_port);
        cfg.fw_gps_rudder = _Read16(p_port);
        cfg.fw_gps_maxclimb = _Read16(p_port);
        cfg.fw_gps_maxdive = _Read16(p_port);
        cfg.fw_climb_throttle = _Read16(p_port);
        cfg.fw_cruise_throttle = _Read16(p_port);
        cfg.fw_idle_throttle = _Read16(p_port);
        cfg.fw_scaler_throttle = _Read16(p_port);
        //cfg.fw_gps_maxdive = _Read32(p_port);// Wrong when using float in MSP!... Change to _Read8
        _Read32(p_port); // Just read and skip
        cfg.fw_rth_alt = _Read8(p_port);
        // next added for future use
        _Read32(p_port);
        _Read32(p_port);
        _Read32(p_port);
        _Read32(p_port);
        break;

    case MSP_MOTOR:
        _SerializeStruct(p_port, (uint8_t *)&motor, 16);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 16);
        break;

    case MSP_RC:
        for (i = 0; i < 8; i++)
            _Serialize16(p_port, rcData[i]);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 16);
        break;

    case MSP_RAW_GPS:
        _Serialize8(p_port, f.GPS_FIX);
        _Serialize8(p_port, GPS_numSat);
        _Serialize32(p_port, GPS_coord[LAT]);
        _Serialize32(p_port, GPS_coord[LON]);
        _Serialize16(p_port, GPS_altitude);
        _Serialize16(p_port, GPS_speed);
        _Serialize16(p_port, GPS_ground_course);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 16);
        break;

    case MSP_COMP_GPS:
        _Serialize16(p_port, GPS_distanceToHome);
        _Serialize16(p_port, GPS_directionToHome);
        _Serialize8(p_port, GPS_update & 1);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 5);
        break;

    case MSP_ATTITUDE:
        for (i = 0; i < 2; i++)
            _Serialize16(p_port, angle[i]);
        _Serialize16(p_port, heading);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 6);
        break;

    case MSP_ALTITUDE:
        _Serialize32(p_port, EstAlt);
        _Serialize16(p_port, vario);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 6);
        break;

    case MSP_ANALOG:
        _Serialize8(p_port, (uint8_t)constrain((int16_t)vbat, 0, 255));
        _Serialize16(p_port, (uint16_t)constrain(mAhdrawn, 0, 0xFFFF)); // milliamphours drawn from battery
        _Serialize16(p_port, rssi);
        if (mcfg.multiwiicurrentoutput)
            _Serialize16(p_port, (uint16_t)constrain((abs(amperage) * 10), 0, 0xFFFF)); // send amperage in 0.001 A steps
        else
            _Serialize16(p_port, (uint16_t)constrain(abs(amperage), 0, 0xFFFF)); // send amperage in 0.01 A steps
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 7);
        break;

    case MSP_RC_TUNING:
        _Serialize8(p_port, cfg.rcRate8);
        _Serialize8(p_port, cfg.rcExpo8);
        _Serialize8(p_port, cfg.rollPitchRate[0]); // here for legacy support
        _Serialize8(p_port, cfg.yawRate);
        _Serialize8(p_port, cfg.dynThrPID);
        _Serialize8(p_port, cfg.thrMid8);
        _Serialize8(p_port, cfg.thrExpo8);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 7);
        break;

    case MSP_PID:
        for (i = 0; i < PIDITEMS; i++) {
            _Serialize8(p_port, cfg.P8[i]);
            _Serialize8(p_port, cfg.I8[i]);
            _Serialize8(p_port, cfg.D8[i]);
#ifndef __PIF_NO_LOG__
            pifLog_Printf(LT_INFO, "PID:%d P=%u I=%u D=%u", i, cfg.P8[i], cfg.I8[i], cfg.D8[i]);
#endif
        }
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 3 * PIDITEMS);
        break;

    case MSP_PIDNAMES:
        _SerializeNames(p_port, pidnames);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, sizeof(pidnames) - 1);
        break;

    case MSP_BOX:
        for (i = 0; i < numberBoxItems; i++)
            _Serialize16(p_port, cfg.activate[availableBoxes[i]]);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 2 * numberBoxItems);
        break;

    case MSP_BOXNAMES:
        tmp = _SerializeBoxNamesReply(p_port);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, tmp);
        break;

    case MSP_BOXIDS:
        for (i = 0; i < numberBoxItems; i++) {
            for  (j = 0; j < CHECKBOXITEMS; j++) {
                if (boxes[j].permanentId == availableBoxes[i]) {
                    _Serialize8(p_port, boxes[j].permanentId);
                    break;
                }
            }
        }
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, numberBoxItems);
        break;

    case MSP_MISC:
        _Serialize16(p_port, mcfg.midrc);
        _Serialize16(p_port, mcfg.minthrottle);
        _Serialize16(p_port, mcfg.maxthrottle);
        _Serialize16(p_port, mcfg.mincommand);
        _Serialize16(p_port, cfg.failsafe_throttle);
        _Serialize8(p_port, mcfg.gps_type);
        _Serialize8(p_port, mcfg.gps_baudrate);
        _Serialize8(p_port, mcfg.gps_ubx_sbas);
        _Serialize8(p_port, mcfg.multiwiicurrentoutput);
        _Serialize8(p_port, mcfg.rssi_aux_channel);
        _Serialize8(p_port, 0);
        _Serialize16(p_port, cfg.mag_declination / 10); // TODO check this shit
        _Serialize8(p_port, mcfg.vbatscale);
        _Serialize8(p_port, mcfg.vbatmincellvoltage);
        _Serialize8(p_port, mcfg.vbatmaxcellvoltage);
        _Serialize8(p_port, mcfg.vbatwarningcellvoltage);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 2 * 6 + 4 + 2 + 4);
        break;

    case MSP_MOTOR_PINS:
        for (i = 0; i < 8; i++)
            _Serialize8(p_port, i + 1);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 8);
        break;

#ifdef GPS
    case MSP_WP:
        wp_no = _Read8(p_port);    // get the wp number
        if (wp_no == 0) {
            lat = GPS_home[LAT];
            lon = GPS_home[LON];
        } else if (wp_no == 16) {
            lat = GPS_hold[LAT];
            lon = GPS_hold[LON];
        }
        _Serialize8(p_port, wp_no);
        _Serialize32(p_port, lat);
        _Serialize32(p_port, lon);
        _Serialize32(p_port, AltHold);           // altitude (cm) will come here -- temporary implementation to test feature with apps
        _Serialize16(p_port, 0);                 // heading  will come here (deg)
        _Serialize16(p_port, 0);                 // time to stay (ms) will come here
        _Serialize8(p_port, 0);                  // nav flag will come here
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 18);
        break;

    case MSP_SET_WP:
        wp_no = _Read8(p_port);    //get the wp number
        lat = _Read32(p_port);
        lon = _Read32(p_port);
        alt = _Read32(p_port);     // to set altitude (cm)
        _Read16(p_port);           // future: to set heading (deg)
        _Read16(p_port);           // future: to set time to stay (ms)
        _Read8(p_port);            // future: to set nav flag
        if (wp_no == 0) {
            GPS_home[LAT] = lat;
            GPS_home[LON] = lon;
            f.GPS_HOME_MODE = 0;        // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
            f.GPS_FIX_HOME = 1;
            if (alt != 0)
                AltHold = alt;          // temporary implementation to test feature with apps
        } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
            GPS_hold[LAT] = lat;
            GPS_hold[LON] = lon;
            if (alt != 0)
                AltHold = alt;          // temporary implementation to test feature with apps
            nav_mode = NAV_MODE_WP;
            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
        }
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;
#endif  // GPS

    case MSP_RESET_CONF:
        if (!f.ARMED)
            checkFirstTime(true);
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_ACC_CALIBRATION:
        if (!f.ARMED)
            calibratingA = CALIBRATING_ACC_CYCLES;
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_MAG_CALIBRATION:
        if (!f.ARMED)
            f.CALIBRATE_MAG = 1;
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        break;

    case MSP_EEPROM_WRITE:
        if (f.ARMED) {
            pifMsp_MakeError(p_owner, p_packet);
        } else {
            writeEEPROM(0, true);
            pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        }
        break;

    case MSP_DEBUG:
        // make use of this crap, output some useful QA statistics
        debug[3] = ((g_crystal_clock / 1000000) * 1000) + (g_core_clock / 1000000);         // XX0YY [crystal clock : core clock]
        for (i = 0; i < 4; i++)
            _Serialize16(p_port, debug[i]);      // 4 variables are here for general monitoring purpose
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 8);
        break;

    // Additional commands that are not compatible with MultiWii
    case MSP_ACC_TRIM:
        _Serialize16(p_port, cfg.angleTrim[PITCH]);
        _Serialize16(p_port, cfg.angleTrim[ROLL]);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 4);
        break;

    case MSP_UID:
        _Serialize32(p_port, g_unique_id[0]);
        _Serialize32(p_port, g_unique_id[1]);
        _Serialize32(p_port, g_unique_id[2]);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 12);
        break;

#ifdef GPS
    case MSP_GPSSVINFO:
        _Serialize8(p_port, GPS_numCh);
        for (i = 0; i < GPS_numCh; i++) {
            _Serialize8(p_port, GPS_svinfo_chn[i]);
            _Serialize8(p_port, GPS_svinfo_svid[i]);
            _Serialize8(p_port, GPS_svinfo_quality[i]);
            _Serialize8(p_port, GPS_svinfo_cno[i]);
        }
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 1 + (GPS_numCh * 4));
        // Poll new SVINFO from GPS
        gpsPollSvinfo();
        break;
    case MSP_GPSDEBUGINFO:
        if (sensors(SENSOR_GPS)) {
            _Serialize32(p_port, GPS_update_rate[1] - GPS_update_rate[0]);
            _Serialize32(p_port, GPS_svinfo_rate[1] - GPS_svinfo_rate[0]);
        } else {
            _Serialize32(p_port, 0);
            _Serialize32(p_port, 0);
        }
        _Serialize32(p_port, GPS_HorizontalAcc);
        _Serialize32(p_port, GPS_VerticalAcc);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 16);
        break;
#endif  // GPS

    case MSP_SET_CONFIG:
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        mcfg.mixerConfiguration = _Read8(p_port); // multitype
        featureClearAll();
        featureSet(_Read32(p_port)); // features bitmap
        mcfg.serialrx_type = _Read8(p_port); // serialrx_type
        mcfg.board_align_roll = _Read16(p_port); // board_align_roll
        mcfg.board_align_pitch = _Read16(p_port); // board_align_pitch
        mcfg.board_align_yaw = _Read16(p_port); // board_align_yaw
        mcfg.currentscale = _Read16(p_port);
        mcfg.currentoffset = _Read16(p_port);
        mcfg.motor_pwm_rate = _Read16(p_port);
        cfg.rollPitchRate[0] = _Read8(p_port);
        cfg.rollPitchRate[1] = _Read8(p_port);
        mcfg.power_adc_channel = _Read8(p_port);
        cfg.small_angle = _Read8(p_port);
        tmp = _Read16(p_port);
        if (tmp != mcfg.looptime) {
            if (mcfg.looptime && tmp) {
                pifTask_ChangePeriod(g_task_compute_imu, tmp);
            }
            else {
                if (tmp) {
                    pifTask_ChangeMode(g_task_compute_imu, TM_PERIOD_US, tmp);
                }
                else {
                    pifTask_ChangeMode(g_task_compute_imu, TM_ALWAYS, 100);	    // 100%
                }
            }
            mcfg.looptime = tmp;
        }
        cfg.locked_in = _Read8(p_port);
        /// ???
        break;

    case MSP_CONFIG:
        _Serialize8(p_port, mcfg.mixerConfiguration);
        _Serialize32(p_port, featureMask());
        _Serialize8(p_port, mcfg.serialrx_type);
        _Serialize16(p_port, mcfg.board_align_roll);
        _Serialize16(p_port, mcfg.board_align_pitch);
        _Serialize16(p_port, mcfg.board_align_yaw);
        _Serialize16(p_port, mcfg.currentscale);
        _Serialize16(p_port, mcfg.currentoffset);
        _Serialize16(p_port, mcfg.motor_pwm_rate);
        _Serialize8(p_port, cfg.rollPitchRate[0]);
        _Serialize8(p_port, cfg.rollPitchRate[1]);
        _Serialize8(p_port, mcfg.power_adc_channel);
        _Serialize8(p_port, cfg.small_angle);
        _Serialize16(p_port, mcfg.looptime);
        _Serialize8(p_port, cfg.locked_in);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 1 + 4 + 1 + 2 + 2 + 2 + 2 + 2 + 2 + 2 + 1 + 1 + 2 + 1);
        /// ???
        break;

    case MSP_RCMAP:
        for (i = 0; i < MAX_SERIAL_INPUTS; i++)
            _Serialize8(p_port, mcfg.rcmap[i]);
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, MAX_SERIAL_INPUTS);
        break;

    case MSP_SET_RCMAP:
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        for (i = 0; i < MAX_SERIAL_INPUTS; i++)
            mcfg.rcmap[i] = _Read8(p_port);
        break;

    case MSP_REBOOT:
        pifMsp_MakeAnswer(p_owner, p_packet, NULL, 0);
        pendReboot = true;
        break;

    case MSP_BUILDINFO:
        for (i = 0; i < 11; i++)
            _Serialize8(p_port, build[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
        _Serialize32(p_port, 0); // future exp
        _Serialize32(p_port, 0); // future exp
        pifMsp_MakeAnswer(p_owner, p_packet, p_port->tx_packet, 11 + 4 + 4);
        break;

    default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
        pifMsp_MakeError(p_owner, p_packet);
        break;
    }
}

static void evtMspOtherPacket(PifMsp* p_owner, uint8_t data)
{
    (void)p_owner;

    if (f.ARMED) return;

    if (data == '#')
        s_change_cli = TRUE;
    else if (data == mcfg.reboot_character)
        systemReset(true);      // reboot to bootloader
}

void serialCom(void)
{
    if (s_change_cli) {
        s_change_cli = FALSE;

        pifMsp_DetachComm(&ports[0].pif_msp);
        cliInit(&core.mainport->comm);
    };

    if (pendReboot)
        systemReset(false); // noreturn
}
