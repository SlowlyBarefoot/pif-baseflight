/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "cli.h"
#include "link_driver.h"
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
} mspPortState_t;

static mspPortState_t ports[2];

static BOOL s_change_cli = FALSE;

static void evtMspReceive(PifMsp* p_owner, PifMspPacket* p_packet, PifIssuerP p_issuer);
static void evtMspOtherPacket(PifMsp* p_owner, uint8_t data, PifIssuerP p_issuer);

static void _SerializeBoxNamesReply(PifMsp* p_owner)
{
    int i, idx;

    // in first run of the loop, we grab total size of junk to be sent
    // then come back and actually send it
    for (i = 0; i < numberBoxItems; i++) {
        idx = availableBoxes[i];
       	pifMsp_AddAnswer(p_owner, (uint8_t*)boxes[idx].boxName, strlen(boxes[idx].boxName));
    }
}

void serialInit(uint8_t port, uint32_t baudrate, uint8_t flexport)
{
    int idx;

    core.mainport = uartOpen(port, baudrate, MODE_RXTX, 2);			// 10ms
    ports[0].port = core.mainport;

    if (!pifMsp_Init(&ports[0].pif_msp, &g_timer_1ms, PIF_ID_MSP(0))) return;
    pifMsp_AttachEvtReceive(&ports[0].pif_msp, evtMspReceive, evtMspOtherPacket, core.mainport);
    pifMsp_AttachComm(&ports[0].pif_msp, &core.mainport->comm);

    serialStartReceiveFunc(&core.mainport->comm);

    // additional telemetry port available only if spektrum sat isn't already assigned there
    if (flexport) {
        core.flexport = uartOpen(flexport, baudrate, MODE_RXTX, 10);	// 10ms
        ports[1].port = core.flexport;

        if (!pifMsp_Init(&ports[1].pif_msp, &g_timer_1ms, PIF_ID_MSP(1))) return;
        pifMsp_AttachEvtReceive(&ports[1].pif_msp, evtMspReceive, evtMspOtherPacket, core.flexport);
        pifMsp_AttachComm(&ports[1].pif_msp, &core.flexport->comm);

        serialStartReceiveFunc(&core.flexport->comm);
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

void evtMspReceive(PifMsp* p_owner, PifMspPacket* p_packet, PifIssuerP p_issuer)
{
    uint32_t i, j, tmp, junk;
#ifdef GPS
    uint8_t wp_no;
    int32_t lat = 0, lon = 0, alt = 0;
#endif
    const char *build = __DATE__;

    (void)p_issuer;

    pifMsp_MakeAnswer(p_owner, p_packet);
    switch (p_packet->command) {
    case MSP_SET_RAW_RC:
        for (i = 0; i < 8; i++)
            rcData[i] = pifMsp_ReadData16(p_packet);
        mspFrameRecieve();
        break;
        
    case MSP_SET_ACC_TRIM:
        cfg.angleTrim[PITCH] = pifMsp_ReadData16(p_packet);
        cfg.angleTrim[ROLL]  = pifMsp_ReadData16(p_packet);
        break;

#ifdef GPS
    case MSP_SET_RAW_GPS:
        f.GPS_FIX = pifMsp_ReadData8(p_packet);
        GPS_numSat = pifMsp_ReadData8(p_packet);
        GPS_coord[LAT] = pifMsp_ReadData32(p_packet);
        GPS_coord[LON] = pifMsp_ReadData32(p_packet);
        GPS_altitude = pifMsp_ReadData16(p_packet);
        GPS_speed = pifMsp_ReadData16(p_packet);
        GPS_update |= 2;        // New data signalisation to GPS functions
        break;
#endif

    case MSP_SET_PID:
        for (i = 0; i < PIDITEMS; i++) {
            cfg.P8[i] = pifMsp_ReadData8(p_packet);
            cfg.I8[i] = pifMsp_ReadData8(p_packet);
            cfg.D8[i] = pifMsp_ReadData8(p_packet);
#ifndef __PIF_NO_LOG__
            pifLog_Printf(LT_INFO, "S-PID:%d P=%u I=%u D=%u", i, cfg.P8[i], cfg.I8[i], cfg.D8[i]);
#endif
        }
        break;

    case MSP_SET_BOX:
        for (i = 0; i < numberBoxItems; i++)
            cfg.activate[availableBoxes[i]] = pifMsp_ReadData16(p_packet);
        break;

    case MSP_SET_RC_TUNING:
        cfg.rcRate8 = pifMsp_ReadData8(p_packet);
        cfg.rcExpo8 = pifMsp_ReadData8(p_packet);
        pifMsp_ReadData8(p_packet); // Legacy pitch-roll rate, read but not set.
        cfg.yawRate = pifMsp_ReadData8(p_packet);
        cfg.dynThrPID = pifMsp_ReadData8(p_packet);
        cfg.thrMid8 = pifMsp_ReadData8(p_packet);
        cfg.thrExpo8 = pifMsp_ReadData8(p_packet);
        break;

    case MSP_SET_MISC:
        tmp = pifMsp_ReadData16(p_packet);
        // sanity check
        if (tmp < 1600 && tmp > 1400)
            mcfg.midrc = tmp;
        mcfg.minthrottle = pifMsp_ReadData16(p_packet);
        mcfg.maxthrottle = pifMsp_ReadData16(p_packet);
        mcfg.mincommand = pifMsp_ReadData16(p_packet);
        cfg.failsafe_throttle = pifMsp_ReadData16(p_packet);
        mcfg.gps_type = pifMsp_ReadData8(p_packet);
        mcfg.gps_baudrate = pifMsp_ReadData8(p_packet);
        mcfg.gps_ubx_sbas = pifMsp_ReadData8(p_packet);
        mcfg.multiwiicurrentoutput = pifMsp_ReadData8(p_packet);
        mcfg.rssi_aux_channel = pifMsp_ReadData8(p_packet);
        pifMsp_ReadData8(p_packet);
        cfg.mag_declination = pifMsp_ReadData16(p_packet) * 10;
        mcfg.vbatscale = pifMsp_ReadData8(p_packet);           // actual vbatscale as intended
        mcfg.vbatmincellvoltage = pifMsp_ReadData8(p_packet);  // vbatlevel_warn1 in MWC2.3 GUI
        mcfg.vbatmaxcellvoltage = pifMsp_ReadData8(p_packet);  // vbatlevel_warn2 in MWC2.3 GUI
        mcfg.vbatwarningcellvoltage = pifMsp_ReadData8(p_packet); // vbatlevel when buzzer starts to alert
        break;

    case MSP_SET_MOTOR:
        for (i = 0; i < 8; i++)
            motor_disarmed[i] = pifMsp_ReadData16(p_packet);
        break;

    case MSP_SELECT_SETTING:
        if (!f.ARMED) {
            mcfg.current_profile = pifMsp_ReadData8(p_packet);
            if (mcfg.current_profile > 2)
                mcfg.current_profile = 0;
            // this writes new profile index and re-reads it
            writeEEPROM(0, false);
        }
        break;

    case MSP_SET_HEAD:
        magHold = pifMsp_ReadData16(p_packet);
        break;

    case MSP_IDENT:
        pifMsp_AddAnswer8(p_owner, VERSION);                    // multiwii version
        pifMsp_AddAnswer8(p_owner, mcfg.mixerConfiguration);    // type of multicopter
        pifMsp_AddAnswer8(p_owner, MSP_VERSION);                // MultiWii Serial Protocol Version
        pifMsp_AddAnswer32(p_owner, CAP_PLATFORM_32BIT | CAP_BASEFLIGHT_CONFIG | CAP_DYNBALANCE | CAP_FW_FLAPS); // "capability"
        break;

    case MSP_STATUS:
        pifMsp_AddAnswer16(p_owner, cycleTime);
        pifMsp_AddAnswer16(p_owner, g_i2c_port.error_count);
        pifMsp_AddAnswer16(p_owner, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
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
        pifMsp_AddAnswer32(p_owner, junk);
        pifMsp_AddAnswer8(p_owner, mcfg.current_profile);
        break;

    case MSP_RAW_IMU:
        // Retarded hack until multiwiidorks start using real units for sensor data
        if (sensor_set.acc.acc_1G > 1024) {
            for (i = 0; i < 3; i++)
                pifMsp_AddAnswer16(p_owner, accSmooth[i] / 8);
        } else {
            for (i = 0; i < 3; i++)
                pifMsp_AddAnswer16(p_owner, accSmooth[i]);
        }
        for (i = 0; i < 3; i++)
            pifMsp_AddAnswer16(p_owner, gyroData[i]);
        for (i = 0; i < 3; i++)
            pifMsp_AddAnswer16(p_owner, magADC[i]);
        break;

    case MSP_SERVO:
        pifMsp_AddAnswer(p_owner, (uint8_t *)&servo, 16);
        break;

    case MSP_SERVO_CONF:
        for (i = 0; i < MAX_SERVOS; i++) {
            pifMsp_AddAnswer16(p_owner, cfg.servoConf[i].min);
            pifMsp_AddAnswer16(p_owner, cfg.servoConf[i].max);
            pifMsp_AddAnswer16(p_owner, cfg.servoConf[i].middle);
            pifMsp_AddAnswer8(p_owner, cfg.servoConf[i].rate);
            pifMsp_AddAnswer16(p_owner, cfg.servoConf[i].direction);
        }
        break;

    case MSP_SET_SERVO_CONF:
        for (i = 0; i < MAX_SERVOS; i++) {
            cfg.servoConf[i].min = pifMsp_ReadData16(p_packet);
            cfg.servoConf[i].max = pifMsp_ReadData16(p_packet);
            cfg.servoConf[i].middle = pifMsp_ReadData16(p_packet);
            cfg.servoConf[i].rate = pifMsp_ReadData8(p_packet);
            cfg.servoConf[i].direction = pifMsp_ReadData16(p_packet);
        }
        break;

    case MSP_SERVOMIX_CONF:
        for (i = 0; i < MAX_SERVO_RULES; i++) {
            pifMsp_AddAnswer8(p_owner, mcfg.customServoMixer[i].targetChannel);
            pifMsp_AddAnswer8(p_owner, mcfg.customServoMixer[i].fromChannel);
            pifMsp_AddAnswer8(p_owner, mcfg.customServoMixer[i].rate);
            pifMsp_AddAnswer8(p_owner, mcfg.customServoMixer[i].speed);
            pifMsp_AddAnswer8(p_owner, mcfg.customServoMixer[i].min);
            pifMsp_AddAnswer8(p_owner, mcfg.customServoMixer[i].max);
            pifMsp_AddAnswer8(p_owner, mcfg.customServoMixer[i].box);
        }
        break;

    case MSP_SET_SERVOMIX_CONF:
        for (i = 0; i < MAX_SERVO_RULES; i++) {
            mcfg.customServoMixer[i].targetChannel = pifMsp_ReadData8(p_packet);
            mcfg.customServoMixer[i].fromChannel = pifMsp_ReadData8(p_packet);
            mcfg.customServoMixer[i].rate = pifMsp_ReadData8(p_packet);
            mcfg.customServoMixer[i].speed = pifMsp_ReadData8(p_packet);
            mcfg.customServoMixer[i].min = pifMsp_ReadData8(p_packet);
            mcfg.customServoMixer[i].max = pifMsp_ReadData8(p_packet);
            mcfg.customServoMixer[i].box = pifMsp_ReadData8(p_packet);
        }
        loadCustomServoMixer();
        break;

    case MSP_FW_CONFIG:
        pifMsp_AddAnswer8(p_owner, mcfg.fw_althold_dir);
        // pifMsp_AddAnswer8(p_owner, cfg.fw_vector_thrust); // Future Gui setting?
        pifMsp_AddAnswer16(p_owner, cfg.fw_gps_maxcorr);
        pifMsp_AddAnswer16(p_owner, cfg.fw_gps_rudder);
        pifMsp_AddAnswer16(p_owner, cfg.fw_gps_maxclimb);
        pifMsp_AddAnswer16(p_owner, cfg.fw_gps_maxdive);
        pifMsp_AddAnswer16(p_owner, cfg.fw_climb_throttle);
        pifMsp_AddAnswer16(p_owner, cfg.fw_cruise_throttle);
        pifMsp_AddAnswer16(p_owner, cfg.fw_idle_throttle);
        pifMsp_AddAnswer16(p_owner, cfg.fw_scaler_throttle);
        pifMsp_AddAnswer32(p_owner, cfg.fw_roll_comp); // Float is Not compatible with Gui. Change to _Serialize8
        pifMsp_AddAnswer8(p_owner, cfg.fw_rth_alt);
        // next added for future use
        pifMsp_AddAnswer32(p_owner, 0);
        pifMsp_AddAnswer32(p_owner, 0);
        pifMsp_AddAnswer32(p_owner, 0);
        pifMsp_AddAnswer32(p_owner, 0);
        break;

    case MSP_SET_FW_CONFIG:
        mcfg.fw_althold_dir = pifMsp_ReadData8(p_packet);
        // cfg.fw_vector_thrust = pifMsp_ReadData8(p_packet); // Future Gui setting?
        cfg.fw_gps_maxcorr = pifMsp_ReadData16(p_packet);
        cfg.fw_gps_rudder = pifMsp_ReadData16(p_packet);
        cfg.fw_gps_maxclimb = pifMsp_ReadData16(p_packet);
        cfg.fw_gps_maxdive = pifMsp_ReadData16(p_packet);
        cfg.fw_climb_throttle = pifMsp_ReadData16(p_packet);
        cfg.fw_cruise_throttle = pifMsp_ReadData16(p_packet);
        cfg.fw_idle_throttle = pifMsp_ReadData16(p_packet);
        cfg.fw_scaler_throttle = pifMsp_ReadData16(p_packet);
        //cfg.fw_gps_maxdive = pifMsp_ReadData32(p_packet);// Wrong when using float in MSP!... Change to pifMsp_ReadData8
        pifMsp_ReadData32(p_packet); // Just read and skip
        cfg.fw_rth_alt = pifMsp_ReadData8(p_packet);
        // next added for future use
        pifMsp_ReadData32(p_packet);
        pifMsp_ReadData32(p_packet);
        pifMsp_ReadData32(p_packet);
        pifMsp_ReadData32(p_packet);
        break;

    case MSP_MOTOR:
        pifMsp_AddAnswer(p_owner, (uint8_t *)&motor, 16);
        break;

    case MSP_RC:
        for (i = 0; i < 8; i++)
            pifMsp_AddAnswer16(p_owner, rcData[i]);
        break;

    case MSP_RAW_GPS:
        pifMsp_AddAnswer8(p_owner, f.GPS_FIX);
        pifMsp_AddAnswer8(p_owner, GPS_numSat);
        pifMsp_AddAnswer32(p_owner, GPS_coord[LAT]);
        pifMsp_AddAnswer32(p_owner, GPS_coord[LON]);
        pifMsp_AddAnswer16(p_owner, GPS_altitude);
        pifMsp_AddAnswer16(p_owner, GPS_speed);
        pifMsp_AddAnswer16(p_owner, GPS_ground_course);
        break;

    case MSP_COMP_GPS:
        pifMsp_AddAnswer16(p_owner, GPS_distanceToHome);
        pifMsp_AddAnswer16(p_owner, GPS_directionToHome);
        pifMsp_AddAnswer8(p_owner, GPS_update & 1);
        break;

    case MSP_ATTITUDE:
        for (i = 0; i < 2; i++)
            pifMsp_AddAnswer16(p_owner, angle[i]);
        pifMsp_AddAnswer16(p_owner, heading);
        break;

    case MSP_ALTITUDE:
        pifMsp_AddAnswer32(p_owner, EstAlt);
        pifMsp_AddAnswer16(p_owner, vario);
        break;

    case MSP_ANALOG:
        pifMsp_AddAnswer8(p_owner, (uint8_t)constrain((int16_t)vbat, 0, 255));
        pifMsp_AddAnswer16(p_owner, (uint16_t)constrain(mAhdrawn, 0, 0xFFFF)); // milliamphours drawn from battery
        pifMsp_AddAnswer16(p_owner, rssi);
        if (mcfg.multiwiicurrentoutput)
            pifMsp_AddAnswer16(p_owner, (uint16_t)constrain((abs(amperage) * 10), 0, 0xFFFF)); // send amperage in 0.001 A steps
        else
            pifMsp_AddAnswer16(p_owner, (uint16_t)constrain(abs(amperage), 0, 0xFFFF)); // send amperage in 0.01 A steps
        break;

    case MSP_RC_TUNING:
        pifMsp_AddAnswer8(p_owner, cfg.rcRate8);
        pifMsp_AddAnswer8(p_owner, cfg.rcExpo8);
        pifMsp_AddAnswer8(p_owner, cfg.rollPitchRate[0]); // here for legacy support
        pifMsp_AddAnswer8(p_owner, cfg.yawRate);
        pifMsp_AddAnswer8(p_owner, cfg.dynThrPID);
        pifMsp_AddAnswer8(p_owner, cfg.thrMid8);
        pifMsp_AddAnswer8(p_owner, cfg.thrExpo8);
        break;

    case MSP_PID:
        for (i = 0; i < PIDITEMS; i++) {
            pifMsp_AddAnswer8(p_owner, cfg.P8[i]);
            pifMsp_AddAnswer8(p_owner, cfg.I8[i]);
            pifMsp_AddAnswer8(p_owner, cfg.D8[i]);
#ifndef __PIF_NO_LOG__
            pifLog_Printf(LT_INFO, "PID:%d P=%u I=%u D=%u", i, cfg.P8[i], cfg.I8[i], cfg.D8[i]);
#endif
        }
        break;

    case MSP_PIDNAMES:
        pifMsp_AddAnswer(p_owner, (uint8_t*)pidnames, sizeof(pidnames));
        break;

    case MSP_BOX:
        for (i = 0; i < numberBoxItems; i++)
            pifMsp_AddAnswer16(p_owner, cfg.activate[availableBoxes[i]]);
        break;

    case MSP_BOXNAMES:
        _SerializeBoxNamesReply(p_owner);
        break;

    case MSP_BOXIDS:
        for (i = 0; i < numberBoxItems; i++) {
            for  (j = 0; j < CHECKBOXITEMS; j++) {
                if (boxes[j].permanentId == availableBoxes[i]) {
                    pifMsp_AddAnswer8(p_owner, boxes[j].permanentId);
                    break;
                }
            }
        }
        break;

    case MSP_MISC:
        pifMsp_AddAnswer16(p_owner, mcfg.midrc);
        pifMsp_AddAnswer16(p_owner, mcfg.minthrottle);
        pifMsp_AddAnswer16(p_owner, mcfg.maxthrottle);
        pifMsp_AddAnswer16(p_owner, mcfg.mincommand);
        pifMsp_AddAnswer16(p_owner, cfg.failsafe_throttle);
        pifMsp_AddAnswer8(p_owner, mcfg.gps_type);
        pifMsp_AddAnswer8(p_owner, mcfg.gps_baudrate);
        pifMsp_AddAnswer8(p_owner, mcfg.gps_ubx_sbas);
        pifMsp_AddAnswer8(p_owner, mcfg.multiwiicurrentoutput);
        pifMsp_AddAnswer8(p_owner, mcfg.rssi_aux_channel);
        pifMsp_AddAnswer8(p_owner, 0);
        pifMsp_AddAnswer16(p_owner, cfg.mag_declination / 10); // TODO check this shit
        pifMsp_AddAnswer8(p_owner, mcfg.vbatscale);
        pifMsp_AddAnswer8(p_owner, mcfg.vbatmincellvoltage);
        pifMsp_AddAnswer8(p_owner, mcfg.vbatmaxcellvoltage);
        pifMsp_AddAnswer8(p_owner, mcfg.vbatwarningcellvoltage);
        break;

    case MSP_MOTOR_PINS:
        for (i = 0; i < 8; i++)
            pifMsp_AddAnswer8(p_owner, i + 1);
        break;

#ifdef GPS
    case MSP_WP:
        wp_no = pifMsp_ReadData8(p_packet);    // get the wp number
        if (wp_no == 0) {
            lat = GPS_home[LAT];
            lon = GPS_home[LON];
        } else if (wp_no == 16) {
            lat = GPS_hold[LAT];
            lon = GPS_hold[LON];
        }
        pifMsp_AddAnswer8(p_owner, wp_no);
        pifMsp_AddAnswer32(p_owner, lat);
        pifMsp_AddAnswer32(p_owner, lon);
        pifMsp_AddAnswer32(p_owner, AltHold);           // altitude (cm) will come here -- temporary implementation to test feature with apps
        pifMsp_AddAnswer16(p_owner, 0);                 // heading  will come here (deg)
        pifMsp_AddAnswer16(p_owner, 0);                 // time to stay (ms) will come here
        pifMsp_AddAnswer8(p_owner, 0);                  // nav flag will come here
        break;

    case MSP_SET_WP:
        wp_no = pifMsp_ReadData8(p_packet);    //get the wp number
        lat = pifMsp_ReadData32(p_packet);
        lon = pifMsp_ReadData32(p_packet);
        alt = pifMsp_ReadData32(p_packet);     // to set altitude (cm)
        pifMsp_ReadData16(p_packet);           // future: to set heading (deg)
        pifMsp_ReadData16(p_packet);           // future: to set time to stay (ms)
        pifMsp_ReadData8(p_packet);            // future: to set nav flag
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
        break;
#endif  // GPS

    case MSP_RESET_CONF:
        if (!f.ARMED)
            checkFirstTime(true);
        break;

    case MSP_ACC_CALIBRATION:
        if (!f.ARMED)
            calibratingA = CALIBRATING_ACC_CYCLES;
        break;

    case MSP_MAG_CALIBRATION:
        if (!f.ARMED)
            f.CALIBRATE_MAG = 1;
        break;

    case MSP_EEPROM_WRITE:
        if (f.ARMED) {
            pifMsp_MakeError(p_owner, p_packet);
        } else {
            writeEEPROM(0, true);
        }
        break;

    case MSP_DEBUG:
        // make use of this crap, output some useful QA statistics
        if (mcfg.looptime) debug[0] = (int16_t)cycleTime - mcfg.looptime + mcfg.looptime / 10;
        else debug[0] = cycleTime;
        debug[1] = pif_performance._use_rate;
        debug[3] = ((g_crystal_clock / 1000000) * 1000) + (g_core_clock / 1000000);         // XX0YY [crystal clock : core clock]
        for (i = 0; i < 4; i++)
            pifMsp_AddAnswer16(p_owner, debug[i]);      // 4 variables are here for general monitoring purpose
        break;

    // Additional commands that are not compatible with MultiWii
    case MSP_ACC_TRIM:
        pifMsp_AddAnswer16(p_owner, cfg.angleTrim[PITCH]);
        pifMsp_AddAnswer16(p_owner, cfg.angleTrim[ROLL]);
        break;

    case MSP_UID:
        pifMsp_AddAnswer32(p_owner, g_unique_id[0]);
        pifMsp_AddAnswer32(p_owner, g_unique_id[1]);
        pifMsp_AddAnswer32(p_owner, g_unique_id[2]);
        break;

#ifdef GPS
    case MSP_GPSSVINFO:
        pifMsp_AddAnswer8(p_owner, GPS_numCh);
        for (i = 0; i < GPS_numCh; i++) {
            pifMsp_AddAnswer8(p_owner, GPS_svinfo_chn[i]);
            pifMsp_AddAnswer8(p_owner, GPS_svinfo_svid[i]);
            pifMsp_AddAnswer8(p_owner, GPS_svinfo_quality[i]);
            pifMsp_AddAnswer8(p_owner, GPS_svinfo_cno[i]);
        }
        // Poll new SVINFO from GPS
        gpsPollSvinfo();
        break;
    case MSP_GPSDEBUGINFO:
        if (sensors(SENSOR_GPS)) {
            pifMsp_AddAnswer32(p_owner, GPS_update_rate[1] - GPS_update_rate[0]);
            pifMsp_AddAnswer32(p_owner, GPS_svinfo_rate[1] - GPS_svinfo_rate[0]);
        } else {
            pifMsp_AddAnswer32(p_owner, 0);
            pifMsp_AddAnswer32(p_owner, 0);
        }
        pifMsp_AddAnswer32(p_owner, GPS_HorizontalAcc);
        pifMsp_AddAnswer32(p_owner, GPS_VerticalAcc);
        break;
#endif  // GPS

    case MSP_SET_CONFIG:
        mcfg.mixerConfiguration = pifMsp_ReadData8(p_packet); // multitype
        featureClearAll();
        featureSet(pifMsp_ReadData32(p_packet)); // features bitmap
        mcfg.serialrx_type = pifMsp_ReadData8(p_packet); // serialrx_type
        mcfg.board_align_roll = pifMsp_ReadData16(p_packet); // board_align_roll
        mcfg.board_align_pitch = pifMsp_ReadData16(p_packet); // board_align_pitch
        mcfg.board_align_yaw = pifMsp_ReadData16(p_packet); // board_align_yaw
        mcfg.currentscale = pifMsp_ReadData16(p_packet);
        mcfg.currentoffset = pifMsp_ReadData16(p_packet);
        mcfg.motor_pwm_rate = pifMsp_ReadData16(p_packet);
        cfg.rollPitchRate[0] = pifMsp_ReadData8(p_packet);
        cfg.rollPitchRate[1] = pifMsp_ReadData8(p_packet);
        mcfg.power_adc_channel = pifMsp_ReadData8(p_packet);
        cfg.small_angle = pifMsp_ReadData8(p_packet);
        tmp = pifMsp_ReadData16(p_packet);
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
        cfg.locked_in = pifMsp_ReadData8(p_packet);
        /// ???
        break;

    case MSP_CONFIG:
        pifMsp_AddAnswer8(p_owner, mcfg.mixerConfiguration);
        pifMsp_AddAnswer32(p_owner, featureMask());
        pifMsp_AddAnswer8(p_owner, mcfg.serialrx_type);
        pifMsp_AddAnswer16(p_owner, mcfg.board_align_roll);
        pifMsp_AddAnswer16(p_owner, mcfg.board_align_pitch);
        pifMsp_AddAnswer16(p_owner, mcfg.board_align_yaw);
        pifMsp_AddAnswer16(p_owner, mcfg.currentscale);
        pifMsp_AddAnswer16(p_owner, mcfg.currentoffset);
        pifMsp_AddAnswer16(p_owner, mcfg.motor_pwm_rate);
        pifMsp_AddAnswer8(p_owner, cfg.rollPitchRate[0]);
        pifMsp_AddAnswer8(p_owner, cfg.rollPitchRate[1]);
        pifMsp_AddAnswer8(p_owner, mcfg.power_adc_channel);
        pifMsp_AddAnswer8(p_owner, cfg.small_angle);
        pifMsp_AddAnswer16(p_owner, mcfg.looptime);
        pifMsp_AddAnswer8(p_owner, cfg.locked_in);
        /// ???
        break;

    case MSP_RCMAP:
        for (i = 0; i < MAX_SERIAL_INPUTS; i++)
            pifMsp_AddAnswer8(p_owner, mcfg.rcmap[i]);
        break;

    case MSP_SET_RCMAP:
        for (i = 0; i < MAX_SERIAL_INPUTS; i++)
            mcfg.rcmap[i] = pifMsp_ReadData8(p_packet);
        break;

    case MSP_REBOOT:
        pendReboot = true;
        break;

    case MSP_BUILDINFO:
        for (i = 0; i < 11; i++)
            pifMsp_AddAnswer8(p_owner, build[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
        pifMsp_AddAnswer32(p_owner, 0); // future exp
        pifMsp_AddAnswer32(p_owner, 0); // future exp
        break;

    default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
        pifMsp_MakeError(p_owner, p_packet);
        break;
    }
    pifMsp_SendAnswer(p_owner);
}

static void evtMspOtherPacket(PifMsp* p_owner, uint8_t data, PifIssuerP p_issuer)
{
    (void)p_owner;
    (void)p_issuer;

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
