/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"

#include "core/pif_log.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static int cliAux(int argc, char *argv[]);
static int cliCMix(int argc, char *argv[]);
static int cliDefaults(int argc, char *argv[]);
static int cliDump(int argc, char *argv[]);
static int cliExit(int argc, char *argv[]);
static int cliFeature(int argc, char *argv[]);
#ifdef GPS
static int cliGpsPassthrough(int argc, char *argv[]);
#endif
static int cliMap(int argc, char *argv[]);
static int cliMixer(int argc, char *argv[]);
static int cliMotor(int argc, char *argv[]);
static int cliProfile(int argc, char *argv[]);
static int cliSave(int argc, char *argv[]);
static int cliSet(int argc, char *argv[]);
static int cliServo(int argc, char *argv[]);
static int cliServoMix(int argc, char *argv[]);
static int cliStatus(int argc, char *argv[]);
static int cliVersion(int argc, char *argv[]);

// from sensors.c
extern uint8_t batteryCellCount;

// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// from mixer.c
extern int16_t motor_disarmed[MAX_MOTORS];

// signal that we're in cli mode
uint8_t cliMode = 0;

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

static float _atof(const char *p);
static char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
static const char *const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", "CUSTOMPLANE", NULL
};

// sync this with AvailableFeatures enum from board.h
static const char *const featureNames[] = {
    "PPM", "VBAT", "INFLIGHT_ACC_CAL", "SERIALRX", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "LED_RING", "GPS",
    "FAILSAFE", "SONAR", "TELEMETRY", "POWERMETER", "VARIO", "3D",
    "FW_FAILSAFE_RTH", "SYNCPWM", "FASTPWM",
    NULL
};

// sync this with AvailableSensors enum from board.h
static const char *const sensorNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "SONAR", "GPS", "GPS+MAG", NULL
};

// should be sorted a..z for bsearch()
const PifLogCmdEntry c_psCmdTable[] = {
    { "aux", cliAux, "feature_name auxflag or blank for list" },
    { "cmix", cliCMix, "design custom mixer" },
    { "defaults", cliDefaults, "reset to defaults and reboot" },
    { "dump", cliDump, "print configurable settings in a pastable form" },
    { "exit", cliExit, "" },
    { "feature", cliFeature, "list or -val or val" },
#ifdef GPS
    { "gpspassthrough", cliGpsPassthrough, "passthrough gps to serial" },
#endif
    { "map", cliMap, "mapping of rc channel order" },
    { "mixer", cliMixer, "mixer name or list" },
    { "motor", cliMotor, "get/set motor output value" },
    { "profile", cliProfile, "index (0 to 2)" },
    { "save", cliSave, "save and reboot" },
    { "servo", cliServo, "edit servo configuration" },
    { "set", cliSet, "name=value or blank or * for list" },
    { "smix", cliServoMix, "design custom servo mixer" },
    { "status", cliStatus, "show system status" },
    { "version", cliVersion, "" },
    { NULL, NULL, NULL }
};

typedef enum {
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

typedef struct {
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
} clivalue_t;

const clivalue_t valueTable[] = {
    { "looptime", VAR_UINT16, &mcfg.looptime, 0, 5000 },
    { "emf_avoidance", VAR_UINT8, &mcfg.emf_avoidance, 0, 1 },
    { "midrc", VAR_UINT16, &mcfg.midrc, 1200, 1700 },
    { "minthrottle", VAR_UINT16, &mcfg.minthrottle, 0, 2000 },
    { "maxthrottle", VAR_UINT16, &mcfg.maxthrottle, 0, 2000 },
    { "mincommand", VAR_UINT16, &mcfg.mincommand, 0, 2000 },
    { "mincheck", VAR_UINT16, &mcfg.mincheck, 0, 2000 },
    { "maxcheck", VAR_UINT16, &mcfg.maxcheck, 0, 2000 },
    { "deadband3d_low", VAR_UINT16, &mcfg.deadband3d_low, 0, 2000 },
    { "deadband3d_high", VAR_UINT16, &mcfg.deadband3d_high, 0, 2000 },
    { "neutral3d", VAR_UINT16, &mcfg.neutral3d, 0, 2000 },
    { "deadband3d_throttle", VAR_UINT16, &mcfg.deadband3d_throttle, 0, 2000 },
    { "motor_pwm_rate", VAR_UINT16, &mcfg.motor_pwm_rate, 50, 32000 },
    { "servo_pwm_rate", VAR_UINT16, &mcfg.servo_pwm_rate, 50, 498 },
    { "pwm_filter", VAR_UINT8, &mcfg.pwm_filter, 0, 15 },
    { "retarded_arm", VAR_UINT8, &mcfg.retarded_arm, 0, 1 },
    { "disarm_kill_switch", VAR_UINT8, &mcfg.disarm_kill_switch, 0, 1 },
    { "fw_althold_dir", VAR_INT8, &mcfg.fw_althold_dir, -1, 1 },
    { "reboot_character", VAR_UINT8, &mcfg.reboot_character, 48, 126 },
    { "serial_baudrate", VAR_UINT32, &mcfg.serial_baudrate, 1200, 115200 },
    { "softserial_baudrate", VAR_UINT32, &mcfg.softserial_baudrate, 1200, 19200 },
    { "softserial_1_inverted", VAR_UINT8, &mcfg.softserial_1_inverted, 0, 1 },
    { "softserial_2_inverted", VAR_UINT8, &mcfg.softserial_2_inverted, 0, 1 },
    { "gps_type", VAR_UINT8, &mcfg.gps_type, 0, GPS_HARDWARE_MAX },
    { "gps_baudrate", VAR_INT8, &mcfg.gps_baudrate, 0, GPS_BAUD_MAX },
    { "gps_ubx_sbas", VAR_INT8, &mcfg.gps_ubx_sbas, -1, 4 },
    { "gps_autobaud", VAR_UINT8, &mcfg.gps_autobaud, 0, 1 },
    { "serialrx_type", VAR_UINT8, &mcfg.serialrx_type, 0, SERIALRX_PROVIDER_MAX },
    { "spektrum_sat_bind", VAR_UINT8, &mcfg.spektrum_sat_bind, 0, 10 },
    { "spektrum_sat_on_flexport", VAR_UINT8, &mcfg.spektrum_sat_on_flexport, 0, 1 },
    { "telemetry_provider", VAR_UINT8, &mcfg.telemetry_provider, 0, TELEMETRY_PROVIDER_MAX },
    { "telemetry_port", VAR_UINT8, &mcfg.telemetry_port, 0, TELEMETRY_PORT_MAX },
    { "telemetry_switch", VAR_UINT8, &mcfg.telemetry_switch, 0, 1 },
    { "vbatscale", VAR_UINT8, &mcfg.vbatscale, 10, 200 },
    { "currentscale", VAR_UINT16, &mcfg.currentscale, 1, 10000 },
    { "currentoffset", VAR_UINT16, &mcfg.currentoffset, 0, 1650 },
    { "multiwiicurrentoutput", VAR_UINT8, &mcfg.multiwiicurrentoutput, 0, 1 },
    { "vbatmaxcellvoltage", VAR_UINT8, &mcfg.vbatmaxcellvoltage, 10, 50 },
    { "vbatmincellvoltage", VAR_UINT8, &mcfg.vbatmincellvoltage, 10, 50 },
    { "vbatwarningcellvoltage", VAR_UINT8, &mcfg.vbatwarningcellvoltage, 10, 50 },
    { "power_adc_channel", VAR_UINT8, &mcfg.power_adc_channel, 0, 9 },
    { "align_gyro", VAR_UINT8, &mcfg.gyro_align, 0, 8 },
    { "align_acc", VAR_UINT8, &mcfg.acc_align, 0, 8 },
    { "align_mag", VAR_UINT8, &mcfg.mag_align, 0, 8 },
    { "align_board_roll", VAR_INT16, &mcfg.board_align_roll, -180, 360 },
    { "align_board_pitch", VAR_INT16, &mcfg.board_align_pitch, -180, 360 },
    { "align_board_yaw", VAR_INT16, &mcfg.board_align_yaw, -180, 360 },
    { "yaw_control_direction", VAR_INT8, &mcfg.yaw_control_direction, -1, 1 },
    { "acc_hardware", VAR_UINT8, &mcfg.acc_hardware, 0, 0 },
    { "mag_hardware", VAR_UINT8, &mcfg.mag_hardware, 0, 0 },
    { "max_angle_inclination", VAR_UINT16, &mcfg.max_angle_inclination, 100, 900 },
    { "moron_threshold", VAR_UINT8, &mcfg.moron_threshold, 0, 128 },
    { "gyro_lpf", VAR_UINT16, &mcfg.gyro_lpf, 0, 256 },
    { "gyro_cmpf_factor", VAR_UINT16, &mcfg.gyro_cmpf_factor, 100, 1000 },
    { "gyro_cmpfm_factor", VAR_UINT16, &mcfg.gyro_cmpfm_factor, 100, 1000 },
    { "pid_controller", VAR_UINT8, &cfg.pidController, 0, 1 },
    { "deadband", VAR_UINT8, &cfg.deadband, 0, 32 },
    { "yawdeadband", VAR_UINT8, &cfg.yawdeadband, 0, 100 },
    { "alt_hold_throttle_neutral", VAR_UINT8, &cfg.alt_hold_throttle_neutral, 1, 250 },
    { "alt_hold_fast_change", VAR_UINT8, &cfg.alt_hold_fast_change, 0, 1 },
    { "throttle_correction_value", VAR_UINT8, &cfg.throttle_correction_value, 0, 150 },
    { "throttle_correction_angle", VAR_UINT16, &cfg.throttle_correction_angle, 1, 900 },
    { "rc_rate", VAR_UINT8, &cfg.rcRate8, 0, 250 },
    { "rc_expo", VAR_UINT8, &cfg.rcExpo8, 0, 100 },
    { "thr_mid", VAR_UINT8, &cfg.thrMid8, 0, 100 },
    { "thr_expo", VAR_UINT8, &cfg.thrExpo8, 0, 100 },
    { "roll_rate", VAR_UINT8, &cfg.rollPitchRate[0], 0, 100 },
    { "pitch_rate", VAR_UINT8, &cfg.rollPitchRate[1], 0, 100 },
    { "yaw_rate", VAR_UINT8, &cfg.yawRate, 0, 100 },
    { "tpa_rate", VAR_UINT8, &cfg.dynThrPID, 0, 100},
    { "tpa_breakpoint", VAR_UINT16, &cfg.tpa_breakpoint, 1000, 2000},
    { "failsafe_delay", VAR_UINT8, &cfg.failsafe_delay, 0, 200 },
    { "failsafe_off_delay", VAR_UINT8, &cfg.failsafe_off_delay, 0, 200 },
    { "failsafe_throttle", VAR_UINT16, &cfg.failsafe_throttle, 1000, 2000 },
    { "failsafe_detect_threshold", VAR_UINT16, &cfg.failsafe_detect_threshold, 100, 2000 },
    { "auto_disarm_board", VAR_UINT8, &mcfg.auto_disarm_board, 0, 60 },
    { "rssi_aux_channel", VAR_INT8, &mcfg.rssi_aux_channel, 0, 14 },
    { "rssi_aux_max", VAR_UINT16, &mcfg.rssi_aux_max, 0, 1000 },
    { "rssi_adc_channel", VAR_INT8, &mcfg.rssi_adc_channel, 0, 9 },
    { "rssi_adc_max", VAR_INT16, &mcfg.rssi_adc_max, 1, 4095 },
    { "rssi_adc_offset", VAR_INT16, &mcfg.rssi_adc_offset, 0, 4095 },
    { "rc_channel_count", VAR_UINT8, &mcfg.rc_channel_count, 8, 18 },
    { "yaw_direction", VAR_INT8, &cfg.yaw_direction, -1, 1 },
    { "tri_unarmed_servo", VAR_INT8, &cfg.tri_unarmed_servo, 0, 1 },
    { "gimbal_flags", VAR_UINT8, &cfg.gimbal_flags, 0, 255},
    { "acc_lpf_factor", VAR_UINT8, &cfg.acc_lpf_factor, 0, 250 },
    { "accxy_deadband", VAR_UINT8, &cfg.accxy_deadband, 0, 100 },
    { "accz_deadband", VAR_UINT8, &cfg.accz_deadband, 0, 100 },
    { "acc_unarmedcal", VAR_UINT8, &cfg.acc_unarmedcal, 0, 1 },
    { "small_angle", VAR_UINT8, &cfg.small_angle, 0, 180 },
    { "acc_trim_pitch", VAR_INT16, &cfg.angleTrim[PITCH], -300, 300 },
    { "acc_trim_roll", VAR_INT16, &cfg.angleTrim[ROLL], -300, 300 },
    { "baro_tab_size", VAR_UINT8, &cfg.baro_tab_size, 0, BARO_TAB_SIZE_MAX },
    { "baro_noise_lpf", VAR_FLOAT, &cfg.baro_noise_lpf, 0, 1 },
    { "baro_cf_vel", VAR_FLOAT, &cfg.baro_cf_vel, 0, 1 },
    { "baro_cf_alt", VAR_FLOAT, &cfg.baro_cf_alt, 0, 1 },
    { "accz_lpf_cutoff", VAR_FLOAT, &cfg.accz_lpf_cutoff, 1, 20 },
    { "mag_declination", VAR_INT16, &cfg.mag_declination, -18000, 18000 },
    { "gps_pos_p", VAR_UINT8, &cfg.P8[PIDPOS], 0, 200 },
    { "gps_pos_i", VAR_UINT8, &cfg.I8[PIDPOS], 0, 200 },
    { "gps_pos_d", VAR_UINT8, &cfg.D8[PIDPOS], 0, 200 },
    { "gps_posr_p", VAR_UINT8, &cfg.P8[PIDPOSR], 0, 200 },
    { "gps_posr_i", VAR_UINT8, &cfg.I8[PIDPOSR], 0, 200 },
    { "gps_posr_d", VAR_UINT8, &cfg.D8[PIDPOSR], 0, 200 },
    { "gps_nav_p", VAR_UINT8, &cfg.P8[PIDNAVR], 0, 200 },
    { "gps_nav_i", VAR_UINT8, &cfg.I8[PIDNAVR], 0, 200 },
    { "gps_nav_d", VAR_UINT8, &cfg.D8[PIDNAVR], 0, 200 },
    { "gps_wp_radius", VAR_UINT16, &cfg.gps_wp_radius, 0, 2000 },
    { "nav_controls_heading", VAR_UINT8, &cfg.nav_controls_heading, 0, 1 },
    { "nav_speed_min", VAR_UINT16, &cfg.nav_speed_min, 10, 2000 },
    { "nav_speed_max", VAR_UINT16, &cfg.nav_speed_max, 10, 2000 },
    { "nav_slew_rate", VAR_UINT8, &cfg.nav_slew_rate, 0, 100 },
    { "p_pitch", VAR_UINT8, &cfg.P8[PITCH], 0, 200 },
    { "i_pitch", VAR_UINT8, &cfg.I8[PITCH], 0, 200 },
    { "d_pitch", VAR_UINT8, &cfg.D8[PITCH], 0, 200 },
    { "p_roll", VAR_UINT8, &cfg.P8[ROLL], 0, 200 },
    { "i_roll", VAR_UINT8, &cfg.I8[ROLL], 0, 200 },
    { "d_roll", VAR_UINT8, &cfg.D8[ROLL], 0, 200 },
    { "p_yaw", VAR_UINT8, &cfg.P8[YAW], 0, 200 },
    { "i_yaw", VAR_UINT8, &cfg.I8[YAW], 0, 200 },
    { "d_yaw", VAR_UINT8, &cfg.D8[YAW], 0, 200 },
    { "p_alt", VAR_UINT8, &cfg.P8[PIDALT], 0, 200 },
    { "i_alt", VAR_UINT8, &cfg.I8[PIDALT], 0, 200 },
    { "d_alt", VAR_UINT8, &cfg.D8[PIDALT], 0, 200 },
    { "p_level", VAR_UINT8, &cfg.P8[PIDLEVEL], 0, 200 },
    { "i_level", VAR_UINT8, &cfg.I8[PIDLEVEL], 0, 200 },
    { "d_level", VAR_UINT8, &cfg.D8[PIDLEVEL], 0, 200 },
    { "p_vel", VAR_UINT8, &cfg.P8[PIDVEL], 0, 200 },
    { "i_vel", VAR_UINT8, &cfg.I8[PIDVEL], 0, 200 },
    { "d_vel", VAR_UINT8, &cfg.D8[PIDVEL], 0, 200 },
    { "fw_vector_thrust", VAR_UINT8, &cfg.fw_vector_thrust, 0, 1},
    { "fw_gps_maxcorr", VAR_INT16, &cfg.fw_gps_maxcorr, -45, 45 },
    { "fw_gps_rudder", VAR_INT16, &cfg.fw_gps_rudder,  -45, 45 },
    { "fw_gps_maxclimb", VAR_INT16, &cfg.fw_gps_maxclimb,  -45, 45 },
    { "fw_gps_maxdive", VAR_INT16, &cfg.fw_gps_maxdive,  -45, 45 },
    { "fw_glide_angle", VAR_UINT8, &cfg.fw_glide_angle, 0, 100 },
    { "fw_climb_throttle", VAR_UINT16, &cfg.fw_climb_throttle, 1000, 2000 },
    { "fw_cruise_throttle", VAR_UINT16, &cfg.fw_cruise_throttle, 1000, 2000 },
    { "fw_idle_throttle", VAR_UINT16, &cfg.fw_idle_throttle, 1000, 2000 },
    { "fw_scaler_throttle", VAR_UINT16, &cfg.fw_scaler_throttle, 0, 15 },
    { "fw_roll_comp", VAR_UINT8, &cfg.fw_roll_comp, 0, 250 },
    { "fw_rth_alt", VAR_UINT8, &cfg.fw_rth_alt, 0, 250 },
    { "fw_cruise_distance", VAR_UINT16, &cfg.fw_cruise_distance, 0, 2000},
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(clivalue_t))


typedef union {
    int32_t int_value;
    float float_value;
} int_float_value_t;

static void cliSetVar(const clivalue_t *var, const int_float_value_t value);
static void cliPrintVar(const clivalue_t *var, uint32_t full);

#ifndef HAVE_ITOA_FUNCTION

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0)
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0) {
        *a = '-';
        *i2a(-(unsigned)i, a + 1, r) = 0;
    } else
        *i2a(i, a, r) = 0;
    return a;
}

#endif

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
static float _atof(const char *p)
{
    int frac = 0;
    float sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p)) {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0f;
    if (*p == '-') {
        sign = -1.0f;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0f;
    while (valid_digit(*p)) {
        value = value * 10.0f + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.') {
        float pow10 = 10.0f;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0f;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0f;
    if ((*p == 'e') || (*p == 'E')) {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.
        frac = 0;
        if (*p == '-') {
            frac = 1;
            p += 1;

        } else if (*p == '+') {
            p += 1;
        }

        // Get digits of exponent, if any.
        expon = 0;
        while (valid_digit(*p)) {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308)
            expon = 308;

        // Calculate scaling factor.
        // while (expon >= 50) { scale *= 1E50f; expon -= 50; }
        while (expon >=  8) {
            scale *= 1E8f;
            expon -=  8;
        }
        while (expon >   0) {
            scale *= 10.0f;
            expon -=  1;
        }
    }

    // Return signed and scaled floating point result.
    return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////
static char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t)(x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1) {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 2) {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 3) {
        intString2[1] = '0';
        strcat(intString2, intString1);
    } else {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

static int cliAux(int argc, char *argv[])
{
    int i, val = 0;

    if (argc == 1) {
        // print out aux channel settings
        for (i = 0; i < CHECKBOXITEMS; i++)
            pifLog_Printf(LT_NONE, "aux %u %u\r\n", i, cfg.activate[i]);
    } else if (argc > 2) {
        i = atoi(argv[1]);
        if (i < CHECKBOXITEMS) {
            val = atoi(argv[2]);
            cfg.activate[i] = val;
        } else {
            pifLog_Printf(LT_NONE, "Invalid Feature index: must be < %u\r\n", CHECKBOXITEMS);
        }
    } else {
    	return PIF_LOG_CMD_TOO_FEW_ARGS;
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliCMix(int argc, char *argv[])
{
    int i, check = 0;
    int num_motors = 0;
    char buf[16];
    float mixsum[3];

    if (argc == 1) {
        pifLog_Print(LT_NONE, "Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_MOTORS; i++) {
            if (mcfg.customMixer[i].throttle == 0.0f)
                break;
            num_motors++;
            pifLog_Printf(LT_NONE, "#%d:\t", i + 1);
            pifLog_Printf(LT_NONE, "%s\t", ftoa(mcfg.customMixer[i].throttle, buf));
            pifLog_Printf(LT_NONE, "%s\t", ftoa(mcfg.customMixer[i].roll, buf));
            pifLog_Printf(LT_NONE, "%s\t", ftoa(mcfg.customMixer[i].pitch, buf));
            pifLog_Printf(LT_NONE, "%s\r\n", ftoa(mcfg.customMixer[i].yaw, buf));
        }
        mixsum[0] = mixsum[1] = mixsum[2] = 0.0f;
        for (i = 0; i < num_motors; i++) {
            mixsum[0] += mcfg.customMixer[i].roll;
            mixsum[1] += mcfg.customMixer[i].pitch;
            mixsum[2] += mcfg.customMixer[i].yaw;
        }
        pifLog_Print(LT_NONE, "Sanity check:\t");
        for (i = 0; i < 3; i++)
            pifLog_Print(LT_NONE, fabsf(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        pifLog_Print(LT_NONE, "\r\n");
    } else if (strcasecmp(argv[1], "reset") == 0) {
        // erase custom mixer
        for (i = 0; i < MAX_MOTORS; i++)
            mcfg.customMixer[i].throttle = 0.0f;
    } else if (strcasecmp(argv[1], "load") == 0) {
        if (argc > 2) {
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    pifLog_Print(LT_NONE, "Invalid mixer type...\r\n");
                    break;
                }
                if (strcasecmp(argv[2], mixerNames[i]) == 0) {
                    mixerLoadMix(i);
                    pifLog_Printf(LT_NONE, "Loaded %s mix...\r\n", mixerNames[i]);
                    cliCMix(1, NULL);
                    break;
                }
            }
        }
        else {
        	return PIF_LOG_CMD_TOO_FEW_ARGS;
        }
    } else {
        i = atoi(argv[1]); // get motor number
        if (--i < MAX_MOTORS) {
            if (argc > 2) {
                mcfg.customMixer[i].throttle = _atof(argv[2]);
                check++;
            }
            if (argc > 3) {
                mcfg.customMixer[i].roll = _atof(argv[3]);
                check++;
            }
            if (argc > 4) {
                mcfg.customMixer[i].pitch = _atof(argv[4]);
                check++;
            }
            if (argc > 5) {
                mcfg.customMixer[i].yaw = _atof(argv[5]);
                check++;
            }
            if (check != 4) {
                pifLog_Print(LT_NONE, "Wrong number of arguments, needs idx thr roll pitch yaw\r\n");
            } else {
                cliCMix(1, NULL);
            }
        } else {
            pifLog_Printf(LT_NONE, "Motor number must be between 1 and %d\r\n", MAX_MOTORS);
        }
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliServo(int argc, char *argv[])
{
    int i;
    int8_t servoRates[8] = { 30, 30, 100, 100, 100, 100, 100, 100 };

    if (argc == 1) {
        pifLog_Print(LT_NONE, "servo servo_number\tmin\tmiddle\tmax\trate\r\n");
        for (i = 0; i < MAX_SERVOS; i++) {
            pifLog_Printf(LT_NONE, "#%d:\t", i + 1);
            pifLog_Printf(LT_NONE, "%d\t", cfg.servoConf[i].min);
            pifLog_Printf(LT_NONE, "%d\t", cfg.servoConf[i].middle);
            pifLog_Printf(LT_NONE, "%d\t", cfg.servoConf[i].max);
            pifLog_Printf(LT_NONE, "%d\t", cfg.servoConf[i].rate);
            pifLog_Print(LT_NONE, "\r\n");
        }
        pifLog_Print(LT_NONE, "\r\n");
        pifLog_Print(LT_NONE, "Reset servos: servo reset\r\n");
    } else if (strcasecmp(argv[1], "reset") == 0) {
        // erase servo config
        for (i = 0; i < MAX_SERVOS; i++) {
            cfg.servoConf[i].min = 1020;
            cfg.servoConf[i].max = 2000;
            cfg.servoConf[i].middle = 1500;
            cfg.servoConf[i].rate = servoRates[i];
        }
    } else {
        enum {SERVO = 0, MIN, MIDDLE, MAX, RATE, ARGS_COUNT};
        int args[ARGS_COUNT], check = 0;

        while (check + 1 < argc && check < ARGS_COUNT) {
            args[check] = atoi(argv[check + 1]);
            check++;
        }

        if (check != ARGS_COUNT) {
            pifLog_Print(LT_NONE, "ERR: Wrong number of arguments, needs servo_number min middle max rate\r\n");
        	return PIF_LOG_CMD_NO_ERROR;
        }

        if (args[SERVO] >= 1 && args[SERVO] <= MAX_SERVOS &&
            args[MIN] >= 900 && args[MIN] <= 2100 &&
            args[MAX] >= 900 && args[MAX] <= 2100 &&
            args[MIDDLE] >= 900 && args[MIDDLE] <= 2100 &&
            args[RATE] >= -100 && args[RATE] <= 100 &&
            args[MIN] <= args[MIDDLE] && args[MIDDLE] <= args[MAX]) {
            args[SERVO]--;
            cfg.servoConf[args[SERVO]].min = args[MIN];
            cfg.servoConf[args[SERVO]].max = args[MAX];
            cfg.servoConf[args[SERVO]].middle = args[MIDDLE];
            cfg.servoConf[args[SERVO]].rate = args[RATE];
        } else
            pifLog_Print(LT_NONE, "ERR: Wrong range for arguments, range for min, max and middle [900,2100], min <= middle <= max, range for rate [-100,100]\r\n");
        cliServo(1, NULL);
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliServoMix(int argc, char *argv[])
{
    int i;
    int args[8], check = 0;

    if (argc == 1) {
        pifLog_Print(LT_NONE, "Custom servo mixer: \r\nchange mixer: smix rule\ttarget_channel\tinput_channel\trate\tspeed\t\tmin\tmax\tbox\r\n");
        pifLog_Print(LT_NONE, "reset mixer: smix reset\r\nload mixer: smix load\r\nchange direction of channel: smix direction\r\n");
        for (i = 0; i < MAX_SERVO_RULES; i++) {
            if (mcfg.customServoMixer[i].rate == 0)
                break;
            pifLog_Printf(LT_NONE, "#%d:\t", i + 1);
            pifLog_Printf(LT_NONE, "%d\t", mcfg.customServoMixer[i].targetChannel + 1);
            pifLog_Printf(LT_NONE, "%d\t", mcfg.customServoMixer[i].fromChannel + 1);
            pifLog_Printf(LT_NONE, "%d\t", mcfg.customServoMixer[i].rate);
            pifLog_Printf(LT_NONE, "%d\t", mcfg.customServoMixer[i].speed);
            pifLog_Printf(LT_NONE, "%d\t", mcfg.customServoMixer[i].min);
            pifLog_Printf(LT_NONE, "%d\t", mcfg.customServoMixer[i].max);
            pifLog_Printf(LT_NONE, "%d\r\n", mcfg.customServoMixer[i].box);
        }
        pifLog_Print(LT_NONE, "\r\n");
    } else if (strcasecmp(argv[1], "reset") == 0) {
        // erase custom mixer
        memset(mcfg.customServoMixer, 0, sizeof(mcfg.customServoMixer));
        for (i = 0; i < MAX_SERVOS; i++)
            cfg.servoConf[i].direction = 0;
    } else if (strcasecmp(argv[1], "load") == 0) {
        if (argc > 2) {
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    pifLog_Print(LT_NONE, "Invalid mixer type...\r\n");
                    break;
                }
                if (strcasecmp(argv[2], mixerNames[i]) == 0) {
                    servoMixerLoadMix(i);
                    pifLog_Printf(LT_NONE, "Loaded %s mix...\r\n", mixerNames[i]);
                    cliServoMix(1, NULL);
                    break;
                }
            }
        }
        else {
        	return PIF_LOG_CMD_TOO_FEW_ARGS;
        }
    } else if (strcasecmp(argv[1], "direction") == 0) {
        enum {SERVO = 0, INPUT_, DIRECTION, ARGS_COUNT};
        int servoIndex, channel;
        char* smix_dir[2] = { "smix", "direction" };

        if (argc == 2) {
            pifLog_Print(LT_NONE, "change the direction a servo reacts to a input channel: \r\nservo input -1|1\r\n");
            pifLog_Print(LT_NONE, "s");
            for (channel = 0; channel < INPUT_ITEMS; channel++)
                pifLog_Printf(LT_NONE, "\ti%d", channel + 1);
            pifLog_Print(LT_NONE, "\r\n");

            for (servoIndex = 0; servoIndex < MAX_SERVOS; servoIndex++) {
                pifLog_Printf(LT_NONE, "%d", servoIndex + 1);
                for (channel = 0; channel < INPUT_ITEMS; channel++)
                    pifLog_Printf(LT_NONE, "\t%s  ", (cfg.servoConf[servoIndex].direction & (1 << channel)) ? "r" : "n");
                pifLog_Print(LT_NONE, "\r\n");
            }
            return PIF_LOG_CMD_NO_ERROR;
        }

        while (check + 1 < argc && check < ARGS_COUNT) {
            args[check] = atoi(argv[check + 1]);
            check++;
        }

        if (check != ARGS_COUNT) {
            pifLog_Print(LT_NONE, "Wrong number of arguments, needs servo input direction\r\n");
            return PIF_LOG_CMD_NO_ERROR;
        }

        if (args[SERVO] >= 1 && args[SERVO] <= MAX_SERVOS && args[INPUT_] >= 1 && args[INPUT_] <= INPUT_ITEMS && (args[DIRECTION] == -1 || args[DIRECTION] == 1)) {
            args[SERVO] -= 1;
            args[INPUT_] -= 1;
            if (args[DIRECTION] == -1)
                cfg.servoConf[args[SERVO]].direction |= 1 << args[INPUT_];
            else
                cfg.servoConf[args[SERVO]].direction &= ~(1 << args[INPUT_]);
        } else
            pifLog_Print(LT_NONE, "ERR: Wrong range for arguments\r\n");

        cliServoMix(2, smix_dir);
    } else {
        enum {RULE = 0, TARGET, INPUT_, RATE, SPEED, MIN, MAX, BOX, ARGS_COUNT};
        while (check + 1 < argc && check < ARGS_COUNT) {
            args[check] = atoi(argv[check + 1]);
            check++;
        }

        if (check != ARGS_COUNT) {
            pifLog_Print(LT_NONE, "ERR: Wrong number of arguments, needs rule target_channel input_channel rate speed min max box\r\n");
            return PIF_LOG_CMD_NO_ERROR;
        }

        i = args[RULE] - 1;
        if (i >= 0 && i < MAX_SERVO_RULES &&
            args[TARGET] > 0 && args[TARGET] <= MAX_SERVOS &&
            args[INPUT_] >= 1 && args[INPUT_] <= INPUT_ITEMS &&
            args[RATE] >= -100 && args[RATE] <= 100 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED &&
            args[MIN] >= 0 && args[MIN] <= 100 &&
            args[MAX] >= 0 && args[MAX] <= 100 && args[MIN] < args[MAX] &&
            args[BOX] >= 0 && args[BOX] <= MAX_SERVO_BOXES) {
            mcfg.customServoMixer[i].targetChannel = args[TARGET] - 1;
            mcfg.customServoMixer[i].fromChannel = args[INPUT_] - 1;
            mcfg.customServoMixer[i].rate = args[RATE];
            mcfg.customServoMixer[i].speed = args[SPEED];
            mcfg.customServoMixer[i].min = args[MIN];
            mcfg.customServoMixer[i].max = args[MAX];
            mcfg.customServoMixer[i].box = args[BOX];
            cliServoMix(1, NULL);
        } else
            pifLog_Print(LT_NONE, "ERR: Wrong range for arguments\r\n");
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliDefaults(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    pifLog_Print(LT_NONE, "Resetting to defaults...\r\n");
    checkFirstTime(true);
    pifLog_Print(LT_NONE, "Rebooting...");
    pif_Delay1ms(10);
    systemReset(false);
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliDump(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    unsigned int i, channel;
    char buf[16];
    float thr, roll, pitch, yaw;
    uint32_t mask;
    const clivalue_t *setval;

    cliVersion(1, NULL);
    pifLog_Print(LT_NONE, "Current Config: Copy everything below here...\r\n");

    // print out aux switches
    cliAux(1, NULL);

    // print out current motor mix
    pifLog_Printf(LT_NONE, "mixer %s\r\n", mixerNames[mcfg.mixerConfiguration - 1]);

    // print custom mix if exists
    if (mcfg.customMixer[0].throttle != 0.0f) {
        for (i = 0; i < MAX_MOTORS; i++) {
            if (mcfg.customMixer[i].throttle == 0.0f)
                break;
            thr = mcfg.customMixer[i].throttle;
            roll = mcfg.customMixer[i].roll;
            pitch = mcfg.customMixer[i].pitch;
            yaw = mcfg.customMixer[i].yaw;
            pifLog_Printf(LT_NONE, "cmix %d", i + 1);
            if (thr < 0)
                pifLog_Print(LT_NONE, " ");
            pifLog_Printf(LT_NONE, "%s", ftoa(thr, buf));
            if (roll < 0)
                pifLog_Print(LT_NONE, " ");
            pifLog_Printf(LT_NONE, "%s", ftoa(roll, buf));
            if (pitch < 0)
                pifLog_Print(LT_NONE, " ");
            pifLog_Printf(LT_NONE, "%s", ftoa(pitch, buf));
            if (yaw < 0)
                pifLog_Print(LT_NONE, " ");
            pifLog_Printf(LT_NONE, "%s\r\n", ftoa(yaw, buf));
        }
        pifLog_Printf(LT_NONE, "cmix %d 0 0 0 0\r\n", i + 1);
    }

    // print custom servo mixer if exists
    if (mcfg.customServoMixer[0].rate != 0) {
        for (i = 0; i < MAX_SERVO_RULES; i++) {
            if (mcfg.customServoMixer[i].rate == 0)
                break;
            pifLog_Printf(LT_NONE, "smix %d ", i + 1);
            pifLog_Printf(LT_NONE, "%d ", mcfg.customServoMixer[i].targetChannel + 1);
            pifLog_Printf(LT_NONE, "%d ", mcfg.customServoMixer[i].fromChannel + 1);
            pifLog_Printf(LT_NONE, "%d ", mcfg.customServoMixer[i].rate);
            pifLog_Printf(LT_NONE, "%d ", mcfg.customServoMixer[i].speed);
            pifLog_Printf(LT_NONE, "%d ", mcfg.customServoMixer[i].min);
            pifLog_Printf(LT_NONE, "%d ", mcfg.customServoMixer[i].max);
            pifLog_Printf(LT_NONE, "%d\r\n", mcfg.customServoMixer[i].box);
        }
        pifLog_Printf(LT_NONE, "smix %d 0 0 0 0\r\n", i + 1);
    }

    // print servo directions
    for (i = 0; i < MAX_SERVOS; i++)
        for (channel = 0; channel < INPUT_ITEMS; channel++)
            if (cfg.servoConf[i].direction & (1 << channel))
                pifLog_Printf(LT_NONE, "smix direction %d %d -1\r\n", i + 1 , channel + 1);

    // print servo config
    for (i = 0; i < MAX_SERVOS; i++)
        pifLog_Printf(LT_NONE, "servo %d %d %d %d %d\r\n", i + 1, cfg.servoConf[i].min, cfg.servoConf[i].middle, cfg.servoConf[i].max, cfg.servoConf[i].rate);

    // print enabled features
    mask = featureMask();
    for (i = 0; ; i++) { // disable all feature first
        if (featureNames[i] == NULL)
            break;
        pifLog_Printf(LT_NONE, "feature -%s\r\n", featureNames[i]);
    }
    for (i = 0; ; i++) {  // reenable what we want.
        if (featureNames[i] == NULL)
            break;
        if (mask & (1 << i))
            pifLog_Printf(LT_NONE, "feature %s\r\n", featureNames[i]);
    }

    // print RC MAPPING
    for (i = 0; i < mcfg.rc_channel_count; i++)
        buf[mcfg.rcmap[i]] = rcChannelLetters[i];
    buf[i] = '\0';
    pifLog_Printf(LT_NONE, "map %s\r\n", buf);

    // print settings
    for (i = 0; i < VALUE_COUNT; i++) {
        setval = &valueTable[i];
        pifLog_Printf(LT_NONE, "set %s = ", valueTable[i].name);
        cliPrintVar(setval, 0);
        pifLog_Print(LT_NONE, "\r\n");
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliExit(int argc, char *argv[])
{
    pifLog_Print(LT_NONE, "\r\nLeaving CLI mode...\r\n");
    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
    // incase some idiot leaves a motor running during motortest, clear it here
    mixerResetMotors();
    // save and reboot... I think this makes the most sense
    cliSave(argc, argv);
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliFeature(int argc, char *argv[])
{
    uint32_t i;
    uint32_t mask;

    mask = featureMask();

    if (argc == 1) {
        pifLog_Print(LT_NONE, "Enabled features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                pifLog_Printf(LT_NONE, "%s ", featureNames[i]);
        }
        pifLog_Print(LT_NONE, "\r\n");
    } else if (strcasecmp(argv[1], "list") == 0) {
        pifLog_Print(LT_NONE, "Available features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            pifLog_Printf(LT_NONE, "%s ", featureNames[i]);
        }
        pifLog_Print(LT_NONE, "\r\n");
    } else {
        bool remove = false;
        if (argv[1][0] == '-') {
            // remove feature
            remove = true;
        }

        for (i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                pifLog_Print(LT_NONE, "Invalid feature name...\r\n");
                break;
            }
            if (strcasecmp(argv[1] + 1, featureNames[i]) == 0) {
                if (remove) {
                    featureClear(1 << i);
                    pifLog_Print(LT_NONE, "Disabled ");
                } else {
                    featureSet(1 << i);
                    pifLog_Print(LT_NONE, "Enabled ");
                }
                pifLog_Printf(LT_NONE, "%s\r\n", featureNames[i]);
                break;
            }
        }
    }
	return PIF_LOG_CMD_NO_ERROR;
}

#ifdef GPS
static int cliGpsPassthrough(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    switch (gpsSetPassthrough()) {
    case 0:
        pifLog_Print(LT_NONE, "Disabling GPS passthrough...\r\n");
        break;

    case 1:
        pifLog_Print(LT_NONE, "Enabling GPS passthrough...\r\n");
        break;

    default:
        pifLog_Print(LT_NONE, "Error: Enable and plug in GPS first\r\n");
        break;
    }
	return PIF_LOG_CMD_NO_ERROR;
}
#endif

static int cliMap(int argc, char *argv[])
{
    uint32_t len;
    uint32_t i;
    char out[9];

    if (argc < 2) return PIF_LOG_CMD_TOO_FEW_ARGS;

    len = strlen(argv[1]);

    if (len == mcfg.rc_channel_count) {
        // uppercase it
        for (i = 0; i < mcfg.rc_channel_count; i++)
            argv[1][i] = toupper((unsigned char)argv[1][i]);
        for (i = 0; i < mcfg.rc_channel_count; i++) {
            if (strchr(rcChannelLetters, argv[1][i]) && !strchr(argv[1] + i + 1, argv[1][i]))
                continue;
            pifLog_Print(LT_NONE, "Must be any order of AETR1234\r\n");
            return PIF_LOG_CMD_NO_ERROR;
        }
        parseRcChannels(argv[1]);
    }
    pifLog_Print(LT_NONE, "Current assignment: ");
    for (i = 0; i < mcfg.rc_channel_count; i++)
        out[mcfg.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    pifLog_Printf(LT_NONE, "%s\r\n", out);
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliMixer(int argc, char *argv[])
{
    int i;

    if (argc == 1) {
        pifLog_Printf(LT_NONE, "Current mixer: %s\r\n", mixerNames[mcfg.mixerConfiguration - 1]);
        return PIF_LOG_CMD_NO_ERROR;
    } else if (strcasecmp(argv[1], "list") == 0) {
        pifLog_Print(LT_NONE, "Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            pifLog_Printf(LT_NONE, "%s ", mixerNames[i]);
        }
        pifLog_Print(LT_NONE, "\r\n");
        return PIF_LOG_CMD_NO_ERROR;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            pifLog_Print(LT_NONE, "Invalid mixer type...\r\n");
            return PIF_LOG_CMD_NO_ERROR;
        }
        if (strcasecmp(argv[1], mixerNames[i]) == 0) {
            mcfg.mixerConfiguration = i + 1;
            pifLog_Printf(LT_NONE, "Mixer set to %s\r\n", mixerNames[i]);

            // Presets for planes. Not functional with current reset
            // Really Ugly Hack
            if (mcfg.mixerConfiguration == MULTITYPE_FLYING_WING || mcfg.mixerConfiguration == MULTITYPE_AIRPLANE) {
                cfg.dynThrPID = 90;
                cfg.rcExpo8 = 0;
                cfg.P8[PIDALT] = 30;
                cfg.I8[PIDALT] = 20;
                cfg.D8[PIDALT] = 45;
                cfg.P8[PIDNAVR] = 30;
                cfg.I8[PIDNAVR] = 20;
                cfg.D8[PIDNAVR] = 45;
            }
            break;
        }
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliMotor(int argc, char *argv[])
{
    int motor_index = 0;
    int motor_value = 0;

    if (argc == 1) {
        pifLog_Print(LT_NONE, "Usage:\r\nmotor index [value] - show [or set] motor value\r\n");
        return PIF_LOG_CMD_NO_ERROR;
    }

    if (argc < 3) {
        pifLog_Printf(LT_NONE, "Motor %d is set at %d\r\n", motor_index, motor_disarmed[motor_index]);
        return PIF_LOG_CMD_NO_ERROR;
    }

    motor_index = atoi(argv[1]);
    motor_value = atoi(argv[2]);

    if (motor_index < 0 || motor_index >= MAX_MOTORS) {
        pifLog_Printf(LT_NONE, "No such motor, use a number [0, %d]\r\n", MAX_MOTORS);
        return PIF_LOG_CMD_NO_ERROR;
    }

    if (motor_value < 1000 || motor_value > 2000) {
        pifLog_Print(LT_NONE, "Invalid motor value, 1000..2000\r\n");
        return PIF_LOG_CMD_NO_ERROR;
    }

    pifLog_Printf(LT_NONE, "Setting motor %d to %d\r\n", motor_index, motor_value);
    motor_disarmed[motor_index] = motor_value;
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliProfile(int argc, char *argv[])
{
    int i;

    if (argc == 1) {
        pifLog_Printf(LT_NONE, "Current profile: %d\r\n", mcfg.current_profile);
    } else {
        i = atoi(argv[1]);
        if (i >= 0 && i <= 2) {
            mcfg.current_profile = i;
            writeEEPROM(0, false);
            cliProfile(1, NULL);
        }
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliSave(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    pifLog_Print(LT_NONE, "Saving...");
    writeEEPROM(0, true);
    pifLog_Print(LT_NONE, "\r\nRebooting...");
    pif_Delay1ms(10);
    systemReset(false);
	return PIF_LOG_CMD_NO_ERROR;
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    switch (var->type) {
        case VAR_UINT8:
            value = *(uint8_t *)var->ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)var->ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)var->ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)var->ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)var->ptr;
            break;

        case VAR_FLOAT:
            pifLog_Printf(LT_NONE, "%s", ftoa(*(float *)var->ptr, buf));
            if (full) {
                pifLog_Printf(LT_NONE, " %s", ftoa((float)var->min, buf));
                pifLog_Printf(LT_NONE, " %s", ftoa((float)var->max, buf));
            }
            return; // return from case for float only
    }
    pifLog_Printf(LT_NONE, "%d", value);
    if (full)
        pifLog_Printf(LT_NONE, " %d %d", var->min, var->max);
}

static void cliSetVar(const clivalue_t *var, const int_float_value_t value)
{
    switch (var->type) {
        case VAR_UINT8:
        case VAR_INT8:
            *(char *)var->ptr = (char)value.int_value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(short *)var->ptr = (short)value.int_value;
            break;

        case VAR_UINT32:
            *(int *)var->ptr = (int)value.int_value;
            break;

        case VAR_FLOAT:
            *(float *)var->ptr = (float)value.float_value;
            break;
    }
}

static int cliSet(int argc, char *argv[])
{
    uint32_t i;
    const clivalue_t *val;
    char *eqptr = NULL;
    int32_t value = 0;
    float valuef = 0;

    if (argc == 1 || (argc == 2 && argv[1][0] == '*')) {
        pifLog_Print(LT_NONE, "Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            pifLog_Printf(LT_NONE, "%s = ", valueTable[i].name);
            cliPrintVar(val, strlen(argv[1])); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            pifLog_Print(LT_NONE, "\r\n");
        }
    } else if ((eqptr = strstr(argv[1], "=")) != NULL) {
        // has equal, set var
        eqptr++;
        value = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            if (strncasecmp(argv[1], valueTable[i].name, strlen(valueTable[i].name)) == 0) {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max) { // here we compare the float value since... it should work, RIGHT?
                    int_float_value_t tmp;
                    if (valueTable[i].type == VAR_FLOAT)
                        tmp.float_value = valuef;
                    else
                        tmp.int_value = value;
                    cliSetVar(val, tmp);
                    pifLog_Printf(LT_NONE, "%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    pifLog_Print(LT_NONE, "ERR: Value assignment out of range\r\n");
                }
                return PIF_LOG_CMD_NO_ERROR;
            }
        }
        pifLog_Print(LT_NONE, "ERR: Unknown variable name\r\n");
        return PIF_LOG_CMD_NO_ERROR;
    } else {
        // no equals, check for matching variables.
        for (i = 0; i < VALUE_COUNT; i++) {
            if (strstr(valueTable[i].name, argv[1])) {
                val = &valueTable[i];
                pifLog_Printf(LT_NONE, "%s = ", valueTable[i].name);
                cliPrintVar(val, 0);
                pifLog_Print(LT_NONE, "\r\n");
            }
        }
    }
	return PIF_LOG_CMD_NO_ERROR;
}

static int cliStatus(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    uint8_t i;
    uint32_t mask;

    pifLog_Printf(LT_NONE, "System Uptime: %ld seconds, Voltage: %d * 0.1V (%dS battery)\r\n",
           pif_cumulative_timer1ms / 1000, vbat, batteryCellCount);
    mask = sensorsMask();

    pifLog_Printf(LT_NONE, "Hardware: %s @ %ldMHz, detected sensors: ", g_board_name, (g_core_clock / 1000000));
    for (i = 0; ; i++) {
        if (sensorNames[i] == NULL)
            break;
        if (mask & (1 << i))
            pifLog_Printf(LT_NONE, "%s ", sensorNames[i]);
    }
    pifLog_Print(LT_NONE, "\r\n");
    if (sensors(SENSOR_GYRO))
        pifLog_Printf(LT_NONE, "GYRO_HW: %s ", sensor_set.gyro.hardware);
    if (sensors(SENSOR_ACC))
        pifLog_Printf(LT_NONE, "ACC_HW: %s ", sensor_set.acc.hardware);
    if (sensors(SENSOR_MAG))
        pifLog_Printf(LT_NONE, ", MAG_HW: %s ", sensor_set.mag.hardware);
    if (sensors(SENSOR_BARO))
        pifLog_Printf(LT_NONE, ", BARO_HW: %s ", sensor_set.baro.hardware);
    pifLog_Print(LT_NONE, "\r\n");

    pifLog_Printf(LT_NONE, "Cycle Time: %d, I2C Errors: %d, config size: %d\r\n", cycleTime, g_i2c_port.error_count, sizeof(master_t));

   	pifLog_Printf(LT_NONE, "PIF Timer 1ms count=%d\n", pifTimerManager_Count(&g_timer_1ms));

    cliStatusCallback();

	return PIF_LOG_CMD_NO_ERROR;
}

static int cliVersion(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    pifLog_Printf(LT_NONE, "Afro32 CLI version 2.3 " __DATE__ " / " __TIME__);
	return PIF_LOG_CMD_NO_ERROR;
}

BOOL cliInit(PifComm* p_comm)
{
    if (!cliMode) {
        cliMode = 1;

#ifndef __PIF_NO_LOG__
        pifLog_DetachComm();
#else
        pifLog_Init();
#endif
        if (!pifLog_AttachComm(p_comm)) return FALSE;
        if (!pifLog_UseCommand(c_psCmdTable, "\r\n# ")) return FALSE;
        pifLog_Print(LT_NONE, "\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
    }
    return TRUE;
}

__attribute__ ((weak)) void cliStatusCallback()
{

}
