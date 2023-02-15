/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"

#ifndef __PIF_NO_LOG__
	#include "core/pif_log.h"
#endif
#include "gps/pif_gps_nmea.h"
#include "gps/pif_gps_ublox.h"


extern void fw_nav_reset(void);
#ifdef GPS

#ifndef sq
#define sq(x) ((x)*(x))
#endif

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2.5second)
#define GPS_TIMEOUT (2500)
// How many entries in gpsInitData array below
#define GPS_INIT_ENTRIES (GPS_BAUD_MAX + 1)
#define GPS_BAUD_DELAY (200)

typedef struct gpsInitData_t {
    uint8_t index;
    uint32_t baudrate;
} gpsInitData_t;

// NMEA will cycle through these until valid data is received
static const gpsInitData_t gpsInitData[] = {
    { GPS_BAUD_115200, 115200 },
    { GPS_BAUD_57600,   57600 },
    { GPS_BAUD_38400,   38400 },
    { GPS_BAUD_19200,   19200 },
    // 9600 is not enough for 5Hz updates - leave for compatibility to dumb NMEA that only runs at this speed
    { GPS_BAUD_9600,     9600 }
};

const uint8_t kCfgMsg[][3] = {
        { GUCI_NMEA_STD, GUMI_NMEA_VTG, 0x00 }, // Course over ground and Ground speed
        { GUCI_NMEA_STD, GUMI_NMEA_GSV, 0x00 }, // GNSS Satellites in View
        { GUCI_NMEA_STD, GUMI_NMEA_GLL, 0x00 }, // Latitude and longitude, with time of position fix and status
        { GUCI_NMEA_STD, GUMI_NMEA_GGA, 0x00 }, // Global positioning system fix data
        { GUCI_NMEA_STD, GUMI_NMEA_GSA, 0x00 }, // GNSS DOP and Active Satellites
        { GUCI_NMEA_STD, GUMI_NMEA_RMC, 0x00 }, // Recommended Minimum data
        { GUCI_NAV, GUMI_NAV_POSLLH, 0x01 },    // set POSLLH MSG rate
        { GUCI_NAV, GUMI_NAV_STATUS, 0x01 },    // set STATUS MSG rate
        { GUCI_NAV, GUMI_NAV_SOL, 0x01 },       // set SOL MSG rate
        { GUCI_NAV, GUMI_NAV_VELNED, 0x01 }     // set VELNED MSG rate
};

const uint8_t kCfgRate[] = {
        0xC8, 0x00,				// messRate 5Hz
        0x01, 0x00, 			// navRate
        0x01, 0x00 				// timeRef
};

const uint8_t kCfgNav5[] = {
        0xFF, 0xFF,						// mask
        0x06, 							// dynModel
        0x03, 							// fixMode
        0x00, 0x00,	0x00, 0x00, 		// fixedAlt
        0x10, 0x27, 0x00, 0x00,			// fixedAltVar
        0x05, 							// minElev
        0x00,							// drLimit
        0xFA, 0x00,  					// pDop
        0xFA, 0x00,						// tDop
        0x64, 0x00, 					// pAcc
        0x2C, 0x01,						// tAcc
        0x00,							// staticHoldThresh
        0x00,							// dgnssTimeout
        0x00,  							// cnoThreshNumSVs
        0x00, 							// cnoThresh
        0x00, 0x00,	 					// reserved1
        0x00, 0x00,						// staticHoldMaxDist
        0x00, 							// utcStandard
        0x00, 0x00, 0x00, 0x00, 0x00	// reserved2
};

const uint8_t kCfgSbas[][8] = {
        { 0x03, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 },   	// Auto
        { 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00 },   	// EGNOS
        { 0x03, 0x07, 0x03, 0x00, 0x04, 0xE0, 0x04, 0x00 },   	// WAAS
        { 0x03, 0x07, 0x03, 0x00, 0x00, 0x02, 0x02, 0x00 },   	// MSAS + KASS
        { 0x03, 0x07, 0x03, 0x00, 0x80, 0x01, 0x00, 0x00 },   	// GAGAN
        { 0x02, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 }		// Disabled
};

enum {
    SBAS_DISABLED = -1,
    SBAS_AUTO,
    SBAS_EGNOS,
    SBAS_WAAS,
    SBAS_MSAS_KASS,
    SBAS_GAGAN,
    SBAS_LAST
};

enum {
    GPS_UNKNOWN,
    GPS_INITIALIZING,
	GPS_SENDBAUD,
    GPS_SETBAUD,
    GPS_CONFIGURATION,
    GPS_RECEIVINGDATA,
    GPS_LOSTCOMMS,
};

enum {
	GUCR_NONE,
	GUCR_ACK,
	GUCR_NAK
};

typedef struct gpsData_t {
    uint8_t state;                  // GPS thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t baudrateIndex;          // index into auto-detecting or current baudrate
    int errors;                     // gps error counter - crc error/lost of data/sync etc. reset on each reinit.
    uint32_t lastMessage;           // last time valid GPS data was received (millis)

    int step;
    int state_position;             // incremental variable for loops
    uint32_t state_ts;              // timestamp for last state_position increment
    BOOL receive;
    int cfg_result;
} gpsData_t;

static gpsData_t gpsData;
static PifGpsNmea gps_nmea;
static PifGpsUblox gps_ublox;


static void gpsSetState(uint8_t state)
{
    gpsData.state = state;
    gpsData.state_position = 0;
    gpsData.state_ts = pif_cumulative_timer1ms;
    gpsData.step = 0;
    gpsData.receive = FALSE;
}

static void _evtGpsUbloxCfgResult(PifGpsUblox* p_owner, BOOL result)
{
	(void)p_owner;

	gpsData.cfg_result = result ? GUCR_ACK : GUCR_NAK;
}

static void _evtGpsReceive(PifGps *p_owner)
{
    f.GPS_FIX = p_owner->_fix;
    if (f.GPS_FIX) {
        GPS_coord[LAT] = p_owner->_coord_deg[PIF_GPS_LAT] * 10000000UL;
        GPS_coord[LON] = p_owner->_coord_deg[PIF_GPS_LON] * 10000000UL;
        GPS_numSat = p_owner->_num_sat;
        GPS_altitude = p_owner->_altitude;
        if (!sensors(SENSOR_BARO) && f.FIXED_WING)
            EstAlt = (GPS_altitude - GPS_home[ALT]) * 100;    // Use values Based on GPS
    }

    GPS_speed = (uint16_t)(p_owner->_ground_speed / 10);
    GPS_ground_course = (uint16_t)(p_owner->_ground_course * 10);
    if (!sensors(SENSOR_MAG) && GPS_speed > 100) {
        GPS_ground_course = wrap_18000(GPS_ground_course * 10) / 10;
        heading = GPS_ground_course / 10;    // Use values Based on GPS if we are moving.
    }

    GPS_HorizontalAcc = p_owner->_horizontal_acc;
    GPS_VerticalAcc = p_owner->_vertical_acc;

    pifTask_SetTrigger(g_task_gps);
    gpsData.receive = TRUE;
}

static void _evtGpsTimeout(PifGps *p_owner)
{
    (void)p_owner;

    // remove GPS from capability
    if (mcfg.gps_type != GPS_NMEA || gpsInitData[gpsData.baudrateIndex].baudrate != 9600) {
        serialStopReceiveFunc(&core.gpsport->comm);
    }
    sensorsClear(SENSOR_GPS);
    gpsSetState(GPS_LOSTCOMMS);
#ifndef __PIF_NO_LOG__
    pifLog_Print(LT_INFO, "GPS: Timeout");
#endif
}

void gpsInit(uint8_t port, uint8_t baudrateIndex)
{
    // init gpsData structure. if we're not actually enabled, don't bother doing anything else
    gpsSetState(GPS_UNKNOWN);

    gpsData.baudrateIndex = baudrateIndex;
    gpsData.lastMessage = pif_cumulative_timer1ms;
    gpsData.errors = 0;

    gpsSetPIDs();
    // Open GPS UART, no callback - buffer will be read out in gpsThread()
    core.gpsport = uartOpen(port, 9600, MODE_RXTX, 5);    // signal GPS "thread" to initialize when it gets to it, 5ms
    if (mcfg.gps_type == GPS_NMEA && gpsInitData[baudrateIndex].baudrate == 9600) {
    	serialStartReceiveFunc(&core.gpsport->comm);

        if (!pifGpsNmea_Init(&gps_nmea, PIF_ID_AUTO)) return;
        gps_nmea._gps.evt_nmea_msg_id = PIF_GPS_NMEA_MSG_ID_GGA;
        pifGpsNmea_AttachComm(&gps_nmea, &core.gpsport->comm);
        gps_nmea._gps.evt_receive = _evtGpsReceive;

		// signal GPS "thread" to initialize when it gets to it
		gpsSetState(GPS_CONFIGURATION);
    }
    else {
        if (!pifGpsUblox_Init(&gps_ublox, PIF_ID_AUTO)) return;
        pifGpsUblox_AttachComm(&gps_ublox, &core.gpsport->comm);
        gps_ublox._gps.evt_receive = _evtGpsReceive;

		// signal GPS "thread" to initialize when it gets to it
		gpsSetState(GPS_INITIALIZING);
    }

    // copy ubx sbas config string to use
    if (mcfg.gps_ubx_sbas >= SBAS_LAST)
        mcfg.gps_ubx_sbas = SBAS_AUTO;
}

static void gpsInitNmea(void)
{
    if (gpsInitData[gpsData.baudrateIndex].baudrate == 9600) {
        pifGps_SetTimeout(&gps_nmea._gps, &g_timer_1ms, GPS_TIMEOUT, _evtGpsTimeout);
    }
    else {
        gps_ublox._gps.evt_nmea_msg_id = PIF_GPS_NMEA_MSG_ID_GGA;
        pifGps_SetTimeout(&gps_ublox._gps, &g_timer_1ms, GPS_TIMEOUT, _evtGpsTimeout);
    }
    gpsSetState(GPS_RECEIVINGDATA);
}

static void gpsInitUblox(void)
{
	static uint8_t cfg_msg_size = 0;
    uint8_t i;
    int line = 0;

	// GPS_CONFIGURATION, push some ublox config strings
	if (gpsData.step >= 20) {
		if (gpsData.cfg_result == GUCR_ACK) {
	  		gpsData.step = (gpsData.step - 20) + 1;
			if (gpsData.step == cfg_msg_size) gpsData.step = 15;
			gpsData.state_ts = pif_cumulative_timer1ms;
		}
		else if (gpsData.cfg_result == GUCR_NAK) {
			pif_error = E_RECEIVE_NACK;
			line = __LINE__;
		}
		else {
			if (pif_cumulative_timer1ms - gpsData.state_ts >= 200) {
				pif_error = E_TIMEOUT;
				line = __LINE__;
			}
		}
	}
	else {
		if (cfg_msg_size == 0) {
			cfg_msg_size = sizeof(kCfgMsg) / sizeof(kCfgMsg[0]);
			gps_ublox.evt_ubx_cfg_result = _evtGpsUbloxCfgResult;
			gpsData.state_ts = pif_cumulative_timer1ms;
		}
		if (pif_cumulative_timer1ms - gpsData.state_ts < 5) return;
		if (gpsData.step < cfg_msg_size) {
			if (pifGpsUblox_SendUbxMsg(&gps_ublox, GUCI_CFG, GUMI_CFG_MSG, sizeof(kCfgMsg[gpsData.step]), (uint8_t*)kCfgMsg[gpsData.step], FALSE)) {
                gpsData.cfg_result = GUCR_NONE;
				gpsData.step += 20;
				gpsData.state_ts = pif_cumulative_timer1ms;
			}
			else {
				pif_error = E_TRANSFER_FAILED;
				line = __LINE__;
			}
		}
		else if (gpsData.step == 15) {
			if (pifGpsUblox_SendUbxMsg(&gps_ublox, GUCI_CFG, GUMI_CFG_RATE, sizeof(kCfgRate), (uint8_t*)kCfgRate, FALSE)) {
                gpsData.cfg_result = GUCR_NONE;
				gpsData.step += 20;
				gpsData.state_ts = pif_cumulative_timer1ms;
			}
			else {
				pif_error = E_TRANSFER_FAILED;
				line = __LINE__;
			}
		}
		else if (gpsData.step == 16) {
			if (pifGpsUblox_SendUbxMsg(&gps_ublox, GUCI_CFG, GUMI_CFG_NAV5, sizeof(kCfgNav5), (uint8_t*)kCfgNav5, FALSE)) {
                gpsData.cfg_result = GUCR_NONE;
				gpsData.step += 20;
				gpsData.state_ts = pif_cumulative_timer1ms;
			}
			else {
				pif_error = E_TRANSFER_FAILED;
				line = __LINE__;
			}
		}
		else if (gpsData.step == 17) {
			i = mcfg.gps_ubx_sbas > SBAS_DISABLED ? mcfg.gps_ubx_sbas : SBAS_LAST;
			if (pifGpsUblox_SendUbxMsg(&gps_ublox, GUCI_CFG, GUMI_CFG_SBAS, sizeof(kCfgSbas[i]), (uint8_t*)kCfgSbas[i], FALSE)) {
                gpsData.cfg_result = GUCR_NONE;
				gpsData.step += 20;
				gpsData.state_ts = pif_cumulative_timer1ms;
			}
			else {
				pif_error = E_TRANSFER_FAILED;
				line = __LINE__;
			}
		}
		else if (gpsData.step == 18) {
			if (pif_cumulative_timer1ms - gpsData.state_ts < 10000) {
				if (gpsData.receive) {
					// ublox should be init'd, time to try receiving some junk
					serialStartReceiveFunc(&core.gpsport->comm);
					pifGps_SetTimeout(&gps_ublox._gps, &g_timer_1ms, GPS_TIMEOUT, _evtGpsTimeout);
					gpsSetState(GPS_RECEIVINGDATA);
				}
			}
			else {
				pif_error = E_TIMEOUT;
				line = __LINE__;
			}
		}
	}

	if (line) {
#ifndef __PIF_NO_LOG__
		pifLog_Printf(LT_ERROR, "GPS(%u) CS:%u S:%u E:%u", line, cfg_msg_size, gpsData.step, pif_error);
#endif
		_evtGpsTimeout(&gps_ublox._gps);
	}
}

static void gpsInitHardware(void)
{
    switch (mcfg.gps_type) {
        case GPS_NMEA:
            gpsInitNmea();
            break;

        case GPS_UBLOX:
            gpsInitUblox();
            break;

        case GPS_MTK_NMEA:
        case GPS_MTK_BINARY:
            // TODO. need to find my old piece of shit MTK GPS.
            break;
    }

    // clear error counter
    gpsData.errors = 0;
}

void gpsThread(void)
{
    uint32_t m;

    switch (gpsData.state) {
        case GPS_UNKNOWN:
            break;

        case GPS_INITIALIZING:
            m = pif_cumulative_timer1ms;
            if (m - gpsData.state_ts < (gpsData.state_position ? GPS_BAUD_DELAY : 3000))
                return;

            if (gpsData.state_position < GPS_INIT_ENTRIES) {
                // try different speed to INIT
                serialSetBaudRate(core.gpsport, gpsInitData[gpsData.state_position].baudrate);
                gpsData.state = GPS_SENDBAUD;
                gpsData.state_ts = m;
            }
            else
            {
                // we're now (hopefully) at the correct rate, next state will switch to it
                gpsSetState(GPS_SETBAUD);
            }
            break;

        case GPS_SENDBAUD:
            m = pif_cumulative_timer1ms;
            if (m - gpsData.state_ts < 200)
                return;

            // but print our FIXED init string for the baudrate we want to be at
            if (pifGpsUblox_SetPubxConfig(&gps_ublox, 1, 0x07, 0x03, gpsInitData[gpsData.baudrateIndex].baudrate, FALSE)) {
                gpsData.state_position++;
                gpsData.state = GPS_INITIALIZING;
                gpsData.state_ts = m;
            }
            else {
        		gpsSetState(GPS_INITIALIZING);
            }
            break;

        case GPS_SETBAUD:
            m = pif_cumulative_timer1ms;
            if (m - gpsData.state_ts < GPS_BAUD_DELAY)
                return;

            serialSetBaudRate(core.gpsport, gpsInitData[gpsData.baudrateIndex].baudrate);
        	serialStartReceiveFunc(&core.gpsport->comm);
            gpsSetState(GPS_CONFIGURATION);
            break;

        case GPS_CONFIGURATION:
            gpsInitHardware();
            break;

        case GPS_LOSTCOMMS:
            if (mcfg.gps_type == GPS_NMEA && gpsInitData[gpsData.baudrateIndex].baudrate == 9600) {
                pifGps_SetTimeout(&gps_nmea._gps, &g_timer_1ms, 0, NULL);
            }
            else {
                pifGps_SetTimeout(&gps_ublox._gps, &g_timer_1ms, 0, NULL);
            }
            gpsData.errors++;
            // try another rate (Only if autobauding is enabled)
            if (mcfg.gps_autobaud) {
                gpsData.baudrateIndex++;
                gpsData.baudrateIndex %= GPS_INIT_ENTRIES;
            }
            gpsData.lastMessage = pif_cumulative_timer1ms;
            // TODO - move some / all of these into gpsData
            GPS_numSat = 0;
            f.GPS_FIX = 0;
            gpsSetState(GPS_INITIALIZING);
            break;

        case GPS_RECEIVINGDATA:
            break;
    }
}

// gpsPollSvinfo-function. Used for polling UBX-NAV-SVINFO (0x01 0x30) information from GPS.
void gpsPollSvinfo(void)
{
    // If selected GPS isn't UBLOX then we don't poll UBX messages.
    if (mcfg.gps_type != GPS_UBLOX)
        return;

    GPS_numCh = gps_ublox._num_ch;
    for (int i = 0; i < GPS_numCh; i++) {
        GPS_svinfo_chn[i] = gps_ublox._svinfo_chn[i];
        GPS_svinfo_svid[i] = gps_ublox._svinfo_svid[i];
        GPS_svinfo_quality[i] = gps_ublox._svinfo_quality[i];
        GPS_svinfo_cno[i] = gps_ublox._svinfo_cno[i];
    }
    // Update GPS SVIFO update rate table.
    GPS_svinfo_rate[0] = gps_ublox._svinfo_rate[0];
    GPS_svinfo_rate[1] = gps_ublox._svinfo_rate[1];

    pifGpsUblox_SendUbxMsg(&gps_ublox, GUCI_NAV, GUMI_NAV_SVINFO, 0, NULL, FALSE);
}


/*-----------------------------------------------------------
 *
 * Multiwii GPS code - revision: 1097
 *
 *-----------------------------------------------------------*/
#define POSHOLD_IMAX           20       // degrees
#define POSHOLD_RATE_IMAX      20       // degrees
#define NAV_IMAX               20       // degrees

/* GPS navigation can control the heading */
#define NAV_TAIL_FIRST             0    // true - copter comes in with tail first
#define NAV_SET_TAKEOFF_HEADING    1    // true - when copter arrives to home position it rotates it's head to takeoff direction

#define GPS_FILTERING              1    // add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency
#define GPS_LOW_SPEED_D_FILTER     1    // below .5m/s speed ignore D term for POSHOLD_RATE, theoretically this also removed D term induced noise

static bool check_missed_wp(void);
static void GPS_distance_cm_bearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, int32_t *dist, int32_t *bearing);
//static void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing);
static void GPS_calc_longitude_scaling(int32_t lat);
static void GPS_calc_velocity(void);
static void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng);
static void GPS_calc_poshold(void);
static void GPS_calc_nav_rate(int max_speed);
static void GPS_update_crosstrack(void);
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow);
int32_t wrap_18000(int32_t err);
static int32_t wrap_36000(int32_t deg);

typedef struct {
    int16_t last_velocity;
} LeadFilter_PARAM;

void leadFilter_clear(LeadFilter_PARAM *param)
{
    param->last_velocity = 0;
}

int32_t leadFilter_getPosition(LeadFilter_PARAM *param, int32_t pos, int16_t vel, float lag_in_seconds)
{
    int16_t accel_contribution = (vel - param->last_velocity) * lag_in_seconds * lag_in_seconds;
    int16_t vel_contribution = vel * lag_in_seconds;

    // store velocity for next iteration
    param->last_velocity = vel;

    return pos + vel_contribution + accel_contribution;
}

LeadFilter_PARAM xLeadFilter;
LeadFilter_PARAM yLeadFilter;

static PID_PARAM posholdPID_PARAM;
static PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;
PID_PARAM altPID_PARAM;

typedef struct {
    float integrator;          // integrator value
    int32_t last_input;        // last input for derivative
    float last_derivative;     // last derivative for low-pass filter
    float output;
    float derivative;
} PID;

static PID posholdPID[2];
static PID poshold_ratePID[2];
static PID navPID[2];

static int32_t get_P(int32_t error, PID_PARAM *pid)
{
    return (float)error * pid->kP;
}

static int32_t get_I(int32_t error, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->integrator += ((float)error * pid_param->kI) **dt;
    pid->integrator = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

static int32_t get_D(int32_t input, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->derivative = (input - pid->last_input) / *dt;

    // Low pass filter cut frequency for derivative calculation
    // Set to  "1 / ( 2 * PI * gps_lpf )"
#define PID_FILTER       (1.0f / (2.0f * M_PI * (float)cfg.gps_lpf))
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->last_derivative + (*dt / (PID_FILTER + *dt)) * (pid->derivative - pid->last_derivative);
    // update state
    pid->last_input = input;
    pid->last_derivative = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

static void reset_PID(PID *pid)
{
    pid->integrator = 0;
    pid->last_input = 0;
    pid->last_derivative = 0;
}

#define GPS_X 1
#define GPS_Y 0

/****************** PI and PID controllers for GPS ********************///32938 -> 33160

#define RADX100                    0.000174532925f
#define CROSSTRACK_GAIN            1
#define NAV_SLOW_NAV               true
#define NAV_BANK_MAX               3000 // 30deg max banking when navigating (just for security and testing)

static float dTnav;             // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static int16_t actual_speed[2] = { 0, 0 };
float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t rate_error[2];
static int32_t error[2];

// Currently used WP
int32_t GPS_WP[2];

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t target_bearing;
////////////////////////////////////////////////////////////////////////////////
// Crosstrack
////////////////////////////////////////////////////////////////////////////////
// deg * 100, The original angle to the next_WP when the next_WP was set
// Also used to check when we pass a WP
static int32_t original_target_bearing;
// The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
static int16_t crosstrack_error;
////////////////////////////////////////////////////////////////////////////////
// The location of the copter in relation to home, updated every GPS read (1deg - 100)
//static int32_t home_to_copter_bearing;
// distance between plane and home in cm
//static int32_t home_distance;
// distance between plane and next_WP in cm
int32_t wp_distance;

// used for slow speed wind up when start navigation;
static int16_t waypoint_speed_gov;

////////////////////////////////////////////////////////////////////////////////////
// moving average filter variables
//
#define GPS_FILTER_VECTOR_LENGTH 5

static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];   //the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];

// This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
int32_t nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;

uint16_t taskGpsNewData(PifTask *p_task)
{
    int axis;
    static uint32_t nav_loopTimer;
    int32_t dist;
    int32_t dir;
    int16_t speed;

    (void)p_task;

        // new data received and parsed, we're in business
        gpsData.lastMessage = pif_cumulative_timer1ms;
        sensorsSet(SENSOR_GPS);
        if (GPS_update == 1)
            GPS_update = 0;
        else
            GPS_update = 1;
        if (f.GPS_FIX && GPS_numSat >= 5) {
            if (!f.ARMED && !f.FIXED_WING)
                f.GPS_FIX_HOME = 0;
            if (!f.GPS_FIX_HOME && f.ARMED)
                GPS_reset_home_position();
            // Apply moving average filter to GPS data
#if defined(GPS_FILTERING)
            GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
            for (axis = 0; axis < 2; axis++) {
                GPS_read[axis] = GPS_coord[axis];               // latest unfiltered data is in GPS_latitude and GPS_longitude
                GPS_degree[axis] = GPS_read[axis] / 10000000;   // get the degree to assure the sum fits to the int32_t

                // How close we are to a degree line ? its the first three digits from the fractions of degree
                // later we use it to Check if we are close to a degree line, if yes, disable averaging,
                fraction3[axis] = (GPS_read[axis] - GPS_degree[axis] * 10000000) / 10000;

                GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
                GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis] * 10000000);
                GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
                GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis] * 10000000);
                if (nav_mode == NAV_MODE_POSHOLD) {             // we use gps averaging only in poshold mode...
                    if (fraction3[axis] > 1 && fraction3[axis] < 999)
                        GPS_coord[axis] = GPS_filtered[axis];
                }
            }
#endif
            // dTnav calculation
            // Time for calculating x,y speed and navigation pids
            dTnav = (float)(pif_cumulative_timer1ms - nav_loopTimer) / 1000.0f;
            nav_loopTimer = pif_cumulative_timer1ms;
            // prevent runup from bad GPS
            dTnav = min(dTnav, 1.0f);

            // calculate distance and bearings for gui and other stuff continously - From home to copter
            GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
            GPS_distanceToHome = dist / 100;
            GPS_directionToHome = dir / 100;

            if (!f.GPS_FIX_HOME) {      // If we don't have home set, do not display anything
                GPS_distanceToHome = 0;
                GPS_directionToHome = 0;
            }

            // calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
            GPS_calc_velocity();

            if (f.GPS_HOLD_MODE || f.GPS_HOME_MODE) { // ok we are navigating
                // do gps nav calculations here, these are common for nav and poshold
                GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
                GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);

                if (f.FIXED_WING)
                    nav_mode = NAV_MODE_WP; // Planes always navigate in Wp mode.

                switch (nav_mode) {
                    case NAV_MODE_POSHOLD:
                        // Desired output is in nav_lat and nav_lon where 1deg inclination is 100
                        GPS_calc_poshold();
                        break;

                    case NAV_MODE_WP:
                        speed = GPS_calc_desired_speed(cfg.nav_speed_max, NAV_SLOW_NAV);    // slow navigation
                        // use error as the desired rate towards the target
                        // Desired output is in nav_lat and nav_lon where 1deg inclination is 100
                        GPS_calc_nav_rate(speed);

                        // Tail control
                        if (cfg.nav_controls_heading) {
                            if (NAV_TAIL_FIRST) {
                                magHold = wrap_18000(nav_bearing - 18000) / 100;
                            } else {
                                magHold = nav_bearing / 100;
                            }
                        }
                        // Are we there yet ?(within x meters of the destination)
                        if ((wp_distance <= cfg.gps_wp_radius) || check_missed_wp()) {      // if yes switch to poshold mode
                            nav_mode = NAV_MODE_POSHOLD;
                            if (NAV_SET_TAKEOFF_HEADING) {
                                magHold = nav_takeoff_bearing;
                            }
                        }
                        break;
                }
            }                   //end of gps calcs
        }

    return 0;
}

void GPS_reset_home_position(void)
{
    if (f.GPS_FIX && GPS_numSat >= 5) {
        GPS_home[LAT] = GPS_coord[LAT];
        GPS_home[LON] = GPS_coord[LON];
        GPS_calc_longitude_scaling(GPS_coord[LAT]); // need an initial value for distance and bearing calc
        nav_takeoff_bearing = heading;              // save takeoff heading
        //Set ground altitude
        GPS_home[ALT] = GPS_altitude;
        f.GPS_FIX_HOME = 1;
    }
}

// reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav(void)
{
    int i;

    for (i = 0; i < 2; i++) {
        GPS_angle[i] = 0;
        nav_rated[i] = 0;
        nav[i] = 0;
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }

    if (f.FIXED_WING)
        fw_nav_reset();

}

// Get the relevant P I D values and set the PID controllers
void gpsSetPIDs(void)
{
    posholdPID_PARAM.kP = (float)cfg.P8[PIDPOS] / 100.0f;
    posholdPID_PARAM.kI = (float)cfg.I8[PIDPOS] / 100.0f;
    posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    poshold_ratePID_PARAM.kP = (float)cfg.P8[PIDPOSR] / 10.0f;
    poshold_ratePID_PARAM.kI = (float)cfg.I8[PIDPOSR] / 100.0f;
    poshold_ratePID_PARAM.kD = (float)cfg.D8[PIDPOSR] / 1000.0f;
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    navPID_PARAM.kP = (float)cfg.P8[PIDNAVR] / 10.0f;
    navPID_PARAM.kI = (float)cfg.I8[PIDNAVR] / 100.0f;
    navPID_PARAM.kD = (float)cfg.D8[PIDNAVR] / 1000.0f;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    if (f.FIXED_WING) {
        altPID_PARAM.kP   = (float)cfg.P8[PIDALT] / 10.0f;
        altPID_PARAM.kI   = (float)cfg.I8[PIDALT] / 100.0f;
        altPID_PARAM.kD   = (float)cfg.D8[PIDALT] / 1000.0f;
    }
}

static void _EvtPrintFrame(char* p_frame)
{
    actLed0State(ON);
#ifndef __PIF_NO_LOG__
    pifLog_Print(LT_NONE, p_frame);
#else
    (void)p_frame;
#endif
    actLed0State(OFF);
}

int8_t gpsSetPassthrough(void)
{
    static bool state = false;
	PifGps* p_gps;

    if (gpsData.state != GPS_RECEIVINGDATA)
        return -1;

    actLed0State(OFF);
    actLed1State(OFF);

    if (mcfg.gps_type == GPS_NMEA) {
		p_gps = (gpsInitData[gpsData.baudrateIndex].baudrate == 9600) ? &gps_nmea._gps : &gps_ublox._gps;
        if (state) {
            p_gps->evt_frame = NULL;
            state = false;
            return 0;
        }
        else {
            p_gps->evt_frame = _EvtPrintFrame;
            state = true;
            return 1;
        }
    }
    return -1;
}

// OK here is the onboard GPS code

////////////////////////////////////////////////////////////////////////////////////
// PID based GPS navigation functions
// Author : EOSBandi
// Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
// Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
static void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (abs((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cosf(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t *lat, int32_t *lon)
{
    GPS_WP[LAT] = *lat;
    GPS_WP[LON] = *lon;

    GPS_calc_longitude_scaling(*lat);
    if (f.CRUISE_MODE)
        fw_FlyTo();  // PatrikE CruiseMode version

    GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);

    nav_bearing = target_bearing;
    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
    original_target_bearing = target_bearing;
    waypoint_speed_gov = cfg.nav_speed_min;
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void)
{
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (abs(temp) > 10000); // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
static void GPS_distance_cm_bearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, int32_t *dist, int32_t *bearing)
{
    float dLat = *lat2 - *lat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * 1.113195f;

    *bearing = 9000.0f + atan2f(-dLat, dLon) * 5729.57795f;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

////////////////////////////////////////////////////////////////////////////////////
// keep old calculation function for compatibility (could be removed later) distance in meters, bearing in degree
//
//static void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
//  uint32_t d1;
//  int32_t  d2;
//  GPS_distance_cm_bearing(&lat1,&lon1,&lat2,&lon2,&d1,&d2);
//  *dist = d1 / 100;          //convert to meters
//  *bearing = d2 /  100;      //convert to degrees
//}

////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps position data
//
static void GPS_calc_velocity(void)
{
    static int16_t speed_old[2] = { 0, 0 };
    static int32_t last[2] = { 0, 0 };
    static uint8_t init = 0;
    // y_GPS_speed positve = Up
    // x_GPS_speed positve = Right

    if (init) {
        float tmp = 1.0f / dTnav;
        actual_speed[GPS_X] = (float)(GPS_coord[LON] - last[LON]) * GPS_scaleLonDown * tmp;
        actual_speed[GPS_Y] = (float)(GPS_coord[LAT] - last[LAT]) * tmp;

        actual_speed[GPS_X] = (actual_speed[GPS_X] + speed_old[GPS_X]) / 2;
        actual_speed[GPS_Y] = (actual_speed[GPS_Y] + speed_old[GPS_Y]) / 2;

        speed_old[GPS_X] = actual_speed[GPS_X];
        speed_old[GPS_Y] = actual_speed[GPS_Y];
    }
    init = 1;

    last[LON] = GPS_coord[LON];
    last[LAT] = GPS_coord[LAT];
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//      100     = 1m
//      1000    = 11m    = 36 feet
//      1800    = 19.80m = 60 feet
//      3000    = 33m
//      10000   = 111m
//
static void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng)
{
    error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;   // X Error
    error[LAT] = *target_lat - *gps_lat;        // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold(void)
{
    int32_t d;
    int32_t target_speed;
    int axis;

    for (axis = 0; axis < 2; axis++) {
        target_speed = get_P(error[axis], &posholdPID_PARAM);       // calculate desired speed from lon error
        rate_error[axis] = target_speed - actual_speed[axis];       // calc the speed error

        nav[axis] = get_P(rate_error[axis], &poshold_ratePID_PARAM) +
                    get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = get_D(error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = constrain(d, -2000, 2000);

        // get rid of noise
#if defined(GPS_LOW_SPEED_D_FILTER)
        if (abs(actual_speed[axis]) < 50)
            d = 0;
#endif

        nav[axis] += d;
        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        navPID[axis].integrator = poshold_ratePID[axis].integrator;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(int max_speed)
{
    float trig[2];
    float temp;
    int axis;

    // push us towards the original track
    GPS_update_crosstrack();

    // nav_bearing includes crosstrack
    temp = (9000l - nav_bearing) * RADX100;
    trig[GPS_X] = cosf(temp);
    trig[GPS_Y] = sinf(temp);

    for (axis = 0; axis < 2; axis++) {
        rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        // P + I + D
        nav[axis] = get_P(rate_error[axis], &navPID_PARAM) +
                    get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM) +
                    get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void)
{
    if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {     // If we are too far off or too close we don't do track following
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * (wp_distance * CROSSTRACK_GAIN); // Meters we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    } else {
        nav_bearing = target_bearing;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow
// speed rampup when starting a navigation
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1.5 m/s as we hit the target
//
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow)
{
    // max_speed is default 400 or 4m/s
    if (_slow) {
        max_speed = min(max_speed, wp_distance / 2);
    } else {
        max_speed = min(max_speed, wp_distance);
        max_speed = max(max_speed, cfg.nav_speed_min);      // go at least 100cm/s
    }

    // limit the ramp up of the speed
    // waypoint_speed_gov is reset to 0 at each new WP command
    if (max_speed > waypoint_speed_gov) {
        waypoint_speed_gov += (int)(100.0f * dTnav);    // increase at .5/ms
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//
int32_t wrap_18000(int32_t err)
{
    if (err > 18000)
        err -= 36000;
    if (err < -18000)
        err += 36000;
    return err;
}

static int32_t wrap_36000(int32_t deg)
{
    if (deg > 36000)
        deg -= 36000;
    if (deg < 0)
        deg += 36000;
    return deg;
}

#else

void gpsInit(uint8_t port, uint8_t baudrateIndex)
{
	(void)port;
	(void)baudrateIndex;
}

void gpsThread(void)
{

}

#endif /* GPS */
