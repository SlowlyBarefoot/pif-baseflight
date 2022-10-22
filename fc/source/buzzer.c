/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"
#include "buzzer.h"

#include "sound/pif_buzzer.h"

/* Buzzer Sound Sequences: (Square wave generation)
 * Sequence must end with 0xFF or 0xFE. 0xFE repeats the sequence from
 * start when 0xFF stops the sound when it's completed.
 * If repeat is used then BUZZER_STOP call must be used for stopping the sound.
 *
 * "Sound" Sequences are made so that 1st, 3rd, 5th.. are the delays how
 * long the buzzer is on and 2nd, 4th, 6th.. are the delays how long buzzer
 * is off.
 */
// fast beep:
static const uint8_t buzz_shortBeep[] = {
    5, 5, 0xF0
};
// fast beep:
static const uint8_t buzz_BatteryBeep[] = {
    10, 5, 0xF0
};
// medium beep
static const uint8_t buzz_mediumBeepFast[] = {
    35, 30, 0xF0
};
// medium beep and pause after that
static const uint8_t buzz_mediumBeep[] = {
    35, 150, 0xF0
};
// Long beep and pause after that
static const uint8_t buzz_longBeep[] = {
    70, 200, 0xF0
};
// SOS morse code:
static const uint8_t buzz_sos[] = {
    10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, 0xF0
};
// Arming when GPS is fixed
static const uint8_t buzz_armed[] = {
    5, 5, 15, 5, 5, 5, 15, 30, 0xF0
};
// Ready beeps. When gps has fix and copter is ready to fly.
static const uint8_t buzz_readyBeep[] = {
    4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, 0xF0
};
// 2 fast short beeps
static const uint8_t buzz_2shortBeeps[] = {
    5, 5, 5, 5, 0xF0
};
// 3 fast short beeps
static const uint8_t buzz_3shortBeeps[] = {
    5, 5, 5, 5, 5, 5, 0xF0
};
#ifdef GPS
// Array used for beeps when reporting GPS satellite count (up to 10 satellites)
static uint8_t buzz_countSats[22];
#endif

// Current Buzzer mode
static uint8_t buzzerMode = BUZZER_STOPPED;
// Variable for checking if ready beep has been done
static uint8_t readyBeepDone = 0;

PifBuzzer s_buzzer;

static void _evtBuzzerFinish(PifId id)
{
	(void)id;

    buzzerMode = BUZZER_STOPPED;
}

bool buzzerInit()
{
    if (!pifBuzzer_Init(&s_buzzer, PIF_ID_BUZZER, actBuzzerAction)) return false;
    s_buzzer.evt_finish = _evtBuzzerFinish;
    return true;
}

/* Buzzer -function is used to activate/deactive buzzer.
 * Parameter defines the used sequence.
 */
void buzzer(uint8_t mode)
{
#ifdef GPS
    uint8_t i = 0;
#endif

    // Just return if same or higher priority sound is active.
    if (buzzerMode <= mode)
        return;

    switch (mode) {
        case BUZZER_STOP:
            buzzerMode = BUZZER_STOPPED;
            pifBuzzer_Stop(&s_buzzer);
            break;
        case BUZZER_READY_BEEP:
            if (readyBeepDone)
                return;
            pifBuzzer_Start(&s_buzzer, buzz_readyBeep);
            buzzerMode = mode;
            readyBeepDone = 1;
            break;
        case BUZZER_ARMING:
        case BUZZER_DISARMING:
            pifBuzzer_Start(&s_buzzer, buzz_mediumBeepFast);
            buzzerMode = mode;
            break;
        case BUZZER_ACC_CALIBRATION:
            pifBuzzer_Start(&s_buzzer, buzz_2shortBeeps);
            buzzerMode = mode;
            break;
        case BUZZER_ACC_CALIBRATION_FAIL:
            pifBuzzer_Start(&s_buzzer, buzz_3shortBeeps);
            buzzerMode = mode;
            break;
        case BUZZER_TX_LOST_ARMED:
            pifBuzzer_Start(&s_buzzer, buzz_sos);
            buzzerMode = mode;
            break;
        case BUZZER_BAT_LOW:
            pifBuzzer_Start(&s_buzzer, buzz_longBeep);
            buzzerMode = mode;
            break;
        case BUZZER_BAT_CRIT_LOW:
            pifBuzzer_Start(&s_buzzer, buzz_BatteryBeep);
            buzzerMode = mode;
            break;
        case BUZZER_ARMED:
        case BUZZER_TX_LOST:
            pifBuzzer_Start(&s_buzzer, buzz_mediumBeep);
            buzzerMode = mode;
            break;
        case BUZZER_ARMING_GPS_FIX:
            pifBuzzer_Start(&s_buzzer, buzz_armed);
            buzzerMode = mode;
            break;
        case BUZZER_TX_SET:
#ifdef GPS
            if (feature(FEATURE_GPS) && f.GPS_FIX && GPS_numSat >= 5) {
                do {
                    buzz_countSats[i] = 5;
                    buzz_countSats[i + 1] = 15;
                    i += 2;
                } while (i < 20 && GPS_numSat > i / 2);
                buzz_countSats[i + 1] = 100;
                buzz_countSats[i + 2] = 0xF0;
                pifBuzzer_Start(&s_buzzer, buzz_countSats);
                buzzerMode = mode;
                break;
            }
#endif
            pifBuzzer_Start(&s_buzzer, buzz_shortBeep);
            buzzerMode = mode;
            break;

        default:
            return;
    }
}

/* buzzerUpdate -function is used in loop. It will update buzzer state
 * when the time is correct.
 */
void buzzerUpdate(void)
{
    // If beeper option from AUX switch has been selected
    if (rcOptions[BOXBEEPERON]) {
        if (buzzerMode > BUZZER_TX_SET)
            buzzer(BUZZER_TX_SET);
    }
}
