/*
 * app_sound.c - Motor Sound Generation
 *
 * Plays tones through the motor coils (VESC-style): a queued tone is emitted
 * by injecting a high-frequency voltage via the FOC inverter (the mcpwm
 * backend hook mcpwm_sound_tone). Supports single beeps, multi-note
 * melodies and a boot startup sound. Non-blocking: app_sound_process()
 * advances the active tone each tick.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#include "app.h"
#include "driver/mcu/mcu_init.h"

/* ======================================================================== */
/* Platform Hook (declared weak in app.c)                                    */
/* ======================================================================== */
/* Injects a high-frequency voltage tone of freq_hz at the given amplitude
 * (fraction of vbus). freq_hz == 0 silences the output. Implemented in
 * app.c as a weak stub; the real FOC layer overrides it. */
void mcpwm_sound_tone(float freq_hz, float amplitude);

/* ======================================================================== */
/* Tone Queue                                                               */
/* ======================================================================== */
#define SOUND_QUEUE_LEN   16U
#define SOUND_AMPLITUDE   0.04f    /* ~4% of vbus, audible but gentle       */
#define SOUND_SILENCE_GAP 30U      /* ms gap inserted between notes        */

typedef struct {
    uint16_t freq;          /* Hz, 0 == silence                          */
    uint16_t duration;     /* ms                                        */
} sound_note_t;

static sound_note_t s_queue[SOUND_QUEUE_LEN];
static uint8_t      s_head;            /* next slot to write                */
static uint8_t      s_tail;            /* next note to play                 */
static uint16_t     s_note_remain_ms;  /* ms left in current note           */
static uint32_t     s_last_ms;

/* ======================================================================== */
/* Local Helpers                                                             */
/* ======================================================================== */
static bool queue_empty(void) { return s_head == s_tail; }
static bool queue_full(void)  { return (uint8_t)(s_head + 1U) == s_tail; }

static void queue_push(uint16_t freq, uint16_t duration_ms)
{
    if (queue_full()) { return; }
    s_queue[s_head].freq     = freq;
    s_queue[s_head].duration  = duration_ms;
    s_head = (uint8_t)(s_head + 1U);
}

static void silence_now(void)
{
    mcpwm_sound_tone(0.0f, 0.0f);
}

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */
void app_sound_init(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_note_remain_ms = 0U;
    s_last_ms = mcu_millis();
    silence_now();
}

void app_sound_play(uint16_t freq, uint16_t duration_ms)
{
    queue_push(freq, duration_ms);
}

void app_sound_startup(void)
{
    /* Two-note rising "boot" chirp. */
    queue_push(880U,  120U);
    queue_push(0U,    SOUND_SILENCE_GAP);
    queue_push(1320U, 160U);
}

void app_sound_process(void)
{
    uint32_t now = mcu_millis();
    uint32_t elapsed = now - s_last_ms;
    s_last_ms = now;

    if (elapsed > s_note_remain_ms) {
        /* Current note finished: advance to the next one. */
        if (queue_empty()) {
            if (s_note_remain_ms != 0U) {
                silence_now();
                s_note_remain_ms = 0U;
            }
            return;
        }
        sound_note_t note = s_queue[s_tail];
        s_tail = (uint8_t)(s_tail + 1U);
        s_note_remain_ms = note.duration;
        if (note.freq == 0U) {
            silence_now();
        } else {
            mcpwm_sound_tone((float)note.freq, SOUND_AMPLITUDE);
        }
    } else {
        s_note_remain_ms = (uint16_t)(s_note_remain_ms - elapsed);
    }
}
