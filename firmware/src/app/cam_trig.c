/*
 * cam_trig.c - camera hardware trigger generator: mailbox words 0..5
 *
 * Part of net_stack.c until 24.09.2026.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "board.h"
#include "mailbox.h"
#include "cam_trig.h"
#include "cache.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"



// ---- Camera hardware trigger ----------------------------------------------
//
// We write the parameter block into DDR3; the FPGA reads it over Lane 5 about
// every 10 ms and starts a burst when it sees a new command number.
static uint32_t s_trig_seq      = 0;
static uint32_t s_trig_freq_chz = 1000;   /* 10 Hz   */
static uint32_t s_trig_count    = 1;
static uint32_t s_trig_width_us = 100;

static bool s_trig_running;                // a series was started and not stopped

bool cam_trig_running(void)
{
    return s_trig_running;
}

void cam_trig_get(uint32_t *freq_chz, uint32_t *count, uint32_t *width_us)
{
    if (freq_chz) *freq_chz = s_trig_freq_chz;
    if (count)    *count    = s_trig_count;
    if (width_us) *width_us = s_trig_width_us;
}

static int cam_trig_write(uint32_t period, uint32_t count, uint32_t width,
                          uint32_t flags)
{
    mailbox_put(CAM_TRIG_W_MAGIC,  CAM_TRIG_MAGIC);
    mailbox_put(CAM_TRIG_W_PERIOD, period);
    mailbox_put(CAM_TRIG_W_COUNT,  count);
    mailbox_put(CAM_TRIG_W_WIDTH,  width);
    mailbox_put(CAM_TRIG_W_FLAGS,  flags);
    mailbox_publish(CAM_TRIG_W_SEQ, ++s_trig_seq);   // order: mailbox.h
    return 0;
}

int cam_trig_apply(uint32_t freq_chz, uint32_t count, uint32_t width_us,
                   uint32_t flags)
{
    uint64_t period;
    uint32_t width;

    if (freq_chz < CAM_TRIG_FREQ_MIN_CHZ || freq_chz > CAM_TRIG_FREQ_MAX_CHZ)
        return -1;
    if (count > CAM_TRIG_COUNT_MAX)
        return -2;
    if (width_us == 0)
        return -3;

    // Frequency in hundredths of a hertz, so that 0.5 Hz is an integer.
    // Period in clocks = lane4_clk clocks per 100 s / freq_chz (board.h; until
    // 29.09.2026 400/3 MHz was hard-coded here); 64 bits are mandatory.
    period = LANE_US_TO_TICKS(100000000ULL) / (uint64_t)freq_chz;
    width  = (uint32_t)LANE_US_TO_TICKS(width_us);

    if (width == 0)      width = 1;
    if (width >= period) return -4;    /* a pulse wider than the period is nonsense */

    s_trig_freq_chz = freq_chz;
    s_trig_running  = (flags & CAM_TRIG_FLAG_STOP) == 0U;
    s_trig_count    = count;
    s_trig_width_us = width_us;

    // The pulses will now go to the camera and it will start writing the ring: the
    // caller resets the capture arming (cam_trig.h), the generator knows nothing
    // about capture.
    return cam_trig_write((uint32_t)period, count, width, flags);
}

int cam_trig_stop(void)
{
    s_trig_running = false;
    return cam_trig_write(1000, 0, 1, CAM_TRIG_FLAG_STOP);
}
