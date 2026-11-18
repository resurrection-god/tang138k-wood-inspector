/*
 * cam_trig.h - camera hardware trigger generator (mailbox TRIG block)
 *
 * Part of net_stack.h until 24.09.2026.
 */

#ifndef APP_CAM_TRIG_H_
#define APP_CAM_TRIG_H_

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "memory_map.h"

// ---- Camera hardware trigger ----------------------------------------------
//
// Words 0..5 of the mailbox MAILBOX_ADDR (memory_map.h).
#define CAM_TRIG_MAGIC          0x54524947   /* "TRIG" */

#define CAM_TRIG_W_MAGIC        0
#define CAM_TRIG_W_PERIOD       1            /* period in lane4_clk clocks     */
#define CAM_TRIG_W_COUNT        2            /* 1..64 pulses; 0 = continuous   */
#define CAM_TRIG_W_WIDTH        3            /* width in clocks                */
#define CAM_TRIG_W_FLAGS        4            /* bit0 = invert, bit1 = stop     */
#define CAM_TRIG_W_SEQ          5            /* command counter                */

#define CAM_TRIG_FLAG_INVERT    0x00000001u
#define CAM_TRIG_FLAG_STOP      0x00000002u

#define CAM_TRIG_COUNT_MAX      64
#define CAM_TRIG_FREQ_MIN_CHZ   50           /* 0.5 Hz, in hundredths of a Hz  */
#define CAM_TRIG_FREQ_MAX_CHZ   120000       /* 1200 Hz                        */

// 0 means the parameters were accepted and went into the mailbox. Then the
// pulses will go to the camera and it will start writing the ring: the caller
// MUST reset the capture arming at once (video_invalidate_trigger_arming),
// under the same ring lock if it holds it.
int  cam_trig_apply(uint32_t freq_chz, uint32_t count, uint32_t width_us,
                    uint32_t flags);
int  cam_trig_stop(void);
void cam_trig_get(uint32_t *freq_chz, uint32_t *count, uint32_t *width_us);
bool cam_trig_running(void);   // started by cam_trig_apply, not stopped since

#endif /* APP_CAM_TRIG_H_ */
