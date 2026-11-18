/*
 * work_mode.h - operating modes STANDBY / WORK / SERVICE (0x0110)
 *
 * Part of net_stack.h until 24.09.2026.
 */

#ifndef APP_WORK_MODE_H_
#define APP_WORK_MODE_H_

#include <stdint.h>
#include <stdbool.h>
#include "camera.h"
#include "settings.h"

// ---- System operating modes (0x0110) ---------------------------------------
//
// Design: docs/ARCHITECTURE.md § 7. Until 18.09.2026 the mode was a variable in
// vmsp_service.c and only forbade frame delivery in STANDBY.
//
// What the mode does now:
//   STANDBY  the laser is turned off, the sync block disabled, the burst
//            generator stopped; capture and delivery answer DENIED. It has no
//            settings set of its own: it is a fixed safe rest state;
//   SERVICE  the block is enabled, the SERVICE SETTINGS SET is applied (cameras,
//            synchroniser, line finder);
//   WORK     the same with its own WORK set, plus 0x0210 is allowed; a frame is
//            written only if the line finder setting has LASER_CFG_F_KEEP_FRAME.
//
// Sets (23.09.2026, docs/ARCHITECTURE.md § 7, docs/DECISIONS.md R-12).
// Commands 0x0112/0x0120/0x0140/0x0142/0x0150 change the CURRENT state; on
// leaving WORK or SERVICE it is remembered as that mode's set, on entering the
// new mode's set is applied. Both sets are stored in the store (settings_save()).
//
// Laser: the enable (laser_enable) is not part of a set and is not saved; each
// mode has its own and it lives until reboot. A transition never lights the
// laser: it only gives the mode back the enable the operator gave in that same
// mode in this session. After boot it is off in all modes.
#define WORK_MODE_STANDBY       0
#define WORK_MODE_WORK          1
#define WORK_MODE_SERVICE       2

#define WORK_MODE_F_FPGA_ACK    0x01   /* the FPGA confirmed the mode by an echo in telemetry */
#define WORK_MODE_F_LASER_EN    0x02   /* laser enabled as a result of the transition */
#define WORK_MODE_F_FPGA_LASR   0x04   /* the bitstream has the LASR block       */
#define WORK_MODE_F_CAM_STOP    0x08   /* sensor capture stopped over I2C        */
#define WORK_MODE_F_CAM_CFG     0x10   /* camera reconfigured from the mode set  */
#define WORK_MODE_F_CAM_FAIL    0x20   /* camera reconfiguration failed          */

uint8_t work_mode_get(void);
// The whole transition, under video_ring_lock(). Returns a VCP_ST_* code (0 = OK);
// mode_actual is the mode the system actually ended up in.
int     work_mode_set(uint8_t mode, uint8_t *mode_actual, uint8_t *flags);


// Mode set index: [0] WORK, [1] SERVICE (settings.c).
#define MODE_SET_IDX(mode)  (((mode) == WORK_MODE_WORK) ? 0U : 1U)


// Camera settings from the SERVICE set for camera_boot_reset() (main.c).
// Fills exp, gain, fps, testpat and ROI.
void    work_mode_boot_cam(uint8_t cam_index, camera_params_t *p);

// The "settings" storage partition (settings.c): main() hooks it up.
extern const settings_section_t work_mode_settings_section;
bool    work_mode_settings_writable(void);

#endif /* APP_WORK_MODE_H_ */
