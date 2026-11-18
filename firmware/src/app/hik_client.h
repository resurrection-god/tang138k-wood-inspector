/*
 * hik_client.h - Autonomous GigE Vision (GVCP/GVSP) client for Hikrobot cameras.
 *
 * Discovers a Hikrobot GigE Vision camera on the local subnet, configures
 * MTU-safe GVSP streaming (SCPS=1440, SCPD=120 us) directly to the board,
 * downscales Mono8 frames on the fly to 200x120 RGB332 with Bayer 2x2 dither
 * and optional auto-contrast, and submits them to lcd_hud.
 */

#ifndef APP_HIK_CLIENT_H_
#define APP_HIK_CLIENT_H_

#include <stdbool.h>
#include <stdint.h>

#define HIK_GVSP_PORT           5556U
#define HIK_GVSP_HW_PORT        5557U   /* Hardware RTL gvsp_rx_hdmi port */
#define HIK_GVSP_SCPS_BYTES     1440U   /* <= 1536 B MAC RX window (rx_jumbo_ena=0) */
#define HIK_GVSP_SCPD_TICKS     12000U  /* 120 us @ 100 MHz tick_freq (SW RX MAC safety) */
#define HIK_GVSP_HW_SCPD_TICKS  600U    /* 6 us @ 100 MHz tick_freq (60 FPS pacing + zero burst loss) */

typedef enum {
    HIK_STATE_DISABLED  = 0,
    HIK_STATE_DISCOVER  = 1,
    HIK_STATE_CCP_BUSY  = 2,
    HIK_STATE_CONFIG    = 3,
    HIK_STATE_STREAMING = 4
} hik_state_t;

typedef struct {
    hik_state_t state;
    bool        enabled;
    bool        decim_2x2;
    bool        auto_contrast;
    uint32_t    cam_ip;
    uint32_t    cam_w;
    uint32_t    cam_h;
    uint32_t    exposure_us;
    uint32_t    frames_ok;
    uint32_t    frames_incomplete;
    uint32_t    pkts_rx;
    uint32_t    fps;
} hik_stats_t;

void hik_client_init(void);
void hik_client_set_enabled(bool en);
bool hik_client_is_enabled(void);
void hik_client_toggle_enabled(void);
void hik_client_toggle_decim(void);
void hik_client_toggle_auto_contrast(void);
void hik_client_adjust_exposure(int32_t delta_us);
void hik_client_get_stats(hik_stats_t *out);
void hik_client_print_status(void);

#endif /* APP_HIK_CLIENT_H_ */
