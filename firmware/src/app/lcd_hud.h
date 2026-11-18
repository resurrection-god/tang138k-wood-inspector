/*
 * lcd_hud.h - APB RGB666 LCD Framebuffer, FPGA NPU Classifier & UDP Control Driver.
 *
 * Drives 200x120 RGB332 framebuffer (4x upscaled in FPGA to 800x480) +
 * Gowin SIMD-4 INT8 DSP NPU Core (0xF8000900) + UDP port 5555.
 */

#ifndef APP_LCD_HUD_H_
#define APP_LCD_HUD_H_

#include <stdint.h>
#include <stdbool.h>

#define LCD_HUD_BASE        0xF8000800UL
#define LCD_HUD_MAGIC_V1    0x4C434431UL /* "LCD1" */
#define LCD_HUD_MAGIC_VAL   0x4C434432UL /* "LCD2" */

#define LCD_UDP_PORT        5555U
#define LCD_FB_W            200U
#define LCD_FB_H            120U
#define LCD_FB_SIZE         (LCD_FB_W * LCD_FB_H)

/* NPU Classification Classes */
#define NPU_CLASS_CAT       0U
#define NPU_CLASS_DOG       1U
#define NPU_CLASS_UNKNOWN   2U

/* Color palette indices (kept for console/vmsp compatibility) */
#define LCD_COL_SLATE       0U
#define LCD_COL_RED         1U
#define LCD_COL_GREEN       2U
#define LCD_COL_GOLD        3U
#define LCD_COL_BLUE        4U
#define LCD_COL_MAGENTA     5U
#define LCD_COL_CYAN        6U
#define LCD_COL_WHITEBLUE   7U
#define LCD_COL_GRAY        8U
#define LCD_COL_WHITE       9U

typedef enum {
    ANIM_MODE_PLASMA          = 0,
    ANIM_MODE_EQUALIZER       = 1,
    ANIM_MODE_GRAPH           = 2,
    ANIM_MODE_IMAGE           = 3,
    ANIM_MODE_CALIBRATION     = 4,
    ANIM_MODE_SOLID           = 5,
    ANIM_MODE_CAMERA          = 6,
    ANIM_MODE_COUNT
} lcd_anim_mode_t;

void lcd_hud_init(void);
bool lcd_hud_is_active(void);
void lcd_hud_tick(uint32_t rx_pkts, uint32_t tx_pkts);
void lcd_hud_set_pc_state(const char *state, uint8_t color);
void lcd_hud_log_exchange(uint8_t color, const char *proto, const char *msg);
void lcd_hud_pulse_rx(void);
void lcd_hud_submit_camera_frame(const uint8_t *fb_200x120, uint32_t frame_id, bool auto_switch);
bool lcd_hud_has_hw_gvsp(void);
void lcd_hud_set_hw_gvsp_mode(bool hw_en, bool mono8);
void lcd_hud_set_link_status(bool link_ok, uint32_t cam_ip);
uint32_t lcd_hud_get_hw_gvsp_stat(void);
uint32_t lcd_hud_get_hw_gvsp_drops(void);

/* Animation & Display control API */
void lcd_hud_anim_next_mode(void);
void lcd_hud_anim_set_mode(lcd_anim_mode_t mode);
lcd_anim_mode_t lcd_hud_anim_get_mode(void);
const char *lcd_hud_anim_get_mode_name(lcd_anim_mode_t mode);
void lcd_hud_cycle_rotation(void);
void lcd_hud_toggle_resolution(void);

#endif /* APP_LCD_HUD_H_ */
