// camera.h - VEYE CS132M cameras over I2C (address 0x3B) behind the TWI
// multiplexer (i2c_bus.h).
// I2Ccam.h until 24.09.2026.

#ifndef APP_CAMERA_H_
#define APP_CAMERA_H_

#include <stdint.h>
#include <stdbool.h>


// Manufacturer id returned by register 0x0000 on a VEYE module: ASCII "VEYE".
#define VEYE_MANUFACTURER_ID  0x56455945u

// Scan all 8 multiplexer positions for the camera. Returns the position, or -1.
int camera_scan_mux(uint8_t cam_index);
// Camera multiplexer position, -1 = not found yet.
int  camera_mux_position(uint8_t cam_index);

// CS132M camera I2C address (7-bit)
#define CAM_I2C_ADDR           0x3B



// Compile-time defaults. They match the host tools' defaults
// (10000 us / 6 dB / 10 fps, CLAUDE.md) and the FPGA geometry.
#define CAM_BOOT_EXP_US   10000U
#define CAM_BOOT_GAIN     60U        /* 0.1 dB */
#define CAM_BOOT_FPS      1000U      /* 0.01 Hz -> 10 fps */
int camera_read_reg_status(uint16_t reg_addr, uint32_t *val);

void camera_select_channel(uint8_t channel);
void camera_load_default_config(uint8_t cam_index);

// Software-trigger capture (VEYE MV register map).
// Continuous streaming keeps rewriting the DDR3 ring, so a JTAG dump of a frame
// is always torn. In trigger mode the sensor emits exactly `frames` images per
// trigger and then goes quiet, which leaves a stable frame in the ring.
//   camera_set_trigger_mode(cam, 1) -> 0x0404=1 (normal trigger), 0x0408=0 (software),
//                                      0x040C=frames, then 0x0400=1 to arm
//   camera_trigger_once(cam)        -> 0x0414=1, fires one trigger
//   camera_set_trigger_mode(cam, 0) -> back to free-running video
int camera_set_trigger_mode(uint8_t cam_index, uint8_t enable_trigger, uint8_t frames_per_trigger);
void camera_dump_link_info(uint8_t cam_index);
int camera_set_acquisition(uint8_t cam_index, uint8_t on);
// Trigger source: 0 = software (write 0x0414 over I2C), 1 = hardware (the
// Trigger In input on camera connector J2, driven by the FPGA from pin A18).
// camera_set_trigger_mode() writes it into register 0x0408 when arming.
extern uint8_t g_trigger_source;

int camera_trigger_once(uint8_t cam_index);
int camera_read_trigger_count(uint8_t cam_index, uint32_t *total, uint32_t *lost);
// Shortest trigger period the camera takes, from the cache (no I2C): exposure + 0.42 ms +
// 6.2 us per ROI row. 0 = not connected or not in trigger mode.
uint32_t camera_trigger_cycle_ns(uint8_t cam_index);
uint32_t camera_cycle_ns(uint32_t rows, uint32_t exp_us);   // the same for given settings
uint32_t camera_cached_roi_h(uint8_t cam_index);
// Last requested camera exposure in us (from the cache, no I2C access).
// 0 = bad camera number. The capture path needs it to compute the laser pulse
// width when the strobe measured by the FPGA is unusable (ERRORS 4.2, former
// 31.52).
uint32_t camera_cached_exposure_us(uint8_t cam_index);
int camera_configure(uint8_t cam_index, uint8_t trg_mode, uint8_t trg_src, uint8_t auto_exp, uint32_t exp, uint32_t gain, uint32_t fps, uint8_t testpat, uint32_t roi_w, uint32_t roi_h, uint32_t offset_x, uint32_t offset_y);
int camera_read_config(uint8_t cam_index, bool *connected, uint32_t *model_id,
                       uint32_t *sensor_w, uint32_t *sensor_h,
                       uint8_t *trg_mode, uint8_t *trg_src,
                       uint8_t *auto_exp, uint32_t *exp, uint32_t *gain,
                       uint32_t *fps, uint32_t *max_fps,
                       uint32_t *err_code, uint8_t *testpat,
                       uint32_t *roi_w, uint32_t *roi_h,
                       uint32_t *offset_x, uint32_t *offset_y);

#include "FreeRTOS.h"
#include "semphr.h"

// Structures for Camera Parameters Data
typedef struct {
	bool     connected;   // Connection status (true = responded over I2C)
	uint32_t model_id;    // Camera Model ID (reg 0x0000)
	uint32_t sensor_w;    // Sensor Width, reg 0x0808: 1080 (the array is portrait)
	uint32_t sensor_h;    // Sensor Height: 1280. "1280 x 1024 native" is refuted, ERRORS 2.2
	uint8_t  trg_mode;    // Trigger/Working Mode (0: Stream, 1: Trigger, 2: Multi-frame, 3: Pulse, reg 0x0404)
	uint8_t  trg_src;     // Trigger Source (0: Software, 1: Hardware pin, reg 0x0408)
	uint8_t  auto_exp;    // Auto Exposure mode (0: Manual, 1: Auto AEC, reg 0x0C00)
	uint32_t exp;         // Exposure time in us (reg 0x0C10)
	uint32_t gain;        // Gain value in 0.1 dB (reg 0x0C20)
	uint32_t fps;         // Current Frame rate (0.01 Hz, reg 0x0814)
	uint32_t max_fps;     // Max Frame rate for current ROI (0.01 Hz)
	uint32_t err_code;    // Error Code / Status (reg 0x0024)
	uint8_t  testpat;     // Test pattern state (reg 0x0800)
	uint32_t roi_w;       // ROI Width (reg 0x0818)
	uint32_t roi_h;       // ROI Height (reg 0x081C)
	uint32_t offset_x;    // ROI Offset X (reg 0x0820)
	uint32_t offset_y;    // ROI Offset Y (reg 0x0824)
} camera_params_t;


// I2C control and read functions
void camera_write_reg(uint16_t reg_addr, uint32_t reg_data);
// Bring the camera into a known state at firmware boot: capture stopped,
// settings from the boot set the caller gives (main.c takes it from
// settings_boot_cam(): SERVICE from the store, or without it the CAM_BOOT_*
// defaults). The camera is a separate microcontroller with its own power and
// survives an FPGA reboot in whatever state it was left (ERRORS 2.6). Call under
// i2c_bus_lock().
int  camera_boot_reset(uint8_t cam_index, const camera_params_t *boot);

// What the firmware REQUESTED from the camera (camera_configure cache and
// 0x0120), without I2C access. Fills trg_mode, trg_src, auto_exp, exp, gain,
// fps, testpat, roi_w, roi_h, offset_x, offset_y. These are the "current
// settings" for saving to the store. -1 = bad camera number.
int  camera_cached_params(uint8_t cam_index, camera_params_t *p);
// 0x0120 writes exposure and gain bypassing camera_configure(); the cache must
// know, otherwise saving to the store would carry the old values. gain < 0 =
// do not change.
void camera_cache_set_exposure(uint8_t cam_index, uint32_t exp_us, int32_t gain);
// ROI height written to the sensor bypassing camera_configure(); offset_y < 0 =
// do not change.
void camera_cache_set_roi_h(uint8_t cam_index, uint32_t roi_h, int32_t offset_y);

// Last read camera parameters (written by the polling tasks, main.c). Until
// 24.09.2026 these were the globals g_camera_memory and xCameraMutex. get
// returns false if the state is busy for more than 100 ms.
void camera_state_store(uint8_t cam_index, const camera_params_t *p);
bool camera_state_get(uint8_t cam_index, camera_params_t *out);
bool camera_connected(uint8_t cam_index);

// ROI listener (mm_table.c, docs/DECISIONS.md R-38): told before the ROI registers
// are written (settled = false, *roi = the ROI requested) and after (settled =
// true, *roi = the ROI the sensor reads back, NULL if it is not the one requested).
// Called with the I2C bus held: the listener must neither take the bus nor wait.
// Only roi_w, roi_h, offset_x, offset_y of *roi are filled.
typedef void (*camera_roi_listener_t)(uint8_t cam, bool settled, const camera_params_t *roi);
void camera_set_roi_listener(camera_roi_listener_t fn);

// Bus busy for longer than CAMERA_BUS_WAIT_MS (camera.c): camera functions take
// the I2C bus themselves and return this code on failure.
#define CAMERA_ERR_BUS_BUSY  (-110)

void camera_init(void);

#endif // APP_CAMERA_H_
