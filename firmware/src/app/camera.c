// camera.c - VEYE CS132M cameras over I2C behind the TWI multiplexer (bus:
// i2c_bus.c). I2Ccam.c until 24.09.2026.

#include "camera.h"
#include "i2c_bus.h"
#include "board.h"

#define CAMERA_BUS_WAIT_MS  1000U

static int camera_write_reg_status_bus(uint16_t reg_addr, uint32_t reg_data);
static void camera_write_reg_bus(uint16_t reg_addr, uint32_t reg_data);
static int camera_read_reg_status_bus(uint16_t reg_addr, uint32_t *val);
static uint32_t camera_read_reg_bus(uint16_t reg_addr);
static int camera_scan_mux_bus(uint8_t cam_index);
static void camera_select_channel_bus(uint8_t channel);
static int camera_boot_reset_bus(uint8_t cam_index, const camera_params_t *boot);
static int camera_configure_bus(uint8_t cam_index, uint8_t trg_mode, uint8_t trg_src, uint8_t auto_exp, uint32_t exp, uint32_t gain, uint32_t fps, uint8_t testpat, uint32_t roi_w, uint32_t roi_h, uint32_t offset_x, uint32_t offset_y);
static int camera_read_config_bus(uint8_t cam_index, bool *connected, uint32_t *model_id,
                       uint32_t *sensor_w, uint32_t *sensor_h,
                       uint8_t *trg_mode, uint8_t *trg_src,
                       uint8_t *auto_exp, uint32_t *exp, uint32_t *gain,
                       uint32_t *fps, uint32_t *max_fps,
                       uint32_t *err_code, uint8_t *testpat,
                       uint32_t *roi_w, uint32_t *roi_h,
                       uint32_t *offset_x, uint32_t *offset_y);
static void camera_load_default_config_bus(uint8_t cam_index);
static int camera_set_trigger_mode_bus(uint8_t cam_index, uint8_t enable_trigger, uint8_t frames_per_trigger);
static void camera_dump_link_info_bus(uint8_t cam_index);
static int camera_set_acquisition_bus(uint8_t cam_index, uint8_t on);
static int camera_trigger_once_bus(uint8_t cam_index);
static int camera_read_trigger_count_bus(uint8_t cam_index, uint32_t *total, uint32_t *lost);
#include "memory_map.h"
#include "platform.h"
#include "uart.h"
#include "app_log_config.h"
#include <stdio.h>
#include <string.h>

#include "Driver_GPIO.h"
extern AE350_DRIVER_GPIO Driver_GPIO;
static AE350_DRIVER_GPIO* pDrv_GPIO = &Driver_GPIO;

#include "Driver_I2C.h"
extern AE350_DRIVER_I2C Driver_I2C;
static AE350_DRIVER_I2C* pDrv_I2C = &Driver_I2C;



// Pause after every transaction with the camera.
//
// The VEYE camera is a microcontroller, and it loses transactions that come
// back to back (the reference project archive/.../camera_i2c_init.sv keeps
// ~1 ms between all writes). The pause used to be only after reads and in one
// place of the trigger setup, while writes went back to back. On 23.09.2026 it
// showed like this: at boot, fifteen reads were followed straight away by the
// stop write 0x0400 = 0, the camera lost it and stayed in free-run at 120 fps;
// the FPGA writer poured rows into DDR3, and the network choked until a manual
// UART command F (ERRORS 2.6). The pause is now after ANY transaction, with
// twice the reference's margin.
#define CAM_I2C_GAP_US   2000U

// Write 32-bit register over I2C. Returns 0 on success, <0 on a bus error.
static int camera_write_reg_status_bus(uint16_t reg_addr, uint32_t reg_data)
{
    LOG_I2C("Write Reg 0x%04X = 0x%08X", reg_addr, (unsigned int)reg_data);
    if (pDrv_I2C->GetStatus().busy)
    {
    	LOG_I2C_ERR("Bus busy before transmit (Reg 0x%04X)", reg_addr);
        i2c_bus_reset();
    }
    static uint8_t buf[6];
    buf[0] = (uint8_t)(reg_addr >> 8);
    buf[1] = (uint8_t)(reg_addr & 0xFF);
    buf[2] = (uint8_t)(reg_data >> 24);
    buf[3] = (uint8_t)((reg_data >> 16) & 0xFF);
    buf[4] = (uint8_t)((reg_data >> 8) & 0xFF);
    buf[5] = (uint8_t)(reg_data & 0xFF);

    int32_t res = pDrv_I2C->MasterTransmit(CAM_I2C_ADDR, buf, 6, false);
    if (res != AE350_DRIVER_OK) {
        LOG_I2C_ERR("MasterTransmit failed with err %d (Reg 0x%04X)", (int)res, reg_addr);
        i2c_bus_reset();
        board_delay_us(CAM_I2C_GAP_US);
        return -1;
    }

    // 50ms timeout using hardware MTIME (100 MHz = 5,000,000 ticks)
    uint32_t start_time = DEV_PLMT->MTIME[0];
    while (pDrv_I2C->GetStatus().busy && ((DEV_PLMT->MTIME[0] - start_time) < 5000000));

    int rc = 0;
    if (pDrv_I2C->GetStatus().busy) {
        LOG_I2C_ERR("Bus stayed busy after 50ms timeout! (Reg 0x%04X)", reg_addr);
        i2c_bus_reset();
        rc = -2;
    } else if (pDrv_I2C->GetStatus().bus_error) {
        // As for a read: NACK or lost arbitration means the write did not get through.
        LOG_I2C_ERR("Bus error on write (Reg 0x%04X)", reg_addr);
        i2c_bus_reset();
        rc = -3;
    }
    board_delay_us(CAM_I2C_GAP_US);
    return rc;
}

static void camera_write_reg_bus(uint16_t reg_addr, uint32_t reg_data)
{
    (void)camera_write_reg_status_bus(reg_addr, reg_data);
}

// Read 32-bit register over I2C with status check (returns 0 on success, -1 on failure/NACK)
static int camera_read_reg_status_bus(uint16_t reg_addr, uint32_t *val)
{
    if (val) *val = 0;

    if (pDrv_I2C->GetStatus().busy)
    {
        i2c_bus_reset();
    }

    uint8_t addr_buf[2];
    uint8_t data_buf[4] = {0, 0, 0, 0};
    addr_buf[0] = (uint8_t)(reg_addr >> 8);
    addr_buf[1] = (uint8_t)(reg_addr & 0xFF);
    
    int32_t res_tx = pDrv_I2C->MasterTransmit(CAM_I2C_ADDR, addr_buf, 2, true);
    if (res_tx != AE350_DRIVER_OK)
    {
        LOG_I2C_ERR("Read Reg 0x%04X: Transmit addr failed (%d)", reg_addr, (int)res_tx);
        i2c_bus_reset();
        return -1;
    }
    
    uint32_t start_time = DEV_PLMT->MTIME[0];
    while (pDrv_I2C->GetStatus().busy && ((DEV_PLMT->MTIME[0] - start_time) < 500000));
    if (pDrv_I2C->GetStatus().busy || pDrv_I2C->GetStatus().bus_error)
    {
        LOG_I2C_ERR("Read Reg 0x%04X: Transmit addr busy/error", reg_addr);
        i2c_bus_reset();
        return -1;
    }
    
    int32_t res_rx = pDrv_I2C->MasterReceive(CAM_I2C_ADDR, data_buf, 4, false);
    if (res_rx != AE350_DRIVER_OK)
    {
        LOG_I2C_ERR("Read Reg 0x%04X: Receive data failed (%d)", reg_addr, (int)res_rx);
        i2c_bus_reset();
        return -1;
    }
    
    start_time = DEV_PLMT->MTIME[0];
    while (pDrv_I2C->GetStatus().busy && ((DEV_PLMT->MTIME[0] - start_time) < 500000));
    if (pDrv_I2C->GetStatus().busy || pDrv_I2C->GetStatus().bus_error)
    {
        LOG_I2C_ERR("Read Reg 0x%04X: Receive data busy/error", reg_addr);
        i2c_bus_reset();
        return -1;
    }

    uint32_t read_val = ((uint32_t)data_buf[0] << 24) | ((uint32_t)data_buf[1] << 16) | ((uint32_t)data_buf[2] << 8) | (uint32_t)data_buf[3];
    if (val) *val = read_val;
    LOG_I2C("Read Reg 0x%04X => 0x%08X", reg_addr, (unsigned int)read_val);
    board_delay_us(CAM_I2C_GAP_US);
    return 0;
}

// Read 32-bit register over I2C for verification check
static uint32_t camera_read_reg_bus(uint16_t reg_addr)
{
    uint32_t val = 0;
    if (camera_read_reg_status_bus(reg_addr, &val) == 0) {
        return val;
    }
    return 0;
}

// Currently selected TWI multiplexer address for Cam 0 and Cam 1 (-1 means not located yet).
static int s_cam_mux[2] = { -1, -1 };

// Last read camera parameters and their lock (camera_state_*, below).
static camera_params_t   s_state[2];
static SemaphoreHandle_t s_state_lock;



// Walk multiplexer positions and accept the first one where register
// 0x0000 returns the VEYE manufacturer id.
// To support two cameras simultaneously, a position already claimed by the
// other camera is skipped.
static int camera_scan_mux_bus(uint8_t cam_index)
{
    static const uint8_t scan_order_cam0[8] = { 3, 7, 2, 6, 1, 5, 0, 4 };
    static const uint8_t scan_order_cam1[8] = { 7, 3, 6, 2, 5, 1, 4, 0 };
    const uint8_t *scan_order = (cam_index == 0) ? scan_order_cam0 : scan_order_cam1;
    uint32_t id;
    uint8_t  i, mux;

    if (cam_index > 1) return -1;

    for (i = 0; i < 8; i++)
    {
        mux = scan_order[i];

        // Do not scan/claim a mux position already assigned to the other camera
        if (s_cam_mux[1 - cam_index] >= 0 && (int)mux == s_cam_mux[1 - cam_index])
        {
            continue;
        }

        i2c_mux_select(mux);
        id = 0;
        if (camera_read_reg_status_bus(0x0000, &id) == 0 && id == VEYE_MANUFACTURER_ID)
        {
            s_cam_mux[cam_index] = (int)mux;
            printf("[CS132M] Camera %d found at TWI mux %u (0x0000 = 0x%08X 'VEYE')\r\n",
                   (int)cam_index, (unsigned int)mux, (unsigned int)id);
            return (int)mux;
        }
    }

    printf("[CS132M] Camera %d NOT found on any available TWI mux positions\r\n", (int)cam_index);
    s_cam_mux[cam_index] = -1;
    return -1;
}

// Select TWI multiplexer channel for cam 0 or cam 1
static void camera_select_channel_bus(uint8_t channel)
{
    if (channel > 1) return;

    if (s_cam_mux[channel] >= 0)
    {
        i2c_mux_select((uint8_t)s_cam_mux[channel]);
        return;
    }

    // Default fallback if not scanned yet (connector 0 -> mux 3, connector 1 -> mux 7)
    uint8_t default_mux = (channel == 0) ? 3 : 7;
    i2c_mux_select(default_mux);
}

typedef struct {
    uint8_t  trg_mode;
    uint8_t  trg_src;
    uint8_t  auto_exp;
    uint32_t exp;
    uint32_t gain;
    uint32_t fps;
    uint8_t  testpat;
    uint32_t roi_w;
    uint32_t roi_h;
    uint32_t offset_x;
    uint32_t offset_y;
} camera_cfg_cache_t;

static camera_cfg_cache_t g_cam_cache[2] = {
    {0, 0, 0, 1200, 150, 1000, 0, VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT, 0, 0},
    {0, 0, 0, 1200, 150, 1000, 0, VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT, 0, 0}
};

// ROI listener (mm_table.c, R-38). Called through a pointer so that camera.c does
// not depend on a module above it (check_layers.py).
static camera_roi_listener_t s_roi_listener = NULL;

void camera_set_roi_listener(camera_roi_listener_t fn)
{
    s_roi_listener = fn;
}

static void camera_roi_notify(uint8_t cam_index, bool settled, const camera_params_t *roi)
{
    camera_roi_listener_t fn = s_roi_listener;
    if (fn != NULL) fn(cam_index, settled, roi);
}

// The last exposure REQUESTED by the firmware from this camera, in microseconds.
//
// Taken from the cache, not read over I2C: it is called from the capture path,
// where going onto the bus under someone else's mutex is not allowed. The cache
// is filled by camera_configure_bus() with what actually went into register
// 0x0C10, and it has a sensible default, so zero is returned only for a bad
// camera number.
uint32_t camera_cached_exposure_us(uint8_t cam_index)
{
    if (cam_index >= 2) return 0U;
    return g_cam_cache[cam_index].exp;
}

// The sensor's frame cycle in 0.1 us: 0.42 ms + 6.2 us per row (ERRORS former 31.54), plus
// the exposure when triggered (HARDWARE "Trigger"); free run overlaps exposure and readout.
static uint32_t sensor_cycle_01us(uint32_t rows, uint32_t exp_us, bool triggered)
{
    uint32_t cyc = 4200U + 62U * rows;
    if (triggered) return cyc + exp_us * 10U;
    return (exp_us * 10U > cyc) ? exp_us * 10U : cyc;
}

uint32_t camera_cycle_ns(uint32_t rows, uint32_t exp_us)
{
    return sensor_cycle_01us(rows ? rows : 1280U, exp_us, true) * 100U;
}

uint32_t camera_trigger_cycle_ns(uint8_t cam_index)
{
    const camera_cfg_cache_t *c;

    if (cam_index >= 2 || !camera_connected(cam_index)) return 0U;
    c = &g_cam_cache[cam_index];
    if (c->trg_mode == 0) return 0U;                  // free run: the generator does not drive it
    return camera_cycle_ns(c->roi_h, c->exp);
}

uint32_t camera_cached_roi_h(uint8_t cam_index)
{
    return (cam_index < 2) ? g_cam_cache[cam_index].roi_h : 0U;
}

int camera_cached_params(uint8_t cam_index, camera_params_t *p)
{
    const camera_cfg_cache_t *c;

    if (cam_index >= 2 || p == 0) return -1;
    c = &g_cam_cache[cam_index];
    p->trg_mode = c->trg_mode;
    p->trg_src  = c->trg_src;
    p->auto_exp = c->auto_exp;
    p->exp      = c->exp;
    p->gain     = c->gain;
    p->fps      = c->fps;
    p->testpat  = c->testpat;
    p->roi_w    = c->roi_w;
    p->roi_h    = c->roi_h;
    p->offset_x = c->offset_x;
    p->offset_y = c->offset_y;
    return 0;
}

void camera_cache_set_exposure(uint8_t cam_index, uint32_t exp_us, int32_t gain)
{
    if (cam_index >= 2) return;
    g_cam_cache[cam_index].exp = exp_us;
    if (gain >= 0) g_cam_cache[cam_index].gain = (uint32_t)gain;
}

// ROI height written to the sensor bypassing camera_configure() (video.c,
// video_ensure_roi_locked). Since 02.10.2026 capture uses the height cache to
// check that the frame in the slot has the height the sensor outputs
// (ERRORS 2.11, 2.13); the cache must not lag the register. offset_y < 0 = the
// offset did not change.
void camera_cache_set_roi_h(uint8_t cam_index, uint32_t roi_h, int32_t offset_y)
{
    camera_params_t roi;

    if (cam_index >= 2) return;
    g_cam_cache[cam_index].roi_h = roi_h;
    if (offset_y >= 0) g_cam_cache[cam_index].offset_y = (uint32_t)offset_y;
    // video.c rewrote the ROI with acquisition stopped: the band follows the cache.
    memset(&roi, 0, sizeof(roi));
    roi.roi_w    = g_cam_cache[cam_index].roi_w;
    roi.roi_h    = g_cam_cache[cam_index].roi_h;
    roi.offset_x = g_cam_cache[cam_index].offset_x;
    roi.offset_y = g_cam_cache[cam_index].offset_y;
    camera_roi_notify(cam_index, true, &roi);
}

// The normal camera state after boot is the boot set (the caller gives it: the
// SERVICE set from the store, or if there is none the CAM_BOOT_* defaults, see
// settings_boot_cam()). Mode 0 (stream) leaves capture off, as before.
static int camera_boot_reset_bus(uint8_t cam_index, const camera_params_t *boot)
{
    int rc_a, rc_b, rc;

    if (cam_index > 1 || boot == NULL) return -1;
    camera_select_channel_bus(cam_index);

    // Stop twice, with a pause longer than a frame between them. A single stop was
    // lost on 23.09.2026 (ERRORS 2.6), and an extra stop is harmless: the camera is
    // stopped anyway.
    rc_a = camera_write_reg_status_bus(0x0400, 0x00000000);
    board_delay_us(150000);
    rc_b = camera_write_reg_status_bus(0x0400, 0x00000000);
    board_delay_us(150000);

    // Then the usual setup: it stops capture once more, writes ROI, frame rate,
    // exposure and mode, reads them back and prints the comparison. Mode 0 (stream)
    // leaves capture OFF; the camera is armed for the trigger on the first capture
    // request, as before.
    rc = camera_configure_bus(cam_index, 0, g_trigger_source, 0,
                          boot->exp, boot->gain, boot->fps, boot->testpat,
                          boot->roi_w, boot->roi_h, boot->offset_x, boot->offset_y);

    printf("[CAMERA] Cam %u: reset to known state at boot - stop x2 (rc %d, %d), configure rc %d\r\n",
           (unsigned int)cam_index, rc_a, rc_b, rc);
    return rc;
}

// Configure camera parameters over I2C
static int camera_configure_bus(uint8_t cam_index, uint8_t trg_mode, uint8_t trg_src, uint8_t auto_exp, uint32_t exp, uint32_t gain, uint32_t fps, uint8_t testpat, uint32_t roi_w, uint32_t roi_h, uint32_t offset_x, uint32_t offset_y)
{
    if (cam_index > 1) return -1;

    // CS132M range limits. Native array read back from the camera itself:
    // 0x0808 Sensor_Width = 1080, 0x080C Sensor_Height = 1280.
    // A too-large ROI is NOT rejected - the camera clamps 0x0818 silently.
    if (auto_exp > 1) return -2;
    if (!auto_exp && (exp < 1 || exp > 10000000)) return -2;
    if (gain > 240) return -3;
    if (fps < 100 || fps > 400000) return -4;
    if (testpat > 1) return -5;
    if (roi_w < 128 || roi_w > 1080) return -6;  // 1080 = sensor max width
    if (roi_h < 64 || roi_h > 1280) return -7;  // 1280 = 0x080C Sensor_Height (was 1024, rejected the full frame)
    if (offset_x + roi_w > 1080) return -8;
    if (offset_y + roi_h > 1280) return -9;
    if (trg_mode > 3) return -10;
    if (trg_src > 1) return -11;

    // Cache requested config in shadow structure
    g_cam_cache[cam_index].trg_mode = trg_mode;
    g_cam_cache[cam_index].trg_src = trg_src;
    g_cam_cache[cam_index].auto_exp = auto_exp;
    g_cam_cache[cam_index].exp = exp;
    g_cam_cache[cam_index].gain = gain;
    g_cam_cache[cam_index].fps = fps;
    g_cam_cache[cam_index].testpat = testpat;
    g_cam_cache[cam_index].roi_w = roi_w;
    g_cam_cache[cam_index].roi_h = roi_h;
    g_cam_cache[cam_index].offset_x = offset_x;
    g_cam_cache[cam_index].offset_y = offset_y;

    camera_select_channel_bus(cam_index);
    {
        // The ROI about to be written: a listener keeps what does not change.
        camera_params_t req;
        memset(&req, 0, sizeof(req));
        req.roi_w = roi_w; req.roi_h = roi_h; req.offset_x = offset_x; req.offset_y = offset_y;
        camera_roi_notify(cam_index, false, &req);
    }

    // Stop sensor stream before applying ROI, Mode and timing adjustments.
    //
    // The pause after the stop is mandatory: while the sensor runs, the camera
    // accepts some registers over I2C but silently does not apply them
    // (docs/HARDWARE.md § 8, "reads in running, but a write silently does not go
    // through"). A frame being transmitted at the stop is finished, so the wait
    // must be longer than the longest frame: 100 ms at 10 fps, we use 150.
    // camera_load_default_config_bus() kept 50 ms, the trigger setup 150; here
    // there was no pause at all.
    camera_write_reg_bus(0x0400, 0x00000000);
    board_delay_us(150000);

    // 1. Write mode, parameters and ROI registers to camera
    camera_write_reg_bus(0x0404, trg_mode);         // 0: Stream, 1: Normal Trigger, 2: Multi-frame, 3: Pulse
    camera_write_reg_bus(0x0408, trg_src);          // 0: Software trigger, 1: Hardware trigger pin
    camera_write_reg_bus(0x0818, roi_w);
    camera_write_reg_bus(0x0820, offset_x);
    camera_write_reg_bus(0x0824, offset_y);
    camera_write_reg_bus(0x081C, roi_h);
    camera_write_reg_bus(0x0814, fps);
    camera_write_reg_bus(0x0C04, auto_exp ? 2 : 0); // Exposure Mode: 0: Manual, 2: Auto Exposure
    if (!auto_exp) {
        camera_write_reg_bus(0x0C10, exp);           // Manual exposure time in us
    }
    camera_write_reg_bus(0x0C20, gain);
    camera_write_reg_bus(0x0800, testpat ? 1 : 0);

    // Restart sensor acquisition only if in trigger mode (1); in stream mode (0)
    // keep stopped to avoid DDR3 bus collisions until explicitly triggered.
    if (trg_mode == 1) {
        camera_write_reg_bus(0x0400, 0x00000001);
    } else {
        camera_write_reg_bus(0x0400, 0x00000000);
    }

    // 2. Read registers over I2C for verification check
    uint32_t read_mode = camera_read_reg_bus(0x0404);
    uint32_t read_src = camera_read_reg_bus(0x0408);
    uint32_t read_expmode = camera_read_reg_bus(0x0C04);
    uint32_t read_exp = camera_read_reg_bus(0x0C10);
    uint32_t read_gain = camera_read_reg_bus(0x0C20);
    uint32_t read_fps = camera_read_reg_bus(0x0814);
    uint32_t read_testpat = camera_read_reg_bus(0x0800);
    uint32_t read_w = camera_read_reg_bus(0x0818);
    uint32_t read_h = camera_read_reg_bus(0x081C);
    uint32_t read_off_x = camera_read_reg_bus(0x0820);
    uint32_t read_off_y = camera_read_reg_bus(0x0824);

    uint8_t aec_verified = (read_expmode >= 1) ? 1 : 0;
    // On RAW series sensors (e.g. SC132M 0x8132 without onboard ISP), software cache maintains state
    if (read_expmode == 0)
    {
        aec_verified = auto_exp;
    }

    printf("[CS132M I2C Verification] Cam %d:\r\n", (int)cam_index);
    printf(" - MODE: Written=%u, Read=%u %s\r\n", (unsigned int)trg_mode, (unsigned int)read_mode, (trg_mode == read_mode) ? "[OK]" : "[FAIL]");
    printf(" - TRG_SRC: Written=%u, Read=%u %s\r\n", (unsigned int)trg_src, (unsigned int)read_src, (trg_src == read_src) ? "[OK]" : "[FAIL]");
    printf(" - AEC: Written=%u, ExpMode=%u %s\r\n", (unsigned int)auto_exp, (unsigned int)read_expmode, (auto_exp == aec_verified) ? "[OK]" : "[FAIL]");
    printf(" - EXP: Written=%u, Read=%u %s\r\n", (unsigned int)exp, (unsigned int)read_exp, (exp == read_exp) ? "[OK]" : "[FAIL]");
    printf(" - GAIN: Written=%u, Read=%u %s\r\n", (unsigned int)gain, (unsigned int)read_gain, (gain == read_gain) ? "[OK]" : "[FAIL]");
    printf(" - FPS: Written=%u, Read=%u %s\r\n", (unsigned int)fps, (unsigned int)read_fps, (fps == read_fps) ? "[OK]" : "[FAIL]");
    printf(" - TESTPAT: Written=%u, Read=%u %s\r\n", (unsigned int)testpat, (unsigned int)read_testpat, (testpat == (read_testpat & 1)) ? "[OK]" : "[FAIL]");
    printf(" - ROI: %ux%u @ (%u,%u) (Read: %ux%u @ (%u,%u))\r\n", 
           (unsigned int)roi_w, (unsigned int)roi_h, (unsigned int)offset_x, (unsigned int)offset_y,
           (unsigned int)read_w, (unsigned int)read_h, (unsigned int)read_off_x, (unsigned int)read_off_y);

    // The ROI the sensor reports (R-38): the band must be the one of the rows the
    // sensor really outputs. The camera aligns offset Y down (770 was read back as
    // 768 on 16.10.2026), and that is the ROI the points are relative to - the
    // client takes the same read-back from 0x0122. Accepted only if the offsets read
    // the same twice with a good I2C status, the size is what was asked and the
    // offset moved down by less than 8 rows; anything else gets no band.
    {
        camera_params_t roi;
        uint32_t ry = 0, rx = 0;
        int sy = camera_read_reg_status_bus(0x0824, &ry);
        int sx = camera_read_reg_status_bus(0x0820, &rx);
        bool ok = (sy == 0) && (sx == 0) && (ry == read_off_y) && (rx == read_off_x) &&
                  (read_w == roi_w) && (read_h == roi_h) && (rx == offset_x) &&
                  (ry <= offset_y) && (offset_y - ry < 8U);
        memset(&roi, 0, sizeof(roi));
        roi.roi_w = read_w; roi.roi_h = read_h; roi.offset_x = rx; roi.offset_y = ry;
        camera_roi_notify(cam_index, true, ok ? &roi : NULL);
    }
    // A ROI size the camera does not take is ignored, not rejected (20.10.2026:
    // 1080x32 and 1080x48 read back as 1080x1280): it keeps sending its old frame,
    // and the FPGA cuts that frame's first rows as if they were the ROI - profiles
    // of the wrong rows with nothing to show it. Refuse instead.
    if (read_w != roi_w || read_h != roi_h) {
        printf("[CAMERA] Cam %u: ROI %ux%u not taken by the camera (reads %ux%u)\r\n",
               (unsigned int)cam_index, (unsigned int)roi_w, (unsigned int)roi_h,
               (unsigned int)read_w, (unsigned int)read_h);
        return -12;
    }
    return 0;
}

// Read camera parameters and full telemetry over I2C with connection detection
static int camera_read_config_bus(uint8_t cam_index, bool *connected, uint32_t *model_id,
                       uint32_t *sensor_w, uint32_t *sensor_h,
                       uint8_t *trg_mode, uint8_t *trg_src,
                       uint8_t *auto_exp, uint32_t *exp, uint32_t *gain,
                       uint32_t *fps, uint32_t *max_fps,
                       uint32_t *err_code, uint8_t *testpat,
                       uint32_t *roi_w, uint32_t *roi_h,
                       uint32_t *offset_x, uint32_t *offset_y)
{
    if (cam_index > 1) return -1;

    camera_select_channel_bus(cam_index);

    uint32_t id = 0;
    int res_id = camera_read_reg_status_bus(0x0000, &id);

    // If I2C NACK / Error OR Vendor ID is 0 -> No camera on this connector
    if (res_id != 0 || id == 0)
    {
        LOG_I2C("Cam %u: NOT DETECTED (No ACK on I2C Reg 0x0000)", cam_index);
        if (connected) *connected = false;
        if (model_id) *model_id = 0;
        if (sensor_w) *sensor_w = 0;
        if (sensor_h) *sensor_h = 0;
        if (trg_mode) *trg_mode = 0;
        if (trg_src) *trg_src = 0;
        if (auto_exp) *auto_exp = 0;
        if (exp) *exp = 0;
        if (gain) *gain = 0;
        if (fps) *fps = 0;
        if (max_fps) *max_fps = 0;
        if (err_code) *err_code = 0;
        if (testpat) *testpat = 0;
        if (roi_w) *roi_w = 0;
        if (roi_h) *roi_h = 0;
        if (offset_x) *offset_x = 0;
        if (offset_y) *offset_y = 0;
        return -1;
    }

    // Camera detected and responded with valid Vendor ID!
    LOG_I2C("Cam %u: >>> DETECTED <<< Model ID = 0x%04X", cam_index, (unsigned int)id);
    if (connected) *connected = true;
    if (model_id) *model_id = id;

    uint32_t r_err = 0;
    uint32_t r_mode = g_cam_cache[cam_index].trg_mode;
    uint32_t r_src = g_cam_cache[cam_index].trg_src;
    uint32_t r_expmode = g_cam_cache[cam_index].auto_exp ? 2 : 0;
    uint32_t r_exp = g_cam_cache[cam_index].exp;
    uint32_t r_gain = g_cam_cache[cam_index].gain;
    uint32_t r_fps = g_cam_cache[cam_index].fps;
    uint32_t r_max_fps = 0;
    uint32_t r_testpat = g_cam_cache[cam_index].testpat;
    uint32_t r_w = g_cam_cache[cam_index].roi_w;
    uint32_t r_h = g_cam_cache[cam_index].roi_h;
    uint32_t r_off_x = g_cam_cache[cam_index].offset_x;
    uint32_t r_off_y = g_cam_cache[cam_index].offset_y;
    // Fallback only; the real geometry is read from 0x0808/0x080C below. Reporting
    // a hard-coded size is what hid the fact that the camera silently clamps a
    // too-wide ROI request (see docs/troubleshooting/ERRORS_AND_SOLUTIONS.md 2.2).
    uint32_t s_w = 1080, s_h = 1280;

    uint32_t hw_val = 0;
    if (camera_read_reg_status_bus(0x0808, &hw_val) == 0 && hw_val > 0) s_w = hw_val;
    if (camera_read_reg_status_bus(0x080C, &hw_val) == 0 && hw_val > 0) s_h = hw_val;
    if (camera_read_reg_status_bus(0x0024, &hw_val) == 0) r_err = hw_val;
    if (camera_read_reg_status_bus(0x0404, &hw_val) == 0) r_mode = hw_val;
    if (camera_read_reg_status_bus(0x0408, &hw_val) == 0) r_src = hw_val;
    if (camera_read_reg_status_bus(0x0C04, &hw_val) == 0) r_expmode = hw_val;
    if (camera_read_reg_status_bus(0x0C10, &hw_val) == 0 && hw_val > 0) r_exp = hw_val;
    if (camera_read_reg_status_bus(0x0C20, &hw_val) == 0) r_gain = hw_val;
    if (camera_read_reg_status_bus(0x0814, &hw_val) == 0 && hw_val > 0) r_fps = hw_val;
    if (camera_read_reg_status_bus(0x0838, &hw_val) == 0 && hw_val > 0) r_max_fps = hw_val;
    if (camera_read_reg_status_bus(0x0800, &hw_val) == 0) r_testpat = hw_val;
    if (camera_read_reg_status_bus(0x0818, &hw_val) == 0 && hw_val > 0) r_w = hw_val;
    if (camera_read_reg_status_bus(0x081C, &hw_val) == 0 && hw_val > 0) r_h = hw_val;
    if (camera_read_reg_status_bus(0x0820, &hw_val) == 0) r_off_x = hw_val;
    if (camera_read_reg_status_bus(0x0824, &hw_val) == 0) r_off_y = hw_val;

    // The sensor's own limit for this ROI (0x0838 reads 0 on the SC132M). Its cycle is
    // 0.42 ms + 6.2 us per row (ERRORS former 31.54: 1280 rows 120 fps, 128 rows 818 fps),
    // plus the exposure in trigger mode (HARDWARE "Trigger"). Until 21.10.2026 a made-up
    // formula capped this at 300 fps while 64 rows give ~1200 (OPEN_ISSUES 2.8).
    if (r_max_fps == 0 || r_max_fps > 35000)
        r_max_fps = 1000000000U / sensor_cycle_01us((r_h > 0) ? r_h : s_h, r_exp, r_mode != 0);   // 0.01 Hz

    uint8_t final_auto_exp = 0;
    if (id >= 0x8000)
    {
        // RAW Series (e.g. SC132M 0x8132 without onboard ISP AE engine): State tracked by software cache
        final_auto_exp = g_cam_cache[cam_index].auto_exp;
    }
    else
    {
        // MV Series with onboard ISP engine: Read from hardware register 0x0C04 (2: AE)
        final_auto_exp = (r_expmode == 2) ? 1 : 0;
    }

    if (sensor_w) *sensor_w = s_w;
    if (sensor_h) *sensor_h = s_h;
    if (trg_mode) *trg_mode = (uint8_t)(r_mode & 0x03);
    if (trg_src)  *trg_src  = (uint8_t)(r_src & 0x01);
    if (auto_exp) *auto_exp = final_auto_exp;
    if (exp) *exp = r_exp;
    if (gain) *gain = r_gain;
    if (fps) *fps = r_fps;
    if (max_fps) *max_fps = r_max_fps;
    if (err_code) *err_code = r_err;
    if (testpat) *testpat = (uint8_t)(r_testpat & 1);
    if (roi_w) *roi_w = r_w;
    if (roi_h) *roi_h = r_h;
    if (offset_x) *offset_x = r_off_x;
    if (offset_y) *offset_y = r_off_y;

    return 0;
}

// Load default settings for CS132M camera (VIDEO_FRAME_WIDTH x VIDEO_FRAME_HEIGHT Mono8, 1080x1280)
static void camera_load_default_config_bus(uint8_t cam_index)
{
    printf("[CS132M] Config camera %d (1080x1280 Mono8 STREAM mode)...\r\n", (int)cam_index);

    uint32_t rb;

    // Register order follows the reference bring-up design
    // (sc132m_mipi_hdmi_pointsearch_.../src/camera_i2c_init.sv): stop acquisition
    // first, then format/lanes, then ROI with the HEIGHT written last as the VEYE
    // map requires, then timing, and only then start.
    //
    // ROI width is the CSI-2 line length the receiver must be configured for, so
    // it has to agree with FRAME_WIDTH / LINE_BYTES in the FPGA. 1080 is the
    // sensor maximum; a larger request is not rejected, the camera just clamps it.
    camera_select_channel_bus(cam_index);
    camera_write_reg_bus(0x0400, 0x00000000); // Stop
    board_delay_us(50000);                      // leave the running state before ROI/mode writes
    camera_write_reg_bus(0x0404, 0x00000000); // Stream mode
    camera_write_reg_bus(0x0800, 0x00000001); // 1: Test Pattern 1 (Horizontal Ramp)
    camera_write_reg_bus(0x0804, 0x00000000); // Mono8 RAW8
    camera_write_reg_bus(0x083C, 0x00000002); // 2 Lanes MIPI
    camera_write_reg_bus(0x0818, VIDEO_FRAME_WIDTH); // ROI Width, must be 8-aligned
    camera_write_reg_bus(0x0820, 0);          // Offset X: 0
    camera_write_reg_bus(0x0824, 0);          // Offset Y: 0
    camera_write_reg_bus(0x081C, VIDEO_FRAME_HEIGHT); // ROI Height, written last per VEYE map
    camera_write_reg_bus(0x0814, 1000);       // 10.00 fps
    camera_write_reg_bus(0x0C10, 20000);      // Exposure: 20000 us (20 ms)
    camera_write_reg_bus(0x0C20, 100);        // Gain: 10.0 dB

    // Read the geometry back before starting. The camera silently clamps an
    // out-of-range ROI, so the only way to know the real line length on the wire
    // is to read it; the reference design verifies these same registers.
    board_delay_us(10000);
    rb = 0; camera_read_reg_status_bus(0x0818, &rb);
    printf("[CS132M]   ROI_Width (0x0818)  = %u %s\r\n",
           (unsigned int)rb, (rb == VIDEO_FRAME_WIDTH) ? "OK" : "MISMATCH");
    rb = 0; camera_read_reg_status_bus(0x081C, &rb);
    printf("[CS132M]   ROI_Height(0x081C)  = %u %s\r\n",
           (unsigned int)rb, (rb == VIDEO_FRAME_HEIGHT) ? "OK" : "MISMATCH");
    rb = 0; camera_read_reg_status_bus(0x083C, &rb);
    printf("[CS132M]   Lane_Num (0x083C)   = %u %s\r\n",
           (unsigned int)rb, (rb == 2) ? "OK" : "MISMATCH");
    rb = 0; camera_read_reg_status_bus(0x0804, &rb);
    printf("[CS132M]   Pixel_Format(0x0804)= %u %s\r\n",
           (unsigned int)rb, (rb == 0) ? "OK (Mono8)" : "MISMATCH");
    rb = 0; camera_read_reg_status_bus(0x0800, &rb);
    printf("[CS132M]   Test_Image (0x0800) = %u %s\r\n",
           (unsigned int)rb, (rb == 1) ? "OK (ramp)" : "MISMATCH");

    camera_write_reg_bus(0x0400, 0x00000001); // Start live sensor streaming!

    printf("[CS132M] Camera %d STREAM mode (%ux%u@10Hz TestPat1) started successfully!\r\n",
           (int)cam_index, (unsigned int)VIDEO_FRAME_WIDTH, (unsigned int)VIDEO_FRAME_HEIGHT);
}

// ---------------------------------------------------------------------------
// Software-trigger capture
//
// Free-running video keeps the FPGA rewriting the DDR3 ring, so a ~60 s JTAG
// dump of a 1.25 MB frame is always torn. Arming the sensor in normal-trigger
// mode with a software source means it emits frames only on demand and then
// stops, leaving a stable frame in the ring to read at leisure.
// ---------------------------------------------------------------------------
// Trigger source applied on the next mode arming.
// 0 = software (write to 0x0414 over I2C), 1 = hardware (the Trigger In input).
uint8_t g_trigger_source = 0;

static int camera_set_trigger_mode_bus(uint8_t cam_index, uint8_t enable_trigger, uint8_t frames_per_trigger)
{
    uint32_t cap = 0, rb_mode = 0, rb_src = 0, rb_num = 0;

    if (cam_index > 1) return -1;
    if (frames_per_trigger == 0) frames_per_trigger = 1;

    camera_select_channel_bus(cam_index);

    // 0x0030 TriggerMode Cap is read-only: bit0 video streaming, bit1 normal trigger,
    // bit2 rolling-shutter multi-frame, bit3 pulse. Check it before arming rather
    // than assuming the datasheet applies to this particular module.
    if (camera_read_reg_status_bus(0x0030, &cap) == 0)
    {
        printf("[CS132M] TriggerMode Cap (0x0030) = 0x%08X  [stream:%c normal:%c multi:%c pulse:%c]\r\n",
               (unsigned int)cap,
               (cap & 0x1) ? 'y' : 'n', (cap & 0x2) ? 'y' : 'n',
               (cap & 0x4) ? 'y' : 'n', (cap & 0x8) ? 'y' : 'n');
        if (enable_trigger && !(cap & 0x2))
        {
            printf("[CS132M] Camera does NOT advertise normal-trigger mode - arming anyway, verify readback.\r\n");
        }
    }
    else
    {
        printf("[CS132M] Could not read TriggerMode Cap (0x0030).\r\n");
    }

    // 0x0400 is write-only and must be stopped before the mode registers change
    // (the VEYE map marks 0x0404/0x0408 update timing "S" = while stopped).
    // Wait longer than one frame period (100 ms at 10 fps) so the sensor has really
    // left streaming, and gap the register writes: the reference bring-up design in
    // archive/.../camera_i2c_init.sv leaves ~1 ms between every write because the
    // camera's MCU drops back-to-back transactions.
    camera_write_reg_bus(0x0400, 0x00000000);
    board_delay_us(150000);

    if (enable_trigger)
    {
        uint32_t v = 0;

        camera_write_reg_bus(0x0404, 0x00000001);              // Normal trigger mode
        board_delay_us(2000);
        camera_read_reg_status_bus(0x0404, &v);
        printf("[CS132M]   0x0404 <- 1, reads %u %s\r\n", (unsigned int)v, (v == 1) ? "OK" : "MISMATCH");

        // 0x0408 is the trigger source: 0 software (over I2C), 1 hardware (the Trigger
        // In input on connector J2). The value comes from g_trigger_source, so the
        // choice is made by a command, not by rebuilding the firmware.
        // It is written only here, with the sensor stopped: a write on the fly silently
        // does not go through; the read-back returns the old value, and the camera
        // waits for a software trigger while the FPGA toggles the pin into the void.
        // And without the following 0x0400 = 1 the camera does not respond to triggers
        // at all.
        camera_write_reg_bus(0x0408, (uint32_t)g_trigger_source);
        board_delay_us(2000);
        camera_read_reg_status_bus(0x0408, &v);
        printf("[CS132M]   0x0408 <- %u (%s), reads %u %s\r\n",
               (unsigned int)g_trigger_source,
               (g_trigger_source == 0) ? "software" : "hardware",
               (unsigned int)v, (v == g_trigger_source) ? "OK" : "MISMATCH");

        camera_write_reg_bus(0x040C, frames_per_trigger);      // Frames emitted per trigger
        board_delay_us(2000);

        camera_write_reg_bus(0x0418, 0x00000001);              // Clear trigger statistics
        board_delay_us(2000);
    }
    else
    {
        camera_write_reg_bus(0x0404, 0x00000000);              // Back to video streaming
        board_delay_us(2000);
    }

    // Verify the sensor actually accepted the mode. 0x0404 (trgmode), 0x0408 (trgsrc)
    // and 0x040C (trgnum) are RW, so a readback proves software triggering is live
    // and that no external trigger wiring is required.
    camera_read_reg_status_bus(0x0404, &rb_mode);
    camera_read_reg_status_bus(0x0408, &rb_src);
    camera_read_reg_status_bus(0x040C, &rb_num);
    printf("[CS132M] Readback: TrgMode(0x0404)=%u TrgSrc(0x0408)=%u TrgNum(0x040C)=%u\r\n",
           (unsigned int)rb_mode, (unsigned int)rb_src, (unsigned int)rb_num);

    // The source is compared with what was written (g_trigger_source), not with
    // zero: the former check "rb_src != 0" rejected the hardware source, and this
    // function could not arm the camera for FPGA pulses (ERRORS 2.4).
    if (enable_trigger && (rb_mode != 1 || rb_src != g_trigger_source))
    {
        printf("[CS132M] WARNING: sensor did not latch trigger mode (want mode=1 src=%u). Keeping stopped.\r\n",
               (unsigned int)g_trigger_source);
        camera_write_reg_bus(0x0400, 0x00000000);
        return -1;
    }

    // Arm: in trigger mode this makes the sensor wait for the trigger signal
    // rather than start emitting frames.
    if (enable_trigger) {
        camera_write_reg_bus(0x0400, 0x00000001);
    } else {
        camera_write_reg_bus(0x0400, 0x00000000);
    }

    printf("[CS132M] Camera %d %s\r\n", (int)cam_index,
           enable_trigger ? "ARMED in software-trigger mode" : "returned to stopped state");
    return 0;
}

// Dump the registers that describe the MIPI link and the sensor's identity.
//
// Register set and semantics follow the official VEYE Linux driver
// (veyeimaging/rk35xx_veye_bsp, linux/drivers/kernel_v5.10/veye_mvcam.c):
// there the host READS MIPI_DataRate (0x840, in kbps) from the camera and sets
// its own receiver link frequency to datarate/2. Our Gowin hard D-PHY runs at a
// fixed rate, so if the camera transmits at a different one the lanes do not
// sample correctly - which shows up as one data lane delivering mostly zeros.
static void camera_dump_link_info_bus(uint8_t cam_index)
{
    struct { uint16_t reg; const char *name; } regs[] = {
        { 0x0000, "Manufacturer   " }, { 0x0004, "Model code     " },
        { 0x0008, "Sensor code    " }, { 0x0024, "Error code     " },
        { 0x0028, "Format Cap     " }, { 0x0030, "TriggerMode Cap" },
        { 0x0034, "LaneNum Cap    " }, { 0x083C, "Lane_Num       " },
        { 0x0840, "MIPI_DataRate  " }, { 0x0804, "Pixel_Format   " },
        { 0x0800, "Test_Image     " }, { 0x0808, "Sensor_Width   " },
        { 0x080C, "Sensor_Height  " }, { 0x0818, "ROI_Width      " },
        { 0x081C, "ROI_Height     " }, { 0x0810, "MaxFrame_Rate  " },
        { 0x0814, "Framerate      " }, { 0x0404, "Trigger_Mode   " },
        { 0x0408, "Trigger_Source " },
    };
    uint32_t v;
    int i;

    camera_select_channel_bus(cam_index);
    printf("[CS132M] ---- Camera %d link / identity registers ----\r\n", (int)cam_index);
    for (i = 0; i < (int)(sizeof(regs) / sizeof(regs[0])); i++)
    {
        v = 0;
        if (camera_read_reg_status_bus(regs[i].reg, &v) == 0)
            printf("[CS132M]   0x%04X %s = 0x%08X (%u)\r\n",
                   regs[i].reg, regs[i].name, (unsigned int)v, (unsigned int)v);
        else
            printf("[CS132M]   0x%04X %s = <read failed>\r\n", regs[i].reg, regs[i].name);
        board_delay_us(2000);
    }
    printf("[CS132M] ---- end ----\r\n");
}

// Start/stop image acquisition (VEYE reg 0x0400, write-only).
// Stopping the sensor is the only way to freeze the DDR3 ring from the CPU: the
// FPGA capture path has no register interface, and the SoC's GPIO bus cannot be
// read by FPGA logic (inout pads), so there is no hardware capture-enable line.
static int camera_set_acquisition_bus(uint8_t cam_index, uint8_t on)
{
    if (cam_index > 1) return -1;
    camera_select_channel_bus(cam_index);
    camera_write_reg_bus(0x0400, on ? 0x00000001 : 0x00000000);
    return 0;
}

static int camera_trigger_once_bus(uint8_t cam_index)
{
    if (cam_index > 1) return -1;
    camera_select_channel_bus(cam_index);
    camera_write_reg_bus(0x0414, 0x00000001); // Trigger Software: write 1 = one trigger
    return 0;
}

static int camera_read_trigger_count_bus(uint8_t cam_index, uint32_t *total, uint32_t *lost)
{
    uint32_t v = 0;
    if (cam_index > 1) return -1;
    camera_select_channel_bus(cam_index);
    if (camera_read_reg_status_bus(0x0418, &v) != 0) return -2;
    if (total) *total = v & 0xFFFF;         // low 16 bits: total triggers
    if (lost)  *lost  = (v >> 16) & 0xFFFF; // high 16 bits: triggers lost
    return 0;
}

void camera_init(void)
{
    printf("[CS132M] Initializing camera system on RISC-V...\r\n");

    // Configure GPIO for camera selection multiplexer
    pDrv_GPIO->SetDir((1 << I2C_MUX_S0_PIN) | (1 << 3), AE350_GPIO_DIR_OUTPUT);

    // Initial power-up stabilization delay
    for (volatile int i = 0; i < 100000; i++) {}

    // AE350 hardware I2C driver initialization (i2c_bus.c)
    i2c_bus_init();

    s_state_lock = xSemaphoreCreateMutex();
    configASSERT(s_state_lock != NULL);

    printf("[CS132M] Camera subsystem ready (Probing handled by FreeRTOS tasks).\r\n");
}


// ---- public functions: they take the bus themselves (i2c_bus.h, recursive lock) ------
// Whoever holds the bus for an unbroken sequence takes it again without
// waiting. Without the bus: int returns CAMERA_ERR_BUS_BUSY, a register read 0,
// void does nothing.

void camera_write_reg(uint16_t reg_addr, uint32_t reg_data)
{
    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return;
    camera_write_reg_bus(reg_addr, reg_data);
    i2c_bus_unlock();
}

int camera_read_reg_status(uint16_t reg_addr, uint32_t *val)
{
    int rc;

    if (val) *val = 0;
    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_read_reg_status_bus(reg_addr, val);
    i2c_bus_unlock();
    return rc;
}

int camera_scan_mux(uint8_t cam_index)
{
    int rc;

    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_scan_mux_bus(cam_index);
    i2c_bus_unlock();
    return rc;
}

void camera_select_channel(uint8_t channel)
{
    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return;
    camera_select_channel_bus(channel);
    i2c_bus_unlock();
}

int camera_boot_reset(uint8_t cam_index, const camera_params_t *boot)
{
    int rc;

    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_boot_reset_bus(cam_index, boot);
    i2c_bus_unlock();
    return rc;
}

int camera_configure(uint8_t cam_index, uint8_t trg_mode, uint8_t trg_src, uint8_t auto_exp, uint32_t exp, uint32_t gain, uint32_t fps, uint8_t testpat, uint32_t roi_w, uint32_t roi_h, uint32_t offset_x, uint32_t offset_y)
{
    int rc;

    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_configure_bus(cam_index, trg_mode, trg_src, auto_exp, exp, gain, fps, testpat, roi_w, roi_h, offset_x, offset_y);
    i2c_bus_unlock();
    return rc;
}

int camera_read_config(uint8_t cam_index, bool *connected, uint32_t *model_id,
                       uint32_t *sensor_w, uint32_t *sensor_h,
                       uint8_t *trg_mode, uint8_t *trg_src,
                       uint8_t *auto_exp, uint32_t *exp, uint32_t *gain,
                       uint32_t *fps, uint32_t *max_fps,
                       uint32_t *err_code, uint8_t *testpat,
                       uint32_t *roi_w, uint32_t *roi_h,
                       uint32_t *offset_x, uint32_t *offset_y)
{
    int rc;

    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_read_config_bus(cam_index, connected, model_id, sensor_w, sensor_h, trg_mode, trg_src, auto_exp, exp, gain, fps, max_fps, err_code, testpat, roi_w, roi_h, offset_x, offset_y);
    i2c_bus_unlock();
    return rc;
}

void camera_load_default_config(uint8_t cam_index)
{
    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return;
    camera_load_default_config_bus(cam_index);
    i2c_bus_unlock();
}

int camera_set_trigger_mode(uint8_t cam_index, uint8_t enable_trigger, uint8_t frames_per_trigger)
{
    int rc;

    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_set_trigger_mode_bus(cam_index, enable_trigger, frames_per_trigger);
    i2c_bus_unlock();
    return rc;
}

void camera_dump_link_info(uint8_t cam_index)
{
    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return;
    camera_dump_link_info_bus(cam_index);
    i2c_bus_unlock();
}

int camera_set_acquisition(uint8_t cam_index, uint8_t on)
{
    int rc;

    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_set_acquisition_bus(cam_index, on);
    i2c_bus_unlock();
    return rc;
}

int camera_trigger_once(uint8_t cam_index)
{
    int rc;

    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_trigger_once_bus(cam_index);
    i2c_bus_unlock();
    return rc;
}

int camera_read_trigger_count(uint8_t cam_index, uint32_t *total, uint32_t *lost)
{
    int rc;

    if (!i2c_bus_lock(CAMERA_BUS_WAIT_MS)) return CAMERA_ERR_BUS_BUSY;
    rc = camera_read_trigger_count_bus(cam_index, total, lost);
    i2c_bus_unlock();
    return rc;
}


// ---- camera state (until 24.09.2026 the globals g_camera_memory, xCameraMutex) ----


void camera_state_store(uint8_t cam_index, const camera_params_t *p)
{
    if (cam_index > 1 || p == NULL) return;
    if (xSemaphoreTake(s_state_lock, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        s_state[cam_index] = *p;
        xSemaphoreGive(s_state_lock);
    }
}

bool camera_state_get(uint8_t cam_index, camera_params_t *out)
{
    bool ok = false;

    if (cam_index > 1 || out == NULL) return false;
    if (xSemaphoreTake(s_state_lock, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        *out = s_state[cam_index];
        xSemaphoreGive(s_state_lock);
        ok = true;
    }
    return ok;
}

// No lock, as before: one byte, written only by this camera's polling task.
bool camera_connected(uint8_t cam_index)
{
    return cam_index <= 1 && s_state[cam_index].connected;
}

int camera_mux_position(uint8_t cam_index)
{
    return (cam_index <= 1) ? s_cam_mux[cam_index] : -1;
}
