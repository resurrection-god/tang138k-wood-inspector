/*
 * lcd_hud.c - Headless Ethernet FPGA NPU v3.0.0 (SIMD-16) Wood Defect Inspector
 *             Spatial FCN Engine (256x96 RGB888 -> 16x6x5 Grid) + UDP :5555 Server.
 */

#include "lcd_hud.h"
#include "npu_regs.h"
#include "npu_weights.h"
#include "ota.h"
#include "net_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <stdio.h>
#include <string.h>

#define WOOD_APBW_MAGIC_REG (*(volatile uint32_t *)(LCD_HUD_BASE + 0x000U))
#define WOOD_MAGIC_VAL      0x574F4F44UL /* "WOOD" */

/* Protocol "WD01" (UDP port 5555) */
#define WD_PROTO_MAGIC      0x31304457UL /* "WD01" LE */
#define WD_CMD_PING         0x01U
#define WD_CMD_IMAGE_CHUNK  0x04U
#define WD_CMD_NPU_QUERY    0x07U

#define WOOD_FRAME_BYTES    (WOOD_IN_W * WOOD_IN_H * 3U)      /* 73,728 bytes */
#define WOOD_GRID_CELLS     (WOOD_GRID_W * WOOD_GRID_H)       /* 96 cells     */

static bool                 s_active = false;
static bool                 s_npu_ok = false;
static SemaphoreHandle_t    s_mtx = NULL;

/* Frame & feature buffers in DDR3 */
static uint8_t  s_rgb_frame[WOOD_FRAME_BYTES];
static int8_t   s_in_i8[WOOD_FRAME_BYTES]                    __attribute__((aligned(4)));
static int8_t   s_feat_a[128U * 48U * 24U]                   __attribute__((aligned(4)));
static int8_t   s_feat_b[128U * 48U * 24U]                   __attribute__((aligned(4)));
static int8_t   s_br1[64U * 24U * 24U]                       __attribute__((aligned(4)));
static int8_t   s_br2[64U * 24U * 24U]                       __attribute__((aligned(4)));
static int8_t   s_br3[64U * 24U * 24U]                       __attribute__((aligned(4)));

/* Inference results & telemetry */
static volatile uint16_t s_infer_cnt    = 0U;
static volatile uint32_t s_udp_rx_cnt   = 0U;
static volatile uint32_t s_exec_ns      = 0U;
static volatile uint32_t s_mac_cnt      = 0U;
static volatile uint32_t s_wall_us      = 0U;
static volatile uint32_t s_npu_hw_hash  = 0U;
static volatile uint8_t  s_board_grade  = 0U; /* 0=A(Clear), 1=B(Light), 2=C(Medium), 3=Reject */
static volatile uint16_t s_defect_cells = 0U;

static uint8_t s_grid_classes[WOOD_GRID_CELLS];
static uint8_t s_grid_conf[WOOD_GRID_CELLS];
static int16_t s_grid_logits[WOOD_GRID_CELLS * WOOD_NUM_CLASSES];

/* ============================================================================
 * Gowin SIMD-16 NPU Drivers (0xF8000900)
 * ============================================================================ */

static void npu_load_layer(const uint32_t *wgt_words, uint32_t vec_words, uint32_t out_ch, const int32_t *bias)
{
    uint32_t quad_words = (vec_words + 3U) & ~3U;
    uint32_t total_w = out_ch * quad_words;

    NPU_REG_DIMS = ((out_ch & 0x1FFU) << NPU_DIMS_OUT_CH_SHIFT)
                 | (vec_words & 0x1FFU);

    NPU_REG_WGT_ADDR = 0U;
    for (uint32_t i = 0; i < total_w; ++i) {
        NPU_REG_WGT_DATA4_RAW = wgt_words[i];
    }

    NPU_REG_BIAS_ADDR = 0U;
    for (uint32_t oc = 0; oc < out_ch; ++oc) {
        NPU_REG_BIAS_DATA_RAW = (uint32_t)bias[oc];
    }
}

static uint32_t npu_exec_pixel_i8(const uint32_t *act_words, uint32_t vec_words,
                                  uint32_t out_ch, uint32_t shift_bits, bool relu_en,
                                  uint32_t *out_packed)
{
    NPU_REG_ACT_ADDR = 0U;
    for (uint32_t i = 0; i < vec_words; ++i) {
        NPU_REG_ACT_DATA4_RAW = act_words[i];
    }

    NPU_REG_CTRL = NPU_CTRL_START_MASK
                 | (relu_en ? NPU_CTRL_RELU_EN_MASK : 0U)
                 | NPU_CTRL_CLAMP_I8_MASK
                 | ((shift_bits & 0x1FU) << NPU_CTRL_SHIFT_BITS_SHIFT)
                 | NPU_CTRL_ROUND_EN_MASK;

    for (uint32_t spin = 0; spin < 65536U; ++spin) {
        if ((NPU_REG_STATUS & NPU_STATUS_DONE_MASK) != 0U) break;
    }

    uint32_t out_words = (out_ch + 3U) / 4U;
    NPU_REG_OUT_ADDR = 0U;
    for (uint32_t i = 0; i < out_words; ++i) {
        out_packed[i] = NPU_REG_OUT_PACKED_RAW;
    }
    return NPU_REG_EXEC_TIME_NS;
}

static uint32_t npu_exec_pixel_i32(const uint32_t *act_words, uint32_t vec_words,
                                   uint32_t out_ch, uint32_t shift_bits, bool relu_en,
                                   int32_t *out_i32)
{
    NPU_REG_ACT_ADDR = 0U;
    for (uint32_t i = 0; i < vec_words; ++i) {
        NPU_REG_ACT_DATA4_RAW = act_words[i];
    }

    NPU_REG_CTRL = NPU_CTRL_START_MASK
                 | (relu_en ? NPU_CTRL_RELU_EN_MASK : 0U)
                 | ((shift_bits & 0x1FU) << NPU_CTRL_SHIFT_BITS_SHIFT)
                 | NPU_CTRL_ROUND_EN_MASK;

    for (uint32_t spin = 0; spin < 65536U; ++spin) {
        if ((NPU_REG_STATUS & NPU_STATUS_DONE_MASK) != 0U) break;
    }

    NPU_REG_OUT_ADDR = 0U;
    for (uint32_t i = 0; i < out_ch; ++i) {
        out_i32[i] = (int32_t)NPU_REG_OUT_DATA_RAW;
    }
    return NPU_REG_EXEC_TIME_NS;
}

/* Conv1 3x3 stride 2 pad 1 (96x256x3 -> 48x128x24) executed on SIMD-16 NPU */
static uint32_t run_conv1_3x3_s2_npu(const int8_t *in_hwc, int8_t *out_hwc)
{
    uint32_t total_ns = 0U;
    uint32_t patch_words[7];

    npu_load_layer(&k_npu_wgt_pool[NPU_CONV1_WO], 7U, 24U, &k_npu_bias_pool[NPU_CONV1_BO]);

    for (int32_t oy = 0; oy < 48; ++oy) {
        for (int32_t ox = 0; ox < 128; ++ox) {
            uint8_t patch[28];
            uint32_t p = 0U;
            for (int32_t ky = -1; ky <= 1; ++ky) {
                int32_t iy = oy * 2 + ky;
                for (int32_t kx = -1; kx <= 1; ++kx) {
                    int32_t ix = ox * 2 + kx;
                    if (iy >= 0 && iy < (int32_t)WOOD_IN_H && ix >= 0 && ix < (int32_t)WOOD_IN_W) {
                        uint32_t idx = (uint32_t)((iy * (int32_t)WOOD_IN_W + ix) * 3);
                        patch[p + 0U] = (uint8_t)in_hwc[idx + 0U];
                        patch[p + 1U] = (uint8_t)in_hwc[idx + 1U];
                        patch[p + 2U] = (uint8_t)in_hwc[idx + 2U];
                    } else {
                        patch[p + 0U] = 0U;
                        patch[p + 1U] = 0U;
                        patch[p + 2U] = 0U;
                    }
                    p += 3U;
                }
            }
            patch[27] = 0U;
            memcpy(patch_words, patch, sizeof(patch));
            total_ns += npu_exec_pixel_i8(
                patch_words, 7U, 24U, NPU_CONV1_SHIFT, true,
                (uint32_t *)&out_hwc[(oy * 128 + ox) * 24]
            );
        }
    }
    return total_ns;
}

/* MaxPool2D 3x3 stride 2 pad 1 (48x128x24 -> 24x64x24, input >= 0) */
static void run_maxpool3x3_s2(const int8_t *in_hwc, int8_t *out_hwc)
{
    for (int32_t oy = 0; oy < 24; ++oy) {
        for (int32_t ox = 0; ox < 64; ++ox) {
            int8_t *dst = &out_hwc[(oy * 64 + ox) * 24];
            for (int32_t c = 0; c < 24; ++c) {
                int8_t mx = 0;
                for (int32_t ky = 0; ky < 3; ++ky) {
                    int32_t iy = oy * 2 + ky - 1;
                    if (iy < 0 || iy >= 48) continue;
                    for (int32_t kx = 0; kx < 3; ++kx) {
                        int32_t ix = ox * 2 + kx - 1;
                        if (ix < 0 || ix >= 128) continue;
                        int8_t v = in_hwc[(iy * 128 + ix) * 24 + c];
                        if (v > mx) mx = v;
                    }
                }
                dst[c] = mx;
            }
        }
    }
}

/* Depthwise 3x3 Conv2D with configurable stride & dilation (pad = dilation) */
static void run_dw3x3(const int8_t *in_hwc, int32_t H, int32_t W, int32_t C,
                      int32_t stride, int32_t dil,
                      const int8_t *dw_w, const int32_t *dw_b,
                      uint32_t shift_bits, bool relu_en, int8_t *out_hwc)
{
    int32_t out_H = (stride == 2) ? (H / 2) : H;
    int32_t out_W = (stride == 2) ? (W / 2) : W;
    int32_t round_bias = (shift_bits > 0U) ? (1 << (shift_bits - 1U)) : 0;

    for (int32_t oy = 0; oy < out_H; ++oy) {
        for (int32_t ox = 0; ox < out_W; ++ox) {
            int8_t *dst = &out_hwc[(oy * out_W + ox) * C];
            for (int32_t c = 0; c < C; ++c) {
                int32_t acc = dw_b[c];
                const int8_t *wc = &dw_w[c * 9];
                for (int32_t ky = 0; ky < 3; ++ky) {
                    int32_t iy = oy * stride + (ky - 1) * dil;
                    if (iy < 0 || iy >= H) continue;
                    for (int32_t kx = 0; kx < 3; ++kx) {
                        int32_t ix = ox * stride + (kx - 1) * dil;
                        if (ix < 0 || ix >= W) continue;
                        acc += (int32_t)in_hwc[(iy * W + ix) * C + c] * (int32_t)wc[ky * 3 + kx];
                    }
                }
                int32_t val = (acc + round_bias) >> shift_bits;
                if (relu_en && val < 0) val = 0;
                else if (val < -127) val = -127;
                if (val > 127) val = 127;
                dst[c] = (int8_t)val;
            }
        }
    }
}

/* Pointwise 1x1 Conv2D executed on SIMD-16 FPGA NPU */
static uint32_t run_pw1x1_npu(const int8_t *in_hwc, int32_t H, int32_t W,
                              int32_t C_in, int32_t C_out,
                              const uint32_t *w_packed, const int32_t *b,
                              uint32_t shift_bits, int8_t *out_hwc)
{
    uint32_t total_ns = 0U;
    uint32_t vec_words = (uint32_t)(C_in / 4);
    int32_t n_px = H * W;

    npu_load_layer(w_packed, vec_words, (uint32_t)C_out, b);

    for (int32_t i = 0; i < n_px; ++i) {
        const uint32_t *act_ptr = (const uint32_t *)&in_hwc[i * C_in];
        uint32_t *out_ptr = (uint32_t *)&out_hwc[i * C_out];
        total_ns += npu_exec_pixel_i8(act_ptr, vec_words, (uint32_t)C_out,
                                      shift_bits, true, out_ptr);
    }
    return total_ns;
}

static void shuffle_concat_2(const int8_t *b1_hwc, const int8_t *b2_hwc,
                             int32_t n_px, int32_t c_branch, int8_t *out_hwc)
{
    int32_t c_out = c_branch * 2;
    for (int32_t p = 0; p < n_px; ++p) {
        const int8_t *s1 = &b1_hwc[p * c_branch];
        const int8_t *s2 = &b2_hwc[p * c_branch];
        int8_t *dst = &out_hwc[p * c_out];
        for (int32_t c = 0; c < c_branch; ++c) {
            dst[2 * c + 0] = s1[c];
            dst[2 * c + 1] = s2[c];
        }
    }
}

static void wood_npu_infer(const uint8_t *rgb888_256x96)
{
    uint32_t total_ns = 0U;
    int32_t cell_logits[WOOD_NUM_CLASSES];
    TickType_t tick_start;

    if (!s_npu_ok) return;
    tick_start = xTaskGetTickCount();

    /* Convert [0..255] -> [-127..127] signed INT8 */
    for (uint32_t i = 0; i < WOOD_FRAME_BYTES; ++i) {
        int32_t v = (int32_t)rgb888_256x96[i] - 128;
        if (v < -127) v = -127;
        else if (v > 127) v = 127;
        s_in_i8[i] = (int8_t)v;
    }

    NPU_REG_CTRL = NPU_CTRL_CLEAR_MASK;

    /* 1. Conv1 3x3 s2 (96x256x3 -> 48x128x24) on NPU + MaxPool -> 24x64x24 */
    total_ns += run_conv1_3x3_s2_npu(s_in_i8, s_feat_a);
    run_maxpool3x3_s2(s_feat_a, s_feat_b);

    int8_t *cur_in  = s_feat_b;
    int8_t *cur_out = s_feat_a;

    /* 2. All 16 ShuffleNetV2 blocks (Stage 2, Stage 3, Stage 4 with s4.0 dil=2) */
    for (uint32_t bi = 0; bi < NPU_NUM_BLOCKS; ++bi) {
        const NpuWoodBlockDesc *d = &k_wood_blocks[bi];
        int32_t H   = (int32_t)d->h_in;
        int32_t W   = (int32_t)d->w_in;
        int32_t C_b = (int32_t)d->c_branch;
        int32_t s   = (int32_t)d->dw_stride;
        int32_t dil = (int32_t)d->dw_dilation;

        if (d->two_branch != 0U) {
            int32_t oH = (s == 2) ? (H / 2) : H;
            int32_t oW = (s == 2) ? (W / 2) : W;

            run_dw3x3(cur_in, H, W, C_b, s, dil,
                      &k_dw_wgt_pool[d->b1_dw_wo], &k_dw_bias_pool[d->b1_dw_bo],
                      d->b1_dw_shift, false, s_br2);
            total_ns += run_pw1x1_npu(s_br2, oH, oW, C_b, C_b,
                                      &k_npu_wgt_pool[d->b1_pw_wo], &k_npu_bias_pool[d->b1_pw_bo],
                                      d->b1_pw_shift, s_br1);

            total_ns += run_pw1x1_npu(cur_in, H, W, C_b, C_b,
                                      &k_npu_wgt_pool[d->b2_pw1_wo], &k_npu_bias_pool[d->b2_pw1_bo],
                                      d->b2_pw1_shift, s_br2);
            run_dw3x3(s_br2, H, W, C_b, s, dil,
                      &k_dw_wgt_pool[d->b2_dw_wo], &k_dw_bias_pool[d->b2_dw_bo],
                      d->b2_dw_shift, false, s_br3);
            total_ns += run_pw1x1_npu(s_br3, oH, oW, C_b, C_b,
                                      &k_npu_wgt_pool[d->b2_pw2_wo], &k_npu_bias_pool[d->b2_pw2_bo],
                                      d->b2_pw2_shift, s_br2);

            shuffle_concat_2(s_br1, s_br2, oH * oW, C_b, cur_out);
        } else {
            int32_t n_px = H * W;
            int32_t C_in = (int32_t)d->c_in;
            for (int32_t p = 0; p < n_px; ++p) {
                memcpy(&s_br1[p * C_b], &cur_in[p * C_in], (size_t)C_b);
                memcpy(&s_br2[p * C_b], &cur_in[p * C_in + C_b], (size_t)C_b);
            }

            total_ns += run_pw1x1_npu(s_br2, H, W, C_b, C_b,
                                      &k_npu_wgt_pool[d->b2_pw1_wo], &k_npu_bias_pool[d->b2_pw1_bo],
                                      d->b2_pw1_shift, s_br3);
            run_dw3x3(s_br3, H, W, C_b, s, dil,
                      &k_dw_wgt_pool[d->b2_dw_wo], &k_dw_bias_pool[d->b2_dw_bo],
                      d->b2_dw_shift, false, s_br2);
            total_ns += run_pw1x1_npu(s_br2, H, W, C_b, C_b,
                                      &k_npu_wgt_pool[d->b2_pw2_wo], &k_npu_bias_pool[d->b2_pw2_bo],
                                      d->b2_pw2_shift, s_br3);

            shuffle_concat_2(s_br1, s_br3, n_px, C_b, cur_out);
        }

        int8_t *tmp = cur_in;
        cur_in = cur_out;
        cur_out = tmp;
    }

    /* 3. Conv5 1x1 PW (6x16x192 -> 6x16x128) on NPU */
    total_ns += run_pw1x1_npu(cur_in, (int32_t)WOOD_GRID_H, (int32_t)WOOD_GRID_W, 192, 128,
                              &k_npu_wgt_pool[NPU_CONV5_WO], &k_npu_bias_pool[NPU_CONV5_BO],
                              NPU_CONV5_SHIFT, cur_out);

    /* 4. Spatial Segmentation Head (6x16 cells x 128 -> 6x16x5 INT32 logits) on NPU */
    npu_load_layer(&k_npu_wgt_pool[NPU_HEAD_WO], 32U, WOOD_NUM_CLASSES, &k_npu_bias_pool[NPU_HEAD_BO]);
    uint16_t def_cnt = 0U;
    uint16_t dead_crack_cnt = 0U;

    for (uint32_t p = 0; p < WOOD_GRID_CELLS; ++p) {
        total_ns += npu_exec_pixel_i32(
            (const uint32_t *)&cur_out[p * 128U], 32U, WOOD_NUM_CLASSES, NPU_HEAD_SHIFT, false, cell_logits
        );
        uint8_t best_c = 0U;
        int32_t best_v = cell_logits[0];
        for (uint32_t c = 0; c < WOOD_NUM_CLASSES; ++c) {
            int32_t lv = cell_logits[c];
            if (lv > 32767) lv = 32767;
            else if (lv < -32767) lv = -32767;
            s_grid_logits[p * WOOD_NUM_CLASSES + c] = (int16_t)lv;
            if (cell_logits[c] > best_v) {
                best_v = cell_logits[c];
                best_c = (uint8_t)c;
            }
        }
        int32_t second_v = -32767;
        for (uint32_t c = 0; c < WOOD_NUM_CLASSES; ++c) {
            if (c != best_c && cell_logits[c] > second_v) second_v = cell_logits[c];
        }
        int32_t conf = 52 + ((best_v - second_v) / 10);
        if (conf < 50) conf = 50;
        if (conf > 99) conf = 99;
        s_grid_classes[p] = best_c;
        s_grid_conf[p] = (uint8_t)conf;
        if (best_c > 0U) {
            ++def_cnt;
            if (best_c == 2U || best_c == 3U) ++dead_crack_cnt;
        }
    }

    s_defect_cells = def_cnt;
    if (def_cnt == 0U) {
        s_board_grade = 0U; /* Grade A (Clear) */
    } else if (dead_crack_cnt == 0U && def_cnt <= 4U) {
        s_board_grade = 1U; /* Grade B (Light live knots) */
    } else if (dead_crack_cnt <= 3U && def_cnt <= 12U) {
        s_board_grade = 2U; /* Grade C (Medium) */
    } else {
        s_board_grade = 3U; /* Reject */
    }

    s_exec_ns  = total_ns;
    s_mac_cnt  = NPU_REG_MAC_COUNT;
    s_wall_us  = (uint32_t)((xTaskGetTickCount() - tick_start) * 1000U);
    ++s_infer_cnt;

    printf("[WOOD-NPU] #%-3u grade=%u defect_cells=%u/96 | hw=%lu us | macs=%lu\r\n",
           (unsigned int)s_infer_cnt, (unsigned int)s_board_grade, (unsigned int)def_cnt,
           (unsigned long)(s_exec_ns / 1000U), (unsigned long)s_mac_cnt);
}

/* ============================================================================
 * Headless UDP Server ("WD01" @ port 5555)
 * ============================================================================ */
static void wood_udp_task(void *arg)
{
    static uint8_t rx_buf[1460];
    static uint8_t rep_buf[1184];
    int sock;
    struct sockaddr_in addr, cli_addr;
    socklen_t cli_len;

    (void)arg;

    for (;;) {
        sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
        if (sock >= 0) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    int rcvbuf = 131072;
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(LCD_UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (lwip_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        lwip_close(sock);
        vTaskDelete(NULL);
        return;
    }

    printf("[WOOD_UDP] Listening on UDP :%u (SIMD-16 Headless NPU, 256x96 -> 16x6)\r\n",
           (unsigned int)LCD_UDP_PORT);

    for (;;) {
        cli_len = sizeof(cli_addr);
        int n = lwip_recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
                              (struct sockaddr *)&cli_addr, &cli_len);
        if (n < 8) continue;

        uint32_t magic = (uint32_t)rx_buf[0]
                       | ((uint32_t)rx_buf[1] << 8)
                       | ((uint32_t)rx_buf[2] << 16)
                       | ((uint32_t)rx_buf[3] << 24);
        if (magic != WD_PROTO_MAGIC) continue;

        uint8_t cmd = rx_buf[4];
        ++s_udp_rx_cnt;

        if (cmd == WD_CMD_PING) {
            uint8_t rep[20];
            uint16_t ic = s_infer_cnt;
            uint16_t rx = (uint16_t)(s_udp_rx_cnt & 0xFFFFU);
            uint32_t mh = s_npu_hw_hash;
            memset(rep, 0, sizeof(rep));
            rep[0] = 'W'; rep[1] = 'D'; rep[2] = '0'; rep[3] = '1';
            rep[4] = 0x81U;
            rep[5] = s_npu_ok ? 1U : 0U;
            rep[6] = WOOD_GRID_W;
            rep[7] = WOOD_GRID_H;
            rep[8] = WOOD_NUM_CLASSES;
            rep[9] = 16U; /* SIMD-16 */
            memcpy(&rep[12], &ic, 2);
            memcpy(&rep[14], &rx, 2);
            memcpy(&rep[16], &mh, 4);
            lwip_sendto(sock, rep, sizeof(rep), 0, (struct sockaddr *)&cli_addr, cli_len);
        } else if (cmd == WD_CMD_IMAGE_CHUNK && n >= 12) {
            uint32_t offset = (uint32_t)rx_buf[5]
                            | ((uint32_t)rx_buf[6] << 8)
                            | ((uint32_t)rx_buf[7] << 16)
                            | ((uint32_t)rx_buf[8] << 24);
            uint16_t len    = (uint16_t)rx_buf[9] | ((uint16_t)rx_buf[10] << 8);
            uint8_t  flags  = rx_buf[11];

            if ((int)(12U + len) <= n && (offset + len) <= WOOD_FRAME_BYTES) {
                bool ran = false;
                if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                    memcpy(&s_rgb_frame[offset], &rx_buf[12], len);
                    if ((flags & 1U) || (offset + len >= WOOD_FRAME_BYTES)) {
                        wood_npu_infer(s_rgb_frame);
                        ran = true;
                    }
                    xSemaphoreGive(s_mtx);
                }
                uint8_t ack[12];
                uint16_t dc = s_defect_cells;
                ack[0] = 'W'; ack[1] = 'D'; ack[2] = '0'; ack[3] = '1';
                ack[4] = 0x84U;
                memcpy(&ack[5], &offset, 4);
                ack[9]  = s_npu_ok ? 1U : 0U;
                ack[10] = ran ? 1U : 0U;
                ack[11] = (uint8_t)(dc & 0xFFU);
                lwip_sendto(sock, ack, sizeof(ack), 0, (struct sockaddr *)&cli_addr, cli_len);
            }
        } else if (cmd == WD_CMD_NPU_QUERY) {
            /* Packet size = 24 + 96 + 96 + 960 = 1,176 bytes (single Ethernet UDP frame) */
            uint16_t ic = s_infer_cnt;
            uint32_t ns = s_exec_ns;
            uint32_t mc = s_mac_cnt;
            uint32_t wu = s_wall_us;
            memset(rep_buf, 0, 24);
            rep_buf[0]  = 'W'; rep_buf[1] = 'D'; rep_buf[2] = '0'; rep_buf[3] = '1';
            rep_buf[4]  = 0x87U;
            rep_buf[5]  = s_npu_ok ? 1U : 0U;
            rep_buf[6]  = WOOD_GRID_W;
            rep_buf[7]  = WOOD_GRID_H;
            memcpy(&rep_buf[8], &ic, 2);
            rep_buf[10] = WOOD_NUM_CLASSES;
            rep_buf[11] = s_board_grade;
            memcpy(&rep_buf[12], &ns, 4);
            memcpy(&rep_buf[16], &mc, 4);
            memcpy(&rep_buf[20], &wu, 4);

            memcpy(&rep_buf[24],  s_grid_classes, WOOD_GRID_CELLS);
            memcpy(&rep_buf[120], s_grid_conf,    WOOD_GRID_CELLS);
            memcpy(&rep_buf[216], s_grid_logits,  WOOD_GRID_CELLS * WOOD_NUM_CLASSES * sizeof(int16_t));

            lwip_sendto(sock, rep_buf, 24 + WOOD_GRID_CELLS * 2 + WOOD_GRID_CELLS * WOOD_NUM_CLASSES * 2,
                        0, (struct sockaddr *)&cli_addr, cli_len);
        }
    }
}

/* ============================================================================
 * Public API & Headless Compatibility Stubs
 * ============================================================================ */

void lcd_hud_init(void)
{
    if (ota_current_slot() == 0U) {
        s_active = false;
        return;
    }

    uint32_t wmagic = WOOD_APBW_MAGIC_REG;
    if (wmagic != WOOD_MAGIC_VAL) {
        printf("[WOOD] Warning: APB wrapper magic=0x%08lX (expected 0x%08lX)\r\n",
               (unsigned long)wmagic, (unsigned long)WOOD_MAGIC_VAL);
    }

    /* Mandatory regmap-codegen MAP_HASH verification between firmware and bitstream */
    s_npu_hw_hash = NPU_REG_ID;
    if (s_npu_hw_hash != NPU_MAP_HASH) {
        printf("[WOOD-NPU] ERROR: RegmapMismatch! HW=0x%08lX != FW=0x%08lX\r\n",
               (unsigned long)s_npu_hw_hash, (unsigned long)NPU_MAP_HASH);
        s_npu_ok = false;
    } else {
        s_npu_ok = true;
        printf("[WOOD-NPU] Headless SIMD-16 NPU v3 Ready (MAP_HASH=0x%08lX)\r\n",
               (unsigned long)s_npu_hw_hash);
    }

    s_mtx = xSemaphoreCreateMutex();
    if (s_mtx == NULL) {
        s_active = false;
        return;
    }

    memset(s_rgb_frame, 180, sizeof(s_rgb_frame));
    if (s_npu_ok) {
        wood_npu_infer(s_rgb_frame);
    }

    s_active = true;
    xTaskCreate(wood_udp_task, "wood_udp", 1536U, NULL, tskIDLE_PRIORITY + 3U, NULL);
    printf("[WOOD] Headless UDP :%u Wood Defect Server Active\r\n", (unsigned int)LCD_UDP_PORT);
}

bool lcd_hud_is_active(void) { return s_active; }
void lcd_hud_tick(uint32_t rx_pkts, uint32_t tx_pkts) { (void)rx_pkts; (void)tx_pkts; }
void lcd_hud_set_pc_state(const char *state, uint8_t color) { (void)state; (void)color; }
void lcd_hud_log_exchange(uint8_t color, const char *proto, const char *msg) { (void)color; (void)proto; (void)msg; }
void lcd_hud_pulse_rx(void) {}
void lcd_hud_submit_camera_frame(const uint8_t *fb_200x120, uint32_t frame_id, bool auto_switch) { (void)fb_200x120; (void)frame_id; (void)auto_switch; }
bool lcd_hud_has_hw_gvsp(void) { return false; }
void lcd_hud_set_hw_gvsp_mode(bool hw_en, bool mono8) { (void)hw_en; (void)mono8; }
void lcd_hud_set_link_status(bool link_ok, uint32_t cam_ip) { (void)link_ok; (void)cam_ip; }
uint32_t lcd_hud_get_hw_gvsp_stat(void) { return 0U; }
uint32_t lcd_hud_get_hw_gvsp_drops(void) { return 0U; }
void lcd_hud_anim_next_mode(void) {}
void lcd_hud_anim_set_mode(lcd_anim_mode_t mode) { (void)mode; }
lcd_anim_mode_t lcd_hud_anim_get_mode(void) { return ANIM_MODE_IMAGE; }
const char *lcd_hud_anim_get_mode_name(lcd_anim_mode_t mode) { (void)mode; return "HEADLESS"; }
void lcd_hud_cycle_rotation(void) {}
void lcd_hud_toggle_resolution(void) {}
