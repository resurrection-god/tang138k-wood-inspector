/*
 * frame_udp_map.h - where udp_tx's control blocks live: DDR3 (bitstreams before 30.10.2026) or
 * the FPGA control block on the extended APB (apb_ctl.v, FEATURES bit 1). The layouts are the
 * same; word w of a block is at block + 4 w in either (APB: word = beat * 8 + lane).
 * Pure, so firmware/tests/frame_udp_map_test.c checks the map on the PC.
 */
#ifndef APP_FRAME_UDP_MAP_H_
#define APP_FRAME_UDP_MAP_H_

#include <stdint.h>
#include <stdbool.h>

#define FU_APB_BASE         0xF8001000U     /* apb_ctl.v; never touched unless FEATURES bit 1 */
#define FU_BLK_DESC         0U              /* descriptor, 24 words, written by the CPU       */
#define FU_BLK_STAT         1U              /* job result, 16 words, written by the FPGA      */
#define FU_BLK_REC0         2U              /* stream record camera 0, 32 words (+0x60 status) */
#define FU_BLK_REC1         3U              /* stream record camera 1                         */

// Address of word w of block blk.
static inline uint32_t fu_addr(bool apb, uint32_t blk, uint32_t w)
{
    static const uint32_t ddr[4] = { 0x06000400U, 0x06000480U, 0x06001100U, 0x06001180U };
    static const uint32_t off[4] = { 0x200U, 0x280U, 0x300U, 0x380U };

    return (apb ? FU_APB_BASE + off[blk & 3U] : ddr[blk & 3U]) + 4U * w;
}

#endif /* APP_FRAME_UDP_MAP_H_ */
