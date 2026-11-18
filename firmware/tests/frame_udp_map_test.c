/*
 * frame_udp_map_test.c - fu_addr() (src/app/frame_udp_map.h) against the two maps it serves:
 * DDR3 (the blocks udp_rd polled until 30.10.2026) and apb_ctl.v's APB block (README of the
 * "control on APB" change: desc 0x200-0x25F, status 0x280-0x2BF, records 0x300-0x3FF, the
 * record status at +0x60). Built and run by gate_bench.py. Exit code = mismatches.
 */
#include <stdio.h>

#include "frame_udp_map.h"

static int bad;

static void eq(const char *what, unsigned a, unsigned want)
{
    if (a != want) {
        printf("FAIL %s: 0x%08X, want 0x%08X\n", what, a, want);
        ++bad;
    }
}

int main(void)
{
    // DDR3: the addresses frame_udp.h has always had
    eq("ddr desc w0", fu_addr(false, FU_BLK_DESC, 0), 0x06000400U);
    eq("ddr desc w23", fu_addr(false, FU_BLK_DESC, 23), 0x0600045CU);
    eq("ddr stat w15", fu_addr(false, FU_BLK_STAT, 15), 0x060004BCU);
    eq("ddr rec0 status", fu_addr(false, FU_BLK_REC0, 24), 0x06001160U);
    eq("ddr rec1 w0", fu_addr(false, FU_BLK_REC1, 0), 0x06001180U);
    // APB: 0xF8001000 + the README offsets, the last word of each region inside it
    eq("apb desc w0", fu_addr(true, FU_BLK_DESC, 0), 0xF8001200U);
    eq("apb desc w23", fu_addr(true, FU_BLK_DESC, 23), 0xF800125CU);
    eq("apb stat w0", fu_addr(true, FU_BLK_STAT, 0), 0xF8001280U);
    eq("apb stat w15", fu_addr(true, FU_BLK_STAT, 15), 0xF80012BCU);
    eq("apb rec0 w0", fu_addr(true, FU_BLK_REC0, 0), 0xF8001300U);
    eq("apb rec0 status", fu_addr(true, FU_BLK_REC0, 24), 0xF8001360U);
    eq("apb rec0 last", fu_addr(true, FU_BLK_REC0, 31), 0xF800137CU);
    eq("apb rec1 w0", fu_addr(true, FU_BLK_REC1, 0), 0xF8001380U);
    eq("apb rec1 status", fu_addr(true, FU_BLK_REC1, 24), 0xF80013E0U);
    eq("apb rec1 last", fu_addr(true, FU_BLK_REC1, 31), 0xF80013FCU);
    // apb_ctl.v's beat RAM: beat = {addr[12], addr[7:5]}, word = beat * 8 + lane
    {
        unsigned a = fu_addr(true, FU_BLK_REC1, 24) - FU_APB_BASE;     /* 0x3E0 */
        unsigned beat = (((a >> 9) & 1U) << 3) | ((a >> 5) & 7U), lane = (a >> 2) & 7U;
        eq("rec1 status beat", beat, 15U);
        eq("rec1 status lane", lane, 0U);
    }
    printf(bad ? "%d mismatches\n" : "frame_udp_map ok\n", bad);
    return bad;
}
