/*
 * ******************************************************************************************
 * File		: loader.c
 * Author	: GowinSemicoductor
 * Chip		: AE350_SOC
 * Function	: Build burn mode
 * ******************************************************************************************
 */

// Includes ---------------------------------------------------------------------------------
#include "config.h"


// Definitions ------------------------------------------------------------------------------

#ifdef CFG_BURN

extern unsigned long __flash_start, __text_start, __text_lmastart, _edata;
extern unsigned long __loader_start, __loader_lmaend, __loader_lmastart;
extern void _start(void);


/* The entry function when boot, executing on ROM/FLASH. */
void __attribute__((naked, no_execit, no_profile_instrument_function, section(".bootloader"))) boot_loader(void)
{
	register unsigned long *src_ptr, *dst_ptr;
	register unsigned long i, size;

	/* Prevent the use of $gp */
	asm volatile (
		".option norelax\n"
	);

	/* Copy loader() code to VMA area (On memory) */
	size = ((unsigned long)&__loader_lmaend - (unsigned long)&__loader_lmastart + (sizeof(long) - 1)) / sizeof(long);

	src_ptr = (unsigned long *)((unsigned long)&__flash_start + (unsigned long)&__loader_lmastart);
	dst_ptr = (unsigned long *)&__loader_start;

	for (i = 0; i < size; i++)
	{
		*dst_ptr++ = *src_ptr++;
	}

	/* Synchronize Instruction & Data Caches before executing loader in memory */
	asm volatile (
		"fence rw, rw\n"
		"li t0, 6\n"
		"csrw 0x7cc, t0\n"
		"fence.i\n"
	);

	/*
	 * Call the real program loader which executes on memory instead
	 * of flash for better copying performance.
	 */
	register void (*ldr_entry)(void) = (void *)&__loader_start;
	ldr_entry();
}

void __attribute__((naked, no_execit, no_profile_instrument_function, section(".loader"))) loader(void)
{
	register unsigned long *src_ptr, *dst_ptr;
	register unsigned long i, size;
	register volatile const unsigned long *meta = (volatile const unsigned long *)0x80950000UL;
	register unsigned long slot_offset = 0;
	register unsigned long end_addr;

	/* Check OTA metadata at physical Flash 0xF50000 (ROM window 0x80950000):
	 * word 0: magic (0x4F544131 "OTA1")
	 * word 1: active_slot (1 = Slot B at 0xF60000)
	 * word 5: slot_b_size (valid range 10 KB .. 512 KB)
	 * word 7: boot_attempts (< 3)
	 */
	if (meta[0] == 0x4F544131UL && meta[1] == 1UL && meta[7] < 3UL &&
	    meta[5] >= 10240UL && meta[5] <= 524288UL)
	{
		slot_offset = 0x00960000UL;  /* 0x80000000 + 0x00960000 = 0x80960000 -> Flash 0xF60000 */
	}

	/* Copy code bank to VMA area: use meta[5] (slot_b_size) for Slot B so images
	 * larger than Golden Slot A are never truncated. */
	if (slot_offset != 0UL && meta[5] > (unsigned long)&__text_lmastart) {
		unsigned long payload_bytes = meta[5] - (unsigned long)&__text_lmastart;
		size = (payload_bytes + (sizeof(long) - 1)) / sizeof(long);
	} else {
		size = ((unsigned long)&_edata - (unsigned long)&__text_start + (sizeof(long) - 1)) / sizeof(long);
	}

	/* Copy code bank to VMA area */
	src_ptr = (unsigned long *)((unsigned long)&__flash_start + slot_offset + (unsigned long)&__text_lmastart);
	dst_ptr = (unsigned long *)&__text_start;

	for (i = 0; i < size; i++)
	{
		*dst_ptr++ = *src_ptr++;
	}

	/* Synchronize Instruction & Data Caches / Write Buffers before executing from RAM */
	asm volatile (
		"fence rw, rw\n"
		"li t0, 6\n"          /* CCTL_L1D_WBINVAL_ALL */
		"csrw 0x7cc, t0\n"
	);

	/* Invalidate entire loaded code range in I-Cache using VA-based invalidation (command 8) */
	end_addr = (unsigned long)&__text_start + size * sizeof(long);
	for (unsigned long addr = (unsigned long)&__text_start; addr < end_addr; addr += 32)
	{
		asm volatile(
			"csrw 0x7cb, %0\n"  /* NDS_MCCTLBEGINADDR */
			"li t0, 8\n"         /* CCTL_L1I_VA_INVAL */
			"csrw 0x7cc, t0\n"  /* NDS_MCCTLCOMMAND */
			: : "r"(addr) : "t0"
		);
	}

	asm volatile (
		"fence.i\n"
	);

	/* Go to entry function */
	_start();
}

#endif	// CFG_BURN
