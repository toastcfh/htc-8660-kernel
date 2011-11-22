/*
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *  Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>
#include <linux/init.h>

/*
 * MSM specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 *
 * This is executing in physical space with cache's off.
 */

/*
The counters to check tz entry and exit for both cpu's
TZ entry for cpu0 = 0x40400000 -24  virt  0xFA822000 - 24
TZ exit for cpu0 = 0x40400000 -32 virt  0xFA822000 - 32
kernel entry for cpu0 = 0x40400000 -40 virt  0xFA822000 - 40
kernel entry for cpu1 = 0x40400000 -48 virt  0xFA822000 - 48
TZ entry for cpu0 = 0x40400000 - 56 virt  0xFA822000 - 56
TZ exit for cpu1 = 0x40400000 - 64  0xFA822000 - 64
TZ progress = 0x40400000 - 72  0xFA822000 - 72
TZ kernel_addr = 0x40400000 - 80  0xFA822000 -80
TZ error = 0x40400000 - 88  0xFA822000 -88
cpu1 rejection = 0x40400000 - 96  0xFA822000 -96
cpu1 wakeup reason = 0x40400000 - 104  0xFA822000 - 104
cpu1 read of ack register = 0x40400000 - 112  0xFA822000 - 104
*/

#define MSM_POWER_COLLAPSE_EXIT_1_COUNT_PHYS	(0x40400000 - 48)
#define MSM_PEN_PHYS	(0x40400000 - 96)
#define CPU1_WAKEUP_REASON_PHYS (0x40400000 - 104)
#define CPU1_READ_ACK_PHYS (0x40400000 - 112)

#define GIC_PENDING_PHYS (0x02080200)
#define GIC_ENABLED_PHYS (0x02080100)

#define GIC_ENABLE_CPU1_PHYS (0x02081000)
#define GIC_PRIMASK_CPU1_PHYS (0x02081000)
#define GIC_INTACK_CPU1_PHYS (0x0208100c)
#define GIC_EOI_CPU1_PHYS (0x02081010)

.macro inc_power_collapse_exit_count
	ldr	r0, =MSM_POWER_COLLAPSE_EXIT_1_COUNT_PHYS
	ldr     r4, [r0]
	add 	r4, r4, #1
	str	r4, [r0]
	dsb						/* ensure data are written. */
.endm

.macro get_wakeup_interrupt
	ldr	r1, =GIC_PENDING_PHYS
	ldr	r3, [r1]
	ldr	r1, =GIC_ENABLED_PHYS
	ldr	r9, [r1]
	and	r7, r9, r3 /*and pending and enabled bits */
	ldr	r1, =CPU1_WAKEUP_REASON_PHYS
	str	r7, [r1]
	dsb						/* ensure data are written. */
.endm

/* if we see the we are valid we write the First byte with AB*/
.macro mark_wakeup_interrupt_valid
	ldr	r0, =CPU1_WAKEUP_REASON_PHYS
	ldr	r7, [r0]
	orr	r7, r7, #0xAB000000
	str	r7, [r0]
	dsb						/* ensure data are written. */
.endm

/* read the intack and write to to eoi,
 this way we trick the gic in thinking that an interrupt is handled
r1 and r7 are addresses and values respt.
r4 is the cpu_ctrl enable, we read it, force enable and restore the read value
*/

.macro quiet_wakeup_interrupt
	ldr	r1, =GIC_PRIMASK_CPU1_PHYS
	mov	r7, #0xf0
	str	r7, [r1]
	ldr	r1, =GIC_ENABLE_CPU1_PHYS
	ldr	r4, [r1]				/* read cpu ctrl state in r4 */
	mov	r7, #1
	str	r7, [r1]				/* force cpu ctrl on */
	dsb						/* ensure cpu ctrl is enabled. */
	ldr	r1, =GIC_INTACK_CPU1_PHYS 		/* read who is waking us up */
	ldr	r7, [r1]
	cmp	r7, #0x3FC				/* check if a bad value was read */
	bgt 	skip_writing_to_eoi			/* and if so dont write to eoi */
	ldr	r1, =GIC_EOI_CPU1_PHYS			/* write to EOI and shut that interrupt up */
	str	r7, [r1]
skip_writing_to_eoi:
	ldr	r1, =CPU1_READ_ACK_PHYS 		/* store the intack value in memory for debug*/
	str	r7, [r1]
	ldr	r1, =GIC_ENABLE_CPU1_PHYS
	str	r4, [r1]				/* store cpu ctrl state from r4 */
	dsb						/* ensure data are written. */
.endm

.macro inc_pen_count
	ldr	r1, =MSM_PEN_PHYS
	ldr	r7, [r1]
	add	r7, r7, #1
	str	r7, [r1]
	dsb						/* ensure data are written. */
.endm

ENTRY(msm_secondary_startup)
	inc_power_collapse_exit_count
	mrc	p15, 0, r0, c0, c0, 5 	@ MPIDR
	and	r0, r0, #15		@ What CPU am I
	adr	r4, 1f			@ address of
	ldmia	r4, {r5, r6}		@ load curr addr and pen_rel addr
	sub	r4, r4, r5		@ determine virtual/phys offsets
	add	r6, r6, r4		@ apply
pen:
	wfe
	dsb				@ ensure subsequent access is
					@ after event

	ldr	r7, [r6]		@ pen_rel has cpu to remove from reset
	cmp	r7, r0			@ are we lucky?
	beq	start_this_cpu
	inc_pen_count
	get_wakeup_interrupt
	quiet_wakeup_interrupt
	b	pen

start_this_cpu:
	/*
	 * we've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	get_wakeup_interrupt
	mark_wakeup_interrupt_valid
	mvn	r7, #0			@ -1 to registers
	str r7,[r6]			@ back to the pen for ack
	b	secondary_startup

1:	.long	.
	.long	pen_release
