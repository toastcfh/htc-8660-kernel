/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __KGSL_MMU_H
#define __KGSL_MMU_H

/* Identifier for the global page table */
/* Per process page tables will probably pass in the thread group
   as an identifier */

#define KGSL_MMU_GLOBAL_PT 0

#define GSL_PT_SUPER_PTE 8
#define GSL_PT_PAGE_WV		0x00000001
#define GSL_PT_PAGE_RV		0x00000002
#define GSL_PT_PAGE_DIRTY	0x00000004

/* MMU registers - the register locations for all cores are the
   same.  The method for getting to those locations differs between
   2D and 3D, but the 2D and 3D register functions do that magic
   for us */

#define MH_MMU_CONFIG                0x0040
#define MH_MMU_VA_RANGE              0x0041
#define MH_MMU_PT_BASE               0x0042
#define MH_MMU_PAGE_FAULT            0x0043
#define MH_MMU_TRAN_ERROR            0x0044
#define MH_MMU_INVALIDATE            0x0045
#define MH_MMU_MPU_BASE              0x0046
#define MH_MMU_MPU_END               0x0047

#define MH_INTERRUPT_MASK            0x0A42
#define MH_INTERRUPT_STATUS          0x0A43
#define MH_INTERRUPT_CLEAR           0x0A44
#define MH_AXI_ERROR                 0x0A45

/* MH_MMU_CONFIG bit definitions */

#define MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT           0x00000004
#define MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT           0x00000006
#define MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT          0x00000008
#define MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT          0x0000000a
#define MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT          0x0000000c
#define MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT          0x0000000e
#define MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT          0x00000010
#define MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT         0x00000012
#define MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT         0x00000014
#define MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT           0x00000016
#define MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT           0x00000018

/* MMU Flags */
#define KGSL_MMUFLAGS_TLBFLUSH         0x10000000
#define KGSL_MMUFLAGS_PTUPDATE         0x20000000

#define MH_INTERRUPT_MASK__AXI_READ_ERROR                  0x00000001L
#define MH_INTERRUPT_MASK__AXI_WRITE_ERROR                 0x00000002L
#define MH_INTERRUPT_MASK__MMU_PAGE_FAULT                  0x00000004L

#ifdef CONFIG_MSM_KGSL_MMU
#define KGSL_MMU_INT_MASK \
	(MH_INTERRUPT_MASK__AXI_READ_ERROR | \
	 MH_INTERRUPT_MASK__AXI_WRITE_ERROR | \
	 MH_INTERRUPT_MASK__MMU_PAGE_FAULT)
#else
#define KGSL_MMU_INT_MASK \
	(MH_INTERRUPT_MASK__AXI_READ_ERROR | \
	 MH_INTERRUPT_MASK__AXI_WRITE_ERROR)
#endif

/* Macros to manage TLB flushing */
#define GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS     (sizeof(unsigned char) * 8)
#define GSL_TLBFLUSH_FILTER_GET(superpte)			     \
	      (*((unsigned char *)				    \
	      (((unsigned int)pagetable->tlbflushfilter.base)    \
	      + (superpte / GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS))))
#define GSL_TLBFLUSH_FILTER_SETDIRTY(superpte)				\
	      (GSL_TLBFLUSH_FILTER_GET((superpte)) |= 1 <<	    \
	      (superpte % GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS))
#define GSL_TLBFLUSH_FILTER_ISDIRTY(superpte)			 \
	      (GSL_TLBFLUSH_FILTER_GET((superpte)) &		  \
	      (1 << (superpte % GSL_TLBFLUSH_FILTER_ENTRY_NUMBITS)))
#define GSL_TLBFLUSH_FILTER_RESET() memset(pagetable->tlbflushfilter.base,\
				      0, pagetable->tlbflushfilter.size)


struct kgsl_device;

struct kgsl_tlbflushfilter {
	unsigned int *base;
	unsigned int size;
};

struct kgsl_pagetable {
	spinlock_t lock;
	struct kref refcount;
	struct kgsl_memdesc  base;
	uint32_t      va_base;
	unsigned int   va_range;
	unsigned int   last_superpte;
	unsigned int   max_entries;
	struct gen_pool *pool;
	struct list_head list;
	unsigned int name;
	/* Maintain filter to manage tlb flushing */
	struct kgsl_tlbflushfilter tlbflushfilter;
	unsigned int tlb_flags;
	struct kobject *kobj;

	struct {
		unsigned int entries;
		unsigned int mapped;
		unsigned int max_mapped;
		unsigned int max_entries;
	} stats;
};

struct kgsl_mmu {
	unsigned int     refcnt;
	uint32_t      flags;
	struct kgsl_device     *device;
	unsigned int     config;
	uint32_t        mpu_base;
	int              mpu_range;
	struct kgsl_memdesc    dummyspace;
	/* current page table object being used by device mmu */
	struct kgsl_pagetable  *defaultpagetable;
	struct kgsl_pagetable  *hwpagetable;
};

struct kgsl_ptpool_chunk {
	size_t size;
	unsigned int count;
	int dynamic;

	void *data;
	unsigned int phys;

	unsigned long *bitmap;
	struct list_head list;
};

struct kgsl_pagetable *kgsl_mmu_getpagetable(unsigned long name);

#ifdef CONFIG_MSM_KGSL_MMU

int kgsl_mmu_init(struct kgsl_device *device);
int kgsl_mmu_start(struct kgsl_device *device);
int kgsl_mmu_stop(struct kgsl_device *device);
int kgsl_mmu_close(struct kgsl_device *device);
void kgsl_mmu_setstate(struct kgsl_device *device,
		      struct kgsl_pagetable *pagetable);
int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
		 struct kgsl_memdesc *memdesc,
		 unsigned int protflags);
int kgsl_mmu_map_global(struct kgsl_pagetable *pagetable,
			struct kgsl_memdesc *memdesc, unsigned int protflags);
int kgsl_mmu_unmap(struct kgsl_pagetable *pagetable,
		    struct kgsl_memdesc *memdesc);
void kgsl_ptpool_destroy(struct kgsl_ptpool *pool);
int kgsl_ptpool_init(struct kgsl_ptpool *pool, int ptsize, int entries);
void kgsl_mh_intrcallback(struct kgsl_device *device);
void kgsl_mmu_putpagetable(struct kgsl_pagetable *pagetable);
unsigned int kgsl_virtaddr_to_physaddr(void *virtaddr);
void kgsl_setstate(struct kgsl_device *device, uint32_t flags);
void kgsl_default_setstate(struct kgsl_device *device, uint32_t flags);

static inline int kgsl_mmu_enabled(void)
{
	return 1;
}

#else

static inline int kgsl_mmu_enabled(void)
{
	return 0;
}

static inline int kgsl_mmu_init(struct kgsl_device *device)
{
	return 0;
}

static inline int kgsl_mmu_start(struct kgsl_device *device)
{
	return 0;
}

static inline int kgsl_mmu_stop(struct kgsl_device *device)
{
	return 0;
}

static inline int kgsl_mmu_close(struct kgsl_device *device)
{
	return 0;
}

static inline void kgsl_mmu_setstate(struct kgsl_device *device,
				    struct kgsl_pagetable *pagetable) { }

static inline int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
		 struct kgsl_memdesc *memdesc,
		 unsigned int protflags)
{
	memdesc->gpuaddr = memdesc->physaddr;
	return 0;
}

static inline int kgsl_mmu_unmap(struct kgsl_pagetable *pagetable,
				 struct kgsl_memdesc *memdesc)
{
	return 0;
}

static inline int kgsl_ptpool_init(struct kgsl_ptpool *pool, int ptsize,
				    int entries)
{
	return 0;
}

static inline int kgsl_mmu_map_global(struct kgsl_pagetable *pagetable,
	struct kgsl_memdesc *memdesc, unsigned int protflags)
{
	memdesc->gpuaddr = memdesc->physaddr;
	return 0;
}

static inline void kgsl_ptpool_destroy(struct kgsl_ptpool *pool) { }

static inline void kgsl_mh_intrcallback(struct kgsl_device *device) { }

static inline void kgsl_mmu_putpagetable(struct kgsl_pagetable *pagetable) { }

static inline unsigned int kgsl_virtaddr_to_physaddr(void *virtaddr)
{
	return 0;
}

static inline void kgsl_setstate(struct kgsl_device *device, uint32_t flags)
{ }

static inline void kgsl_default_setstate(struct kgsl_device *device,
	uint32_t flags) { }
#endif

static inline unsigned int kgsl_pt_get_flags(struct kgsl_pagetable *pt,
					     enum kgsl_deviceid id)
{
	unsigned int result = 0;

	if (pt == NULL)
		return 0;

	spin_lock(&pt->lock);
	if (pt->tlb_flags && (1<<id)) {
		result = KGSL_MMUFLAGS_TLBFLUSH;
		pt->tlb_flags &= ~(1<<id);
	}
	spin_unlock(&pt->lock);
	return result;
}

#endif /* __KGSL_MMU_H */
