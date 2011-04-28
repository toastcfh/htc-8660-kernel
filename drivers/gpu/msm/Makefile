ccflags-y := -Iinclude/drm

msm_kgsl-y = \
	kgsl.o \
	kgsl_sharedmem.o \
	kgsl_pwrctrl.o \
	kgsl_postmortem.o \
	kgsl_ringbuffer.o \
	kgsl_drawctxt.o \
	kgsl_cmdstream.o \
	kgsl_yamato.o \
	kgsl_g12_drawctxt.o \
	kgsl_g12_cmdstream.o \
	kgsl_g12.o

msm_kgsl-$(CONFIG_DEBUG_FS) += kgsl_debugfs.o
msm_kgsl-$(CONFIG_DEBUG_FS) += kgsl_yamato_debugfs.o

msm_kgsl-$(CONFIG_MSM_KGSL_MMU) += kgsl_mmu.o
msm_kgsl-$(CONFIG_MSM_KGSL_CFF_DUMP) += kgsl_cffdump.o
msm_kgsl-$(CONFIG_MSM_KGSL_DRM) += kgsl_drm.o

obj-$(CONFIG_GPU_MSM_KGSL) += msm_kgsl.o
