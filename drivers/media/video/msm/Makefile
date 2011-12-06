obj-$(CONFIG_MSM_CAMERA_FLASH) += flash.o

obj-$(CONFIG_MT9T013) += mt9t013.o mt9t013_reg.o
obj-$(CONFIG_MT9D112) += mt9d112.o mt9d112_reg.o
obj-$(CONFIG_OV9665) += ov9665.o ov9665_reg.o
obj-$(CONFIG_MT9P012) += mt9p012_fox.o mt9p012_reg.o
obj-$(CONFIG_MSM_CAMERA_LEGACY) += msm_camera.o
#msm_v4l2.o
obj-$(CONFIG_MSM_CAMERA_7X30) += msm_camera-7x30.o msm_vfe31.o msm_io_vfe31.o msm_axi_qos.o msm_vpe1.o
ifdef CONFIG_CAMERA_ZSL
obj-$(CONFIG_MSM_CAMERA_8X60) += msm_camera-8x60_ZSL.o msm_io_8x60_ZSL.o msm_vfe_8x60_ZSL.o msm_axi_qos.o msm_vpe1_8x60_ZSL.o
else
obj-$(CONFIG_MSM_CAMERA_8X60) += msm_camera-8x60.o msm_io_8x60.o msm_vfe_8x60.o msm_axi_qos.o msm_vpe1_8x60.o
ifdef CONFIG_SP3D
obj-$(CONFIG_MSM_CAMERA_8X60) += msm_camera.o msm_vfe31.o msm_vpe1.o
endif
endif
obj-$(CONFIG_S5K3E2FX) += s5k3e2fx.o
ifdef CONFIG_ARCH_MSM8X60
obj-$(CONFIG_S5K4E1GX) += s5k4e1gx_8x60.o s5k4e1gx_reg_8x60.o
else
obj-$(CONFIG_S5K4E1GX) += s5k4e1gx.o s5k4e1gx_reg.o
endif
ifdef CONFIG_MACH_BLISS
obj-$(CONFIG_S5K4E5YX) += s5k4e5yx.o s5k4e5yx_reg_bls.o
else
ifdef CONFIG_MACH_BLISSC
obj-$(CONFIG_S5K4E5YX) += s5k4e5yx.o s5k4e5yx_reg_bls.o
else
obj-$(CONFIG_S5K4E5YX) += s5k4e5yx.o s5k4e5yx_reg.o
endif
endif
ifdef CONFIG_S5K6AAFX
obj-$(CONFIG_MACH_MECHA) +=  s5k6aafx.o s5k6aafx_reg_mecha.o
obj-$(CONFIG_MACH_FLYER) += s5k6aafx.o s5k6aafx_reg_flyer.o
obj-$(CONFIG_MACH_VIVO) += s5k6aafx.o s5k6aafx_reg_vivo.o
obj-$(CONFIG_MACH_VIVOW) += s5k6aafx.o s5k6aafx_reg_vivo.o
obj-$(CONFIG_MACH_EXPRESS) += s5k6aafx.o s5k6aafx_reg_flyer.o
obj-$(CONFIG_MACH_EXPRESS_KT) += s5k6aafx.o s5k6aafx_reg_flyer.o
obj-$(CONFIG_MACH_SUPERSONIC) += s5k6aafx_suc.o s5k6aafx_reg_suc.o
obj-$(CONFIG_MACH_SHOOTER) += s5k6aafx.o s5k6aafx_reg_shooter.o s5k6aafx_reg_shooter_cob.o
obj-$(CONFIG_MACH_SHOOTER_U) += s5k6aafx.o s5k6aafx_reg_shooter.o s5k6aafx_reg_shooter_cob.o
obj-$(CONFIG_MACH_SHOOTER_CT) += s5k6aafx.o s5k6aafx_reg_shooter.o s5k6aafx_reg_shooter_cob.o
obj-$(CONFIG_MACH_VERDI) += s5k6aafx.o s5k6aafx_reg_verdi.o
obj-$(CONFIG_MACH_VERDI_LTE) += s5k6aafx.o s5k6aafx_reg_verdi.o s5k6aafx_reg_verdi_cob.o
obj-$(CONFIG_MACH_RIDER) += s5k6aafx.o s5k6aafx_reg_rider.o s5k6aafx_reg_rider_cob.o
obj-$(CONFIG_MACH_KINGDOM) += s5k6aafx.o s5k6aafx_reg_kingdom.o s5k6aafx_reg_kingdom_cob.o
obj-$(CONFIG_MACH_HOLIDAY) += s5k6aafx.o s5k6aafx_reg_holiday.o s5k6aafx_reg_holiday_cob.o
obj-$(CONFIG_MACH_RUNNYMEDE) += s5k6aafx.o s5k6aafx_reg_runnymede_cob.o
endif
obj-$(CONFIG_ARCH_MSM_ARM11) += msm_vfe7x.o msm_io7x.o
obj-$(CONFIG_ARCH_QSD8X50) += msm_vfe8x.o msm_vfe8x_proc.o msm_io8x.o
obj-$(CONFIG_VB6801) += vb6801.o
obj-$(CONFIG_OV8810) += ov8810.o
obj-$(CONFIG_OV8830) += ov8830.o ov8830_reg.o
obj-$(CONFIG_IMX105) += imx105.o imx105_reg.o
obj-$(CONFIG_SP3D) += sp3d.o sp3d_reg_3d.o sp3d_reg_2d.o sp3d_reg_com.o sp3d_reg_cal.o
obj-$(CONFIG_QS_S5K4E1) += qs_s5k4e1.o qs_s5k4e1_reg.o
obj-$(CONFIG_MT9D015) += mt9d015.o mt9d015_reg.o
ifdef CONFIG_MACH_SUPERSONIC
obj-$(CONFIG_S5K3H1GX) += s5k3h1gx_suc.o s5k3h1gx_reg_suc.o
else
ifdef CONFIG_MACH_PYRAMID
obj-$(CONFIG_S5K3H1GX) += s5k3h1gx.o s5k3h1gx_reg_pyd.o
else
obj-$(CONFIG_S5K3H1GX) += s5k3h1gx.o s5k3h1gx_reg.o
endif
endif
ifdef CONFIG_MACH_DOUBLESHOT
obj-$(CONFIG_S5K3H2YX) += s5k3h2yx.o s5k3h2yx_reg_dot.o
else
ifdef CONFIG_MACH_HOLIDAY
obj-$(CONFIG_S5K3H2YX) += s5k3h2yx.o s5k3h2yx_reg_holiday.o
else
ifdef CONFIG_MACH_RUBY
obj-$(CONFIG_S5K3H2YX) += s5k3h2yx.o s5k3h2yx_reg_holiday.o
else
ifdef CONFIG_MACH_RUNNYMEDE
obj-$(CONFIG_S5K3H2YX) += s5k3h2yx.o s5k3h2yx_reg_runnymede.o
else
ifdef CONFIG_MACH_VERDI_LTE
ifdef CONFIG_CAMERA_ZSL
obj-$(CONFIG_S5K3H2YX) += s5k3h2yx.o s5k3h2yx_reg.o
else
obj-$(CONFIG_S5K3H2YX) += s5k3h2yx_pui.o  s5k3h2yx_reg.o
endif
else
obj-$(CONFIG_S5K3H2YX) += s5k3h2yx.o s5k3h2yx_reg.o
endif
endif
endif
endif
endif
ifdef CONFIG_MACH_PYRAMID
obj-$(CONFIG_MT9V113) += mt9v113.o mt9v113_reg.o
else
ifdef CONFIG_MACH_SAGA
obj-$(CONFIG_MT9V113) += mt9v113.o mt9v113_reg_lens_9251.o
else
ifdef CONFIG_MACH_DOUBLESHOT
obj-$(CONFIG_MT9V113) += mt9v113.o mt9v113_reg_dot.o
else
obj-$(CONFIG_MT9V113) += mt9v113.o mt9v113_reg_lens_9218.o
endif
endif
endif
obj-$(CONFIG_MSM_GEMINI) += msm_gemini_dev.o msm_gemini_sync.o msm_gemini_core.o msm_gemini_hw.o msm_gemini_platform.o
