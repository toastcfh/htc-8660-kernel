/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */


#include <linux/delay.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera_sensor.h>
#include <mach/gpio.h>
#include <mach/camera-8x60.h>
#include <linux/smp_lock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <asm/uaccess.h>
#include "sp3d.h"
#include <mach/msm_flashlight.h>

#define SP3D_DEBUG_MODE 0
/* SP3D module operation mode */
#define SP3D_REGADDR_WR 0x70
#define SP3D_REGVAL_WR	0x72
#define SP3D_REGVAL_RD	0x73
#define	Q8						0x100
#define	Q10						0x400

/* SENSOR REGISTER DEFINES */

/* Full	Size */
/* Quarter Size	*/
/* Blanking as measured	on the scope */
/* Full	Size */
/* Quarter Size	*/

static struct platform_device *sp3d_pdev;
static int dmode = CAMERA_3D_MODE;
static atomic_t start_counting;
static atomic_t snapshot_flag;
static atomic_t preview_flag;


struct sp3d_spi_ctrl_blk {
	struct spi_device *spi;
	spinlock_t		spinlock;
};

struct sp3d_spi_ctrl_blk *sp3d_spi_ctrl;


struct sp3d_spi_ctrl {
	const struct msm_camera_sensor_info *sensordata;
	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;
	int16_t  curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;
	enum sp3d_resolution_t prev_res;
	enum sp3d_resolution_t pict_res;
	enum sp3d_resolution_t curr_res;
	enum sp3d_test_mode_t set_test;
	unsigned short imgaddr;
};

static uint8_t sp3d_delay_msecs_stdby = 20;
static int32_t config_csi;

static struct sp3d_spi_ctrl *sp3d_ctrl;
static uint8_t flash_state;
static uint16_t exp_pre, exp_off, iris_pre, iris_off, flashlight_exp_div;
#define SP3D_REG_FLAHS_DIV_1 7
#define SP3D_REG_FLAHS_DIV_2 22
#define SP3D_FLASHLIGHT_BASE 0x0500
#define SP3D_MAX_ALLOCATE 1000

static uint8_t *sp3d_spi_write_addr;

enum sp3d_state_e{
	SP3D_SNAPSHOT_MODE = 0,
	SP3D_PREVIEW_MODE,
};
static enum sp3d_state_e sp3d_state;

struct sp3d_int_t {
	spinlock_t sp3d_spin_lock;
	wait_queue_head_t sp3d_wait;
	atomic_t frame_count;
};

static struct sp3d_int_t sp3d_int;

enum sp3d_snapshot_state_e{
	SP3D_CONTINUE_SHUTTER = 0,
	SP3D_SHUTTER_ON,
	SP3D_FIRST_INT,
	SP3D_L_START_CMD,
	SP3D_L_LP11,
	SP3D_L_FRAME,
	SP3D_DUMMY_LP11,
	SP3D_R_START_CMD,
	SP3D_R_LP11,
	SP3D_R_FRAME,
	SP3D_R_DUMMY_LP11,
	SP3D_SNAPSHOT_NOT_START,
};

enum sp3d_2Dsnapshot_state_e{
	SP3D_2DCONTINUE_SHUTTER = 0,
	SP3D_2DSHUTTER_ON,
	SP3D_2DFIRST_INT,
	SP3D_2DSTART_CMD,
	SP3D_2DLP11,
	SP3D_2DFRAME,
	SP3D_2DDUMMY_LP11_2,
	SP3D_2DDUMMY_FRAME_2,
	SP3D_2DDUMMY_LP11_3,
	SP3D_2DDUMMY_FRAME_3,
	SP3D_2DDUMMY_FINISH,
	SP3D_2DSNAPSHOT_NOT_START,

};

enum sp3d_command_e{
	SP3D_SET_AUTO_FOCUS = 0,
	SP3D_SET_CANCEL_FOCUS,
	SP3D_EFFECT,
	SP3D_AF_MODE,
	SP3D_ISO,
	SP3D_WB,
	SP3D_EV,
	SP3D_ANTIBANDING,
	SP3D_CONTRAST,
	SP3D_SHARPNESS,
	SP3D_SATURATION,
	SP3D_AF_AREA,
	SP3D_SET_SHUTTER_ON,
};

static atomic_t sp3d_snapshot_state;
static atomic_t sp3d_2Dsnapshot_state;


DEFINE_MUTEX(sp3d_mut);
DEFINE_MUTEX(sp3d_set);

#define SP3D_CHECK_SPI(a) ({\
	if(a < 0) \
		pr_err("%s rc:%d line:%d",__func__, a, __LINE__); \
	a; \
})

static int sp3d_spi_sync_write_then_read(uint8_t *txbuf, size_t n_tx,
	uint8_t *rxbuf, size_t n_rx)
{
	struct spi_transfer	rx_t;
	struct spi_message	m;
	memset(&rx_t, 0, sizeof(struct spi_transfer));
	rx_t.tx_buf = txbuf,
	rx_t.rx_buf = rxbuf,
	rx_t.len = n_rx,
	spi_message_init(&m);
	spi_message_add_tail(&rx_t, &m);
	return spi_sync(sp3d_spi_ctrl->spi, &m); // USE spi_sync function for now.
}


static int sp3d_spi_single_write(uint8_t *wbuf, int wlen)
{
	struct spi_transfer t;
	struct spi_message m;

	memset(&t, 0, sizeof(struct spi_transfer));
	t.tx_buf = wbuf;
	t.len = wlen;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(sp3d_spi_ctrl->spi, &m);
}

static int sp3d_spi_read(uint16_t reg, uint16_t *rval)
{
	int rc = 0;
	uint8_t tx[3], rx[6];

	tx[0] = SP3D_REGADDR_WR;
	tx[1] = (reg & 0xff00) >> 8;
	tx[2] = reg & 0x00ff;

	rc = sp3d_spi_single_write(&tx[0], 3);

	rx[0] = SP3D_REGVAL_RD;
	rx[1] = SP3D_REGVAL_RD;
	rx[2] = SP3D_REGVAL_RD;
	rx[3] = 0;
	rx[4] = 0;
	rx[5] = 0;

	rc = sp3d_spi_sync_write_then_read(&rx[0], 3,
		&rx[3], 3);
#if 0
	pr_info("%s: rc = %d\n", __func__, rc);
	pr_info("rx[3] = 0x%x, rx[4] = 0x%x, rx[5] = 0x%x\n",
		rx[3], rx[4], rx[5]);
#endif
	if (rc >= 0)
		*rval = rx[4] << 8 | rx[5];
	else 
		*rval = 0;

	return rc;
}

static void sp3d_spi_complete(void *arg)
{
	complete(arg);
}

static int sp3d_spi_transaction(struct spi_message *msg)
{
	DECLARE_COMPLETION_ONSTACK(sp3d_done);
	int status;
	unsigned int actual_length;
	actual_length = msg->actual_length;
	msg->complete = sp3d_spi_complete;
	msg->context = &sp3d_done;

	spin_lock_irq(&sp3d_spi_ctrl->spinlock);
	if (sp3d_spi_ctrl->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(sp3d_spi_ctrl->spi, msg);
	spin_unlock_irq(&sp3d_spi_ctrl->spinlock);

	if (status == 0) {
		wait_for_completion(&sp3d_done);
		status = msg->status;
		if (status == 0)
			status = actual_length;
	}

	return status;
}

static int sp3d_spi_sync_write_once(uint8_t *tbuf, uint8_t *wbuf)
{
	struct spi_message	m;
	struct spi_transfer tx_addr;
	struct spi_transfer tx_buf;
	memset(&tx_addr, 0, sizeof(struct spi_transfer));
	memset(&tx_buf, 0, sizeof(struct spi_transfer));
	tx_addr.tx_buf	= tbuf;
	tx_addr.len = 3;
	tx_buf.tx_buf = wbuf;
	tx_buf.len = 3;
	spi_message_init(&m);
	spi_message_add_tail(&tx_addr, &m);
	spi_message_add_tail(&tx_buf, &m);
	return sp3d_spi_transaction(&m);
}

static int sp3d_spi_write(uint16_t reg, uint16_t val)
{
	uint8_t tx[3], wb[3];

	tx[0] = SP3D_REGADDR_WR;
	tx[1] = (reg & 0xff00) >> 8;
	tx[2] = reg & 0x00ff;

	wb[0] = SP3D_REGVAL_WR;
	wb[1] = (val & 0xff00) >> 8;
	wb[2] = val & 0x00ff;

    return sp3d_spi_sync_write_once(&tx[0], &wb[0]);
}

static int32_t sp3d_spi_write_table2(struct sp3d_reg_conf const
					 *reg_conf_tbl, int orig_num)
{
	int i;
	for(i = 0;i<orig_num;i++)
		sp3d_spi_write(reg_conf_tbl[i].waddr, reg_conf_tbl[i].wdata);
	return true;
}



static int32_t sp3d_spi_write_table(struct sp3d_reg_conf const
					 *reg_conf_tbl, int orig_num)
{
    int i;
    int num,add_num;
    struct spi_message	m;
    struct spi_transfer	tx_addr;
    static int last_write;
    last_write = 0;
    if(orig_num > SP3D_MAX_ALLOCATE){
		add_num=num = SP3D_MAX_ALLOCATE;
    }else{
		last_write = 1;
		num = orig_num;
    }

 SP3D_KEEPWRITE:

    if (!sp3d_spi_write_addr) {
		pr_err("Error allocating memory retrying num:%d\n",orig_num);
		return -1;
    }
    spi_message_init(&m);
    memset(&tx_addr, 0, sizeof(struct spi_transfer));
    for (i = 0; i < num; i++) {
        sp3d_spi_write_addr[i*8] = SP3D_REGADDR_WR;
        sp3d_spi_write_addr[i*8+1] = (reg_conf_tbl->waddr & 0xff00) >> 8;
        sp3d_spi_write_addr[i*8+2] = reg_conf_tbl->waddr & 0x00ff;
        sp3d_spi_write_addr[i*8+3] = 0;
        
	 sp3d_spi_write_addr[i*8+4] = SP3D_REGVAL_WR;
        sp3d_spi_write_addr[i*8+5] = (reg_conf_tbl->wdata & 0xff00) >> 8;
        sp3d_spi_write_addr[i*8+6] = reg_conf_tbl->wdata & 0x00ff;
        sp3d_spi_write_addr[i*8+7] = 0;
        reg_conf_tbl++;
    }

	tx_addr.tx_buf = sp3d_spi_write_addr;
	tx_addr.len = num*8;
	tx_addr.cs_change = 1;
	tx_addr.bits_per_word = 32;
	spi_message_add_tail(&tx_addr, &m);
	sp3d_spi_transaction(&m);
	if(last_write)
		return 0;
	if(orig_num > (add_num+SP3D_MAX_ALLOCATE)){
		num = SP3D_MAX_ALLOCATE;
		add_num = add_num + SP3D_MAX_ALLOCATE;
	}else{
		num = orig_num - add_num;
		last_write = 1;
	}
	goto SP3D_KEEPWRITE;
}


static int sp3d_vreg_enable(const struct msm_camera_sensor_info *sdata)
{
  int rc = 0;
  pr_info("[CAM]%s camera vreg on\n", __func__);

  if (sdata->camera_power_on == NULL) {
    pr_err("[CAM]sensor platform_data didnt register\n");
    return -EIO;
  }
  rc = sdata->camera_power_on();
  return rc;
}


static int sp3d_vreg_disable(const struct msm_camera_sensor_info *data)
{
  int rc = 0;
  pr_info("[CAM]%s camera vreg off\n", __func__);
  data->pdata->camera_gpio_off();
  mdelay(10);
  if (data->camera_power_off == NULL) {
    pr_err("[CAM]sensor platform_data didnt register\n");
    return -EIO;
  }
  rc = data->camera_power_off();
  return rc;
}

static void sp3d_check_reg_status(void){
	uint16_t rval;
	int count = 0;
	while(1){
		count ++;
		usleep(10*1000);
		sp3d_spi_write(0x0492,0x33F8);
		sp3d_spi_write(0x049A,0x020F);
		sp3d_spi_write(0x0490,0x0402);
		sp3d_spi_write(0x049A,0x00FC);
		sp3d_spi_write(0x0490,0x0402);
		sp3d_spi_write(0x049A,0x0024);
		sp3d_spi_write(0x0490,0x0006);
		sp3d_spi_read(0x04A2, &rval);
		if(rval == 0x80){/*MIPI enabled*/
			pr_info("[CAM]%s got MIPI signal from sharp",__func__);
			break;
		}
		if(count > 200){
			pr_info("[CAM]%s check reg time out",__func__);
			break;
		}
	}
}

static int sp3d_wait_2Dstate(enum sp3d_2Dsnapshot_state_e state){
	wait_event_timeout(sp3d_int.sp3d_wait,
		atomic_read(&sp3d_2Dsnapshot_state) == state,
		msecs_to_jiffies(1000));
	smp_mb();
	pr_info("[CAM]sp3d_wait_state wait done sp3d_2Dsnapshot_state:%d",atomic_read(&sp3d_2Dsnapshot_state));
	return 0;
}


static int sp3d_wait_state(enum sp3d_snapshot_state_e state){
	wait_event_timeout(sp3d_int.sp3d_wait,
		atomic_read(&sp3d_snapshot_state) == state,
		msecs_to_jiffies(1000));
	smp_mb();
	pr_info("[CAM]sp3d_wait_state wait done sp3d_snapshot_state:%d",atomic_read(&sp3d_snapshot_state));
	return 0;
}


static int sp3d_wait_INT(int frame){
	atomic_set(&start_counting, true);
	//smp_mb();
	wait_event_timeout(sp3d_int.sp3d_wait,
		atomic_read(&sp3d_int.frame_count) == frame,
		msecs_to_jiffies(1000));
	//smp_mb();
	//pr_info("sp3d_wait_INT wait done sp3d_int.frame_count:%d",atomic_read(&sp3d_int.frame_count));
	atomic_set(&sp3d_int.frame_count,0);
	atomic_set(&start_counting, false);
	//smp_mb();
	return 0;
}


static int sp3d_video_setting(void){
	int rc = 0;
	pr_info("[CAM]sp3d_video_setting");
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_koj_2d_720p_video,
                               sp3d_2D_regs.reg_koj_2d_720p_video_size);
	/*waitint for state transfer*/
	sp3d_wait_INT(4);
	return rc;
}


static int sp3d_preview_setting(void){
	int rc = 0;
	
	pr_info("[CAM]sp3d_preview_setting");
	if ( dmode == CAMERA_2D_MODE ) {
		rc = sp3d_spi_write_table(sp3d_2D_regs.reg_koj_2d_preview,
                               sp3d_2D_regs.reg_koj_2d_preview_size);
	}else{
		rc = sp3d_spi_write_table(sp3d_3D_regs.reg_koj_3d_preview,
                               sp3d_3D_regs.reg_koj_3d_preview_size);
	}
	/*waitint for state transfer*/
	sp3d_wait_INT(4);
	if(rc < 0){
		return rc;
	}
	return 0;
}

static void sp3d_read_shutter_status(void){
	int rc = 0;
	uint16_t val;
	/*write*/
	sp3d_spi_write(0x0492,0x33F8);
	sp3d_spi_write(0x049A,0x020F);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x00fc);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0024);
	sp3d_spi_write(0x0490,0x0006);
	/*read*/
	rc = sp3d_spi_read(0x04A2, &val);
	pr_info("[CAM]shutter on status val:0x%x",val);
	sp3d_spi_write(0x0490,0x0006);
}


static void sp3d_config_csi_param(struct msm_camera_csi_params *sp3d_csi_params) {
	sp3d_csi_params->lane_cnt = 2;
	sp3d_csi_params->data_format = CSI_8BIT;
	sp3d_csi_params->lane_assign = 0xe4;
	sp3d_csi_params->dpcm_scheme = 0;
	if(dmode == CAMERA_3D_MODE)
		sp3d_csi_params->settle_cnt = 25;
	else
		sp3d_csi_params->settle_cnt = 23;
	sp3d_csi_params->mipi_driving_strength = 1;
	sp3d_csi_params->hs_impedence = 0x0F;
}


static inline int rt_policy(int policy)
{
	if (unlikely(policy == SCHED_FIFO) || unlikely(policy == SCHED_RR))
		return 1;
	return 0;
}

static inline int task_has_rt_policy(struct task_struct *p)
{
	return rt_policy(p->policy);
}

static int sp3d_3D_get_L_frame(void){
	int rc = 0;
	pr_info("[CAM]sp3d 3D shutter mixer setting on full resolution master");
	rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_3d_shutter_full_m,
		sp3d_snapshot_regs.reg_koj_3d_shutter_full_m_size);
	if(rc < 0)
		return rc;
	sp3d_read_shutter_status();
	sp3d_wait_state(SP3D_L_LP11);/*LP11*/
	msm_camio_csi_enable_lp_rec();
	return 0;
}




static int sp3d_3D_get_R_frame(void){
	int rc = 0;
	struct msm_camera_csi_params sp3d_csi_params;
	struct sched_param s = { .sched_priority = 1 };
	int old_prio = current->rt_priority;
	int old_policy = current->policy;
	int cap_nice;
	/* raise up scheduling policy and priority */
	cap_nice = cap_raised(current_cap(), CAP_SYS_NICE);
	/* just for this write, set us real-time */
	if (!task_has_rt_policy(current)) {
		struct cred *new = prepare_creds();
		if (new != NULL) {
		    cap_raise(new->cap_effective, CAP_SYS_NICE);
		    commit_creds(new);
		    sched_setscheduler(current, SCHED_FIFO, &s);
		}
	}
	sp3d_wait_state(SP3D_R_START_CMD);

#if SP3D_DEBUG_MODE
	//test debug
	msm_camio_csi_misr_read();
#endif
	/*LP00*/
	msm_camio_csi_disable_lp_rec();
	/*disable and reset MIPI*/
	msm_camio_csi_core_reset();
	msm_camio_csi_core_on();
	sp3d_config_csi_param(&sp3d_csi_params);
	rc = msm_camio_csi_config_withReceiverDisabled(&sp3d_csi_params);

	pr_info("[CAM]sp3d 3D shutter mixer setting on full resolution slave");
#if 0
	if(flash_state) {
		flash_state = false;
		sdata->flash_data->flash_src->camera_flash(FL_MODE_OFF);
	}
#endif
	/*Config sensor*/
	rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_3d_shutter_full_s,
	sp3d_snapshot_regs.reg_koj_3d_shutter_full_s_size);
	if(rc < 0)
		return rc;
	/*LP11*/
	sp3d_wait_state(SP3D_R_LP11);
	/*enable receiver*/
	msm_camio_csi_enable_lp_rec();

#if SP3D_DEBUG_MODE
	for (i=0;i<50;i++)
	{
		msm_camio_csi_misr_read();
		usleep(100);
	}
	sp3d_wait_state(SP3D_R_DUMMY_LP11);
	for (i=0;i<50;i++)
	{
		msm_camio_csi_misr_read();
		usleep(100);
	}
#endif
	/* restore scheduling policy and priority */
	if (!rt_policy(old_policy)) {
		struct sched_param v = { .sched_priority = old_prio };
		sched_setscheduler(current, old_policy, &v);
		if (likely(!cap_nice)) {
			struct cred *new = prepare_creds();
			if (new != NULL) {
			    cap_lower(new->cap_effective, CAP_SYS_NICE);
			    commit_creds(new);
			}
		}
	}
	return 0;
}

static int sp3d_stop_snapshot(void){
	struct msm_camera_sensor_info *sdata = sp3d_pdev->dev.platform_data;
	int rc = 0;
	atomic_set(&snapshot_flag,false);
	atomic_set(&preview_flag,true);
	if (dmode == CAMERA_2D_MODE) {
		if(flash_state) {
			flash_state = false;
			sdata->flash_data->flash_src->camera_flash(FL_MODE_OFF);
		}
		rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_2d_shutter_full_INT,
			sp3d_snapshot_regs.reg_koj_2d_shutter_full_INT_size);
		/*waiting for 4 frame to fix sharp issue
		when start snapshot and stop snapshot immediately*/
		sp3d_wait_INT(4);
		rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_shutter_off,
			sp3d_snapshot_regs.reg_koj_shutter_off_size);
		
		/*set sensor output size to Q-size*/
		rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_2d_shutter_stop,
			sp3d_snapshot_regs.reg_koj_2d_shutter_stop_size);
	} else {
		/*set sensor output size to 720p*/
		rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_3d_shutter_stop,
			sp3d_snapshot_regs.reg_koj_3d_shutter_stop_size);
		rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_shutter_off,
			sp3d_snapshot_regs.reg_koj_shutter_off_size);
	}
	return rc;
}

static int sp3d_check(enum sp3d_command_e command){
	uint16_t val;
	static int sp3d_check_count = 0;
SP3D_RECHECKCPU:
	sp3d_spi_write(0x0492,0x33F8);
	sp3d_spi_write(0x049A,0x020D);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x000f);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0024);
	sp3d_spi_write(0x0490,0x0006);
	sp3d_spi_read(0x04A2, &val);
	if(val == 0x01){
		sp3d_check_count++;
		if(sp3d_check_count > 20){
			pr_info("[CAM]sp3d check CPU timeout:%d val:0x%x ",command,val);
			return 0;
		}else{
			pr_info("[CAM]sp3d recheck CPU:%d val:0x%x",command,val);
			goto SP3D_RECHECKCPU;
		}
	}
	sp3d_check_count = 0;
SP3D_RECHECKVALUE:
	sp3d_spi_write(0x0492,0x33F8);
	sp3d_spi_write(0x049A,0x020F);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x00fc);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0024);
	sp3d_spi_write(0x0490,0x0006);
	sp3d_spi_read(0x04A2, &val);
	if(val == 0x8f){
		sp3d_check_count ++;
		if(sp3d_check_count > 20){
			pr_info("[CAM]sp3d recheck value timeout:%d val:0x%x",command,val);
			return 0;
		}else{
			pr_info("[CAM]sp3d recheck value:%d val:0x%x",command,val);
			goto SP3D_RECHECKVALUE;
		}
	}else if(val == 0x80){
		pr_info("[CAM]sp3d value check ACK : %d",command);
		return 0;
	}else if(val == 0x81){
		sp3d_check_count ++;
		if(sp3d_check_count>20){
			pr_info("[CAM]sp3d check NACK timeout:%d val:0x%x",command,val);
			return 0;
		}else{
			pr_info("[CAM]sp3d value check NACK :%d sp3d_check_count:%d",
				command,sp3d_check_count);
			sp3d_wait_INT(1);
			goto SP3D_RECHECKVALUE;
		}
	}else{
		pr_info("[CAM]sp3d unkonw status val:0x%x",val);
		return 1;
	}
}

static void sp3d_check_cpu_status(void){
	uint16_t val =0;
	int rc = 0;
	/*write*/
	pr_info("[CAM]---------- sp3d_check_cpu_status");
SP3D_RECHECK:
	sp3d_spi_write(0x0492,0x33F8);
	sp3d_spi_write(0x049A,0x020F);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x00fc);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0024);
	sp3d_spi_write(0x0490,0x0006);
	/*read*/
	rc = sp3d_spi_read(0x04A2, &val);
	pr_info("[CAM]sp3d check cpu status:0x%x",val);
#if 1
	/*CPU not ready*/
	if(val == 0x8f){
		pr_info("[CAM]sp3d cpu not ready, wait till cpu ready");
		goto SP3D_RECHECK;
	}
	/*AF not stable, send cancel focus*/
	if(val == 0x91 || val == 0x82 || val == 0x81){
		pr_info("[CAM]sp3d AF not ready yet, waiting for AF ready");
		rc = sp3d_spi_write_table(sp3d_com_feature_regs.reg_pat_koj_cancel_af,
                               sp3d_com_feature_regs.reg_pat_koj_cancel_af_size);
		sp3d_wait_INT(4);
	}
#endif
	return;
}

static int sp3d_start_snapshot(void){
	int rc = 0;
	struct msm_camera_sensor_info *sdata = sp3d_pdev->dev.platform_data;
	struct msm_camera_csi_params sp3d_csi_params;
	struct sched_param s = { .sched_priority = 1 };
	int old_prio = current->rt_priority;
	int old_policy = current->policy;
	int cap_nice;

	pr_info("[CAM]sp3d_snapshot_setting");
	pr_info("[CAM]KPI PA: start sensor snapshot config: %d\n", __LINE__);
	sdata->kpi_sensor_start = ktime_to_ns(ktime_get());
	atomic_set(&snapshot_flag,true);
	atomic_set(&preview_flag,false);
	sp3d_check(SP3D_SET_SHUTTER_ON);
	sp3d_check_cpu_status();
	sp3d_wait_INT(1);
	if(dmode == CAMERA_2D_MODE) {/*2D mode*/
		/* raise up scheduling policy and priority */
		cap_nice = cap_raised(current_cap(), CAP_SYS_NICE);
		/* just for this write, set us real-time */
		if (!task_has_rt_policy(current)) {
			struct cred *new = prepare_creds();
			if (new != NULL) {
		    		cap_raise(new->cap_effective, CAP_SYS_NICE);
		    		commit_creds(new);
		    		sched_setscheduler(current, SCHED_FIFO, &s);
			}
		}

		if(flash_state)
			sdata->flash_data->flash_src->camera_flash(FL_MODE_FLASH);

		rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_continue3_shutter,
                               sp3d_snapshot_regs.reg_koj_continue3_shutter_size);
		atomic_set(&sp3d_2Dsnapshot_state, SP3D_2DCONTINUE_SHUTTER);
		sp3d_wait_2Dstate(SP3D_2DSHUTTER_ON);
		pr_info("[CAM]sp3d shutter on");
		if(flash_state){
			pr_info("[CAM]sp3d flash_exp_div=0x%x\n", flashlight_exp_div);
			sp3d_snapshot_regs.reg_koj_shutter_on_evoffset[SP3D_REG_FLAHS_DIV_1].wdata
				= sp3d_snapshot_regs.reg_koj_shutter_on_evoffset[SP3D_REG_FLAHS_DIV_2].wdata
				= SP3D_FLASHLIGHT_BASE + flashlight_exp_div;
			rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_shutter_on_evoffset,
                               sp3d_snapshot_regs.reg_koj_shutter_on_evoffset_size);
		}else{
			rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_shutter_on,
                               sp3d_snapshot_regs.reg_koj_shutter_on_size);
		}
		if(rc < 0){
			if (!rt_policy(old_policy)) {
			struct sched_param v = { .sched_priority = old_prio };
			sched_setscheduler(current, old_policy, &v);
			if (likely(!cap_nice)) {
			struct cred *new = prepare_creds();
					if (new != NULL) {
			    			cap_lower(new->cap_effective, CAP_SYS_NICE);
			    			commit_creds(new);
					}
				}
			}
			return rc;
		}
		sp3d_wait_2Dstate(SP3D_2DSTART_CMD);/*after shutter on command*/
		msm_camio_csi_disable_lp_rec();
		msm_camio_csi_core_reset();
		msm_camio_csi_core_on();
		sp3d_config_csi_param(&sp3d_csi_params);
		rc = msm_camio_csi_config_withReceiverDisabled(&sp3d_csi_params);
		sp3d_read_shutter_status();
		pr_info("[CAM]sp3d 2D shutter mixer setting on full resolution");
		rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_2d_shutter_full,
                               sp3d_snapshot_regs.reg_koj_2d_shutter_full_size);
		if(rc < 0){
			if (!rt_policy(old_policy)) {
				struct sched_param v = { .sched_priority = old_prio };
				sched_setscheduler(current, old_policy, &v);
				if (likely(!cap_nice)) {
				struct cred *new = prepare_creds();
					if (new != NULL) {
			    			cap_lower(new->cap_effective, CAP_SYS_NICE);
			    			commit_creds(new);
					}
				}
			}
			return rc;
		}
		sp3d_wait_2Dstate(SP3D_2DLP11);/*LP11*/
		msm_camio_csi_enable_lp_rec();
		if (!rt_policy(old_policy)) {
			struct sched_param v = { .sched_priority = old_prio };
			sched_setscheduler(current, old_policy, &v);
			if (likely(!cap_nice)) {
			struct cred *new = prepare_creds();
				if (new != NULL) {
			    		cap_lower(new->cap_effective, CAP_SYS_NICE);
			    		commit_creds(new);
				}
			}
		}
	}else{/*3D mode*/

		if(flash_state)
			sdata->flash_data->flash_src->camera_flash(FL_MODE_FLASH);

		rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_continue1_shutter,
                               sp3d_snapshot_regs.reg_koj_continue1_shutter_size);
		atomic_set(&sp3d_snapshot_state, SP3D_CONTINUE_SHUTTER);
		smp_mb();
		sp3d_wait_state(SP3D_SHUTTER_ON);
		/*wait interrupt*/
		pr_info("[CAM]sp3d shutter on\n");
		if(flash_state){
			pr_info("[CAM]sp3d flash_exp_div=0x%x\n", flashlight_exp_div);
			sp3d_snapshot_regs.reg_koj_shutter_on_evoffset[SP3D_REG_FLAHS_DIV_1].wdata
				= sp3d_snapshot_regs.reg_koj_shutter_on_evoffset[SP3D_REG_FLAHS_DIV_2].wdata
				= SP3D_FLASHLIGHT_BASE + flashlight_exp_div;
			rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_shutter_on_evoffset,
                               sp3d_snapshot_regs.reg_koj_shutter_on_evoffset_size);
		}else{
			rc = sp3d_spi_write_table(sp3d_snapshot_regs.reg_koj_shutter_on,
                               sp3d_snapshot_regs.reg_koj_shutter_on_size);
		}
		if(rc < 0)
			return rc;
		sp3d_wait_state(SP3D_L_START_CMD);
		msm_camio_csi_disable_lp_rec();
		msm_camio_csi_core_reset();
		msm_camio_csi_core_on();
		sp3d_config_csi_param(&sp3d_csi_params);
		rc = msm_camio_csi_config_withReceiverDisabled(&sp3d_csi_params);
	}
	return 0;
}


int sp2d_init_setting_thread(void){
	
		int rc = 0;
		pr_info("[CAM]%s reg_para8_koj_stop_htc_s\n",__func__);
		#if 1 /*take off due to 2D mode dosent need this setting suggest by sharp*/
		rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para8_koj_stop_htc_s,
                               sp3d_2D_regs.reg_para8_koj_stop_htc_s_size);
		if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

		pr_info("[CAM]%s reg_para9_koj_scs_main_htc_s\n",__func__);
		rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para9_koj_scs_main_htc_s,
                               sp3d_2D_regs.reg_para9_koj_scs_main_htc_s_size);
		if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

		pr_info("[CAM]%s reg_para10_koj_start_htc_s\n",__func__);
		rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para10_koj_start_htc_s,
                               sp3d_2D_regs.reg_para10_koj_start_htc_s_size);
		if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

		#endif
		pr_info("[CAM]%s reg_para11_koj_stop_htc_m\n",__func__);
		rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para11_koj_stop_htc_m,
                               sp3d_2D_regs.reg_para11_koj_stop_htc_m_size);
		if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

		pr_info("[CAM]%s reg_para12_koj_scs_main_htc_m\n",__func__);
		rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para12_koj_scs_main_htc_m,
                               sp3d_2D_regs.reg_para12_koj_scs_main_htc_m_size);
		if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

		pr_info("[CAM]%s reg_para13_koj_start_htc_m\n",__func__);
		rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para13_koj_start_htc_m,
                               sp3d_2D_regs.reg_para13_koj_start_htc_m_size);
		if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

		/*check register status*/
		/*read register to make sure MIPI enable*/
		sp3d_check_reg_status();
		sp3d_wait_INT(1);
		sp3d_preview_setting();
		pr_info("[CAM]%s CPU init setting done",__func__);
		atomic_set(&preview_flag,true);
		return rc;
}


int sp3d_init_setting_thread(void){
	int rc = 0;
	pr_info("[CAM]%s reg_para8_koj_stop_htc_s\n",__func__);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para8_koj_stop_htc_s,
			sp3d_3D_regs.reg_para8_koj_stop_htc_s_size);
	if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

	pr_info("[CAM]%s reg_para9_koj_scs_main_htc_s\n",__func__);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para9_koj_scs_main_htc_s,
			sp3d_3D_regs.reg_para9_koj_scs_main_htc_s_size);
	if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

	pr_info("[CAM]%s reg_para10_koj_start_htc_s\n",__func__);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para10_koj_start_htc_s,
			sp3d_3D_regs.reg_para10_koj_start_htc_s_size);
	if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

	pr_info("[CAM]%s reg_para11_koj_stop_htc_m\n",__func__);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para11_koj_stop_htc_m,
			sp3d_3D_regs.reg_para11_koj_stop_htc_m_size);
	if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

	pr_info("[CAM]%s reg_para12_koj_scs_main_htc_m\n",__func__);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para12_koj_scs_main_htc_m,
			sp3d_3D_regs.reg_para12_koj_scs_main_htc_m_size);
	if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

	pr_info("[CAM]%s reg_para13_koj_start_htc_m\n",__func__);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para13_koj_start_htc_m,
			sp3d_3D_regs.reg_para13_koj_start_htc_m_size);
	if(SP3D_CHECK_SPI(rc) < 0 ) return rc;

	/*check register status*/
	/*read register to make sure MIPI enable*/
	sp3d_check_reg_status();
	sp3d_wait_INT(1);
	sp3d_preview_setting();
	pr_info("[CAM]%s CPU init setting done",__func__);
	atomic_set(&preview_flag,true);
	return rc;
}

static irqreturn_t sp3d_irq_handler(int irq, void *dev_id){
	unsigned long flags;
	smp_mb();
	spin_lock_irqsave(&sp3d_int.sp3d_spin_lock,flags);

#if 1
	if (atomic_read(&sp3d_snapshot_state) != SP3D_SNAPSHOT_NOT_START) 
	{
		atomic_add(1,&sp3d_snapshot_state);
		smp_mb();
	}
	if (atomic_read(&sp3d_2Dsnapshot_state) != SP3D_2DSNAPSHOT_NOT_START) 
	{
		atomic_add(1,&sp3d_2Dsnapshot_state);
		smp_mb();
	}
#endif

	if (atomic_read(&start_counting))
		atomic_add(1,&sp3d_int.frame_count);
	smp_mb();
	if(atomic_read(&snapshot_flag)){
		if(atomic_read(&sp3d_snapshot_state) != SP3D_2DSNAPSHOT_NOT_START ||
			atomic_read(&sp3d_2Dsnapshot_state) != SP3D_SNAPSHOT_NOT_START){
			pr_info("[CAM]----------Got INT from sharp state3D:%d state2D:%d---------------"
			,atomic_read(&sp3d_snapshot_state)
			,atomic_read(&sp3d_2Dsnapshot_state));
		}
	}

	wake_up(&sp3d_int.sp3d_wait);
	spin_unlock_irqrestore(&sp3d_int.sp3d_spin_lock,flags);
	return IRQ_HANDLED;
}

static int sp3d_create_irq(void){
	init_waitqueue_head(&sp3d_int.sp3d_wait);
	return request_irq(MSM_GPIO_TO_INT(106), sp3d_irq_handler,
			IRQF_TRIGGER_RISING, "sp3d_irq", 0);
}

static int sp3d_2D_init_setting(void){
	int rc = 0;
	pr_info("[CAM]sp3d init in 2D mode");
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para1_koj_mixer_2d_htc,
			sp3d_2D_regs.reg_para1_koj_mixer_2d_htc_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	udelay(100);
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para1_koj_mixer_2d_htc_1,
			sp3d_2D_regs.reg_para1_koj_mixer_2d_htc_size_1);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	udelay(10);
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para2_koj_init_2d_htc_s,
			sp3d_2D_regs.reg_para2_koj_init_2d_htc_s_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	mdelay(1);
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para2_koj_init_2d_htc_s_1,
			sp3d_2D_regs.reg_para2_koj_init_2d_htc_s_size_1);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	udelay(100);
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para2_koj_init_2d_htc_s_2,
			sp3d_2D_regs.reg_para2_koj_init_2d_htc_s_size_2);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

#if 1 /*take off due to 2D mode dosent need this setting suggest by sharp*/
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para3_koj_scs_2d_boot_htc_s,
		sp3d_2D_regs.reg_para3_koj_scs_2d_boot_htc_s_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para4_koj_start_htc_s,
		sp3d_2D_regs.reg_para4_koj_start_htc_s_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;
#endif


	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para5_koj_init_2d_htc_m,
		sp3d_2D_regs.reg_para5_koj_init_2d_htc_m_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	mdelay(1);
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para5_koj_init_2d_htc_m_1,
			sp3d_2D_regs.reg_para5_koj_init_2d_htc_m_size_1);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	udelay(100);
	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para5_koj_init_2d_htc_m_2,
			sp3d_2D_regs.reg_para5_koj_init_2d_htc_m_size_2);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para6_koj_scs_2d_boot_htc_m,
			sp3d_2D_regs.reg_para6_koj_scs_2d_boot_htc_m_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	rc = sp3d_spi_write_table(sp3d_2D_regs.reg_para7_koj_start_htc_m,
			sp3d_2D_regs.reg_para7_koj_start_htc_m_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	/*check register status*/
	/*read register to make sure MIPI enable*/
	sp3d_check_reg_status();
	sp2d_init_setting_thread();
	atomic_set(&sp3d_2Dsnapshot_state, SP3D_2DSNAPSHOT_NOT_START);
	smp_mb();
	return rc;
}

static int sp3d_3D_init_setting(void){
	int rc = 0;
	pr_info("[CAM]sp3d init in 3D mode");
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para1_koj_mixer_3d_htc,
		sp3d_3D_regs.reg_para1_koj_mixer_3d_htc_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	udelay(100);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para1_koj_mixer_3d_htc_1,
		sp3d_3D_regs.reg_para1_koj_mixer_3d_htc_size_1);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	udelay(10);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para2_koj_init_3d_htc_s,
		sp3d_3D_regs.reg_para2_koj_init_3d_htc_s_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	mdelay(1);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para2_koj_init_3d_htc_s_1,
		sp3d_3D_regs.reg_para2_koj_init_3d_htc_s_size_1);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	udelay(100);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para2_koj_init_3d_htc_s_2,
		sp3d_3D_regs.reg_para2_koj_init_3d_htc_s_size_2);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para3_koj_scs_3d_boot_htc_s,
		sp3d_3D_regs.reg_para3_koj_scs_3d_boot_htc_s_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para4_koj_start_htc_s,
		sp3d_3D_regs.reg_para4_koj_start_htc_s_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para5_koj_init_3d_htc_m,
		sp3d_3D_regs.reg_para5_koj_init_3d_htc_m_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	mdelay(1);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para5_koj_init_3d_htc_m_1,
		sp3d_3D_regs.reg_para5_koj_init_3d_htc_m_size_1);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	udelay(100);
	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para5_koj_init_3d_htc_m_2,
		sp3d_3D_regs.reg_para5_koj_init_3d_htc_m_size_2);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para6_koj_scs_3d_boot_htc_m,
		sp3d_3D_regs.reg_para6_koj_scs_3d_boot_htc_m_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	rc = sp3d_spi_write_table(sp3d_3D_regs.reg_para7_koj_start_htc_m,
		sp3d_3D_regs.reg_para7_koj_start_htc_m_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	/*read register to make sure MIPI enable*/
	sp3d_check_reg_status();
	sp3d_init_setting_thread();
#if SP3D_DEBUG_MODE
	msm_camio_csi_misr_debug_on();
#endif
	atomic_set(&sp3d_snapshot_state, SP3D_SNAPSHOT_NOT_START);
	smp_mb();
	
	return rc;
}

static int32_t sp3d_sensor_setting(int update_type, int rt)
{
	int32_t rc = 0;
	
	struct msm_camera_csi_params sp3d_csi_params;
	pr_info("[CAM]sp3d_sensor_setting type:%d config:%d\n",update_type, rt);
	switch (update_type) {
	case REG_INIT:
		/*create irq*/
		rc = sp3d_create_irq();
		if(rc < 0)
			pr_err("[CAM]request GPIO irq error");
		sp3d_state = SP3D_PREVIEW_MODE;
		break;
	case UPDATE_PERIODIC:/*3D*/
		if (!config_csi) {
			msm_camio_csi_disable_lp_rec();
			msm_camio_csi_core_reset();
			mdelay(5);
			msm_camio_csi_core_on();
			mdelay(5);
			pr_info("[CAM]csi_config");
			sp3d_config_csi_param(&sp3d_csi_params);
			rc = msm_camio_csi_config_withReceiverDisabled(&sp3d_csi_params);
			config_csi = 1;
			if(dmode == CAMERA_3D_MODE)
				sp3d_3D_init_setting();
			else
				sp3d_2D_init_setting();
			msm_camio_csi_enable_lp_rec();
			break;
		}
		sp3d_wait_INT(1);
		if(rt == RES_CAPTURE){
			sp3d_state = SP3D_SNAPSHOT_MODE;
			sp3d_start_snapshot();
		}else if(rt == RES_PREVIEW){
			sp3d_preview_setting();
			if(sp3d_state == SP3D_SNAPSHOT_MODE){
				sp3d_wait_INT(1);
				sp3d_stop_snapshot();
			}

			sp3d_state = SP3D_PREVIEW_MODE;
		}else if(rt == RES_VIDEO){
			/*2D 720p camcode only*/
			sp3d_video_setting();
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t sp3d_video_config(int mode)
{
	int32_t	rc = 0;
	rc = sp3d_sensor_setting(UPDATE_PERIODIC, RES_VIDEO);
	if (SP3D_CHECK_SPI(rc) < 0)
		return rc;

	sp3d_ctrl->curr_res = sp3d_ctrl->prev_res;
	sp3d_ctrl->sensormode = mode;
	return rc;

}


static int32_t sp3d_preview_config(int mode)
{
	int32_t	rc = 0;
	int	rt = 0;
	/* change sensor resolution	if needed */
	if (sp3d_ctrl->prev_res == QTR_SIZE) {
		rt = RES_PREVIEW;
		sp3d_delay_msecs_stdby =
			((((2 * 1000 * sp3d_ctrl->fps_divider)/
				sp3d_ctrl->fps) * Q8) / Q10) + 1;
	} else {
		rt = RES_CAPTURE;
		sp3d_delay_msecs_stdby =
			((((1000 * sp3d_ctrl->fps_divider)/
			sp3d_ctrl->fps) * Q8) / Q10) + 1;
	}
	rc = sp3d_sensor_setting(UPDATE_PERIODIC, rt);
	if (SP3D_CHECK_SPI(rc) < 0)
		return rc;

	sp3d_ctrl->curr_res = sp3d_ctrl->prev_res;
	sp3d_ctrl->sensormode = mode;
	return rc;
}

static int32_t sp3d_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt = 0;
	/* change sensor resolution if needed */
	if (sp3d_ctrl->curr_res != sp3d_ctrl->pict_res) {
		if (sp3d_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;
			sp3d_delay_msecs_stdby =
				((((2*1000 * sp3d_ctrl->fps_divider)/
				sp3d_ctrl->fps) * Q8) / Q10) + 1;
		} else {
			rt = RES_CAPTURE;
			sp3d_delay_msecs_stdby =
				((((1000 * sp3d_ctrl->fps_divider)/
				sp3d_ctrl->fps) * Q8) / Q10) + 1;
		}
	}
	rc = sp3d_sensor_setting(UPDATE_PERIODIC, rt);
	if (SP3D_CHECK_SPI(rc) < 0)
		return rc;

	sp3d_ctrl->curr_res = sp3d_ctrl->pict_res;
	sp3d_ctrl->sensormode = mode;
	return rc;
}

static int32_t sp3d_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt = 0;
	/* change sensor resolution if needed */
	if (sp3d_ctrl->curr_res != sp3d_ctrl->pict_res) {
		if (sp3d_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;
			sp3d_delay_msecs_stdby =
				((((2*1000 * sp3d_ctrl->fps_divider)/
				sp3d_ctrl->fps) * Q8) / Q10) + 1;
		} else {
			rt = RES_CAPTURE;
				sp3d_delay_msecs_stdby =
					((((1000 * sp3d_ctrl->fps_divider)/
					sp3d_ctrl->fps) * Q8) / Q10) + 1;
		}
	}
	rc = sp3d_sensor_setting(UPDATE_PERIODIC, rt);
	if (SP3D_CHECK_SPI(rc) < 0)
		return rc;
	sp3d_ctrl->curr_res = sp3d_ctrl->pict_res;
	sp3d_ctrl->sensormode = mode;
	return rc;
}

static int32_t sp3d_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *sinfo = sp3d_pdev->dev.platform_data;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = sp3d_preview_config(mode);
		break;
	case SENSOR_VIDEO_MODE:
		rc = sp3d_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		rc = sp3d_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		rc = sp3d_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t sp3d_power_down(void)
{
	sp3d_vreg_disable(sp3d_ctrl->sensordata);
	return 0;
}

static int sp3d_probe_init_done(const struct msm_camera_sensor_info *data)
{
	sp3d_vreg_disable(data);
	return 0;
}

static int sp3d_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t val;
	pr_info("[CAM]%s\n", __func__);
	/* read registers and to verify:
	 * 1. 0x078  default value: 0x7eaf
	 * 2. 0x13c  default value: 0x01df
	 * 3. 0x164  default value: 0x0000 */
	rc = sp3d_spi_read(0x78, &val);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;
	mdelay(1);
	pr_info("[CAM]read 0x78, val = %x\n", val);
	if (val != 0x7eaf)
		return -EFAULT;

	rc = sp3d_spi_read(0x13c, &val);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;
	mdelay(1);
	pr_info("[CAM]read 0x13c, val = %x\n", val);
	if (val != 0x01df)
		return -EFAULT;

	return rc;
}


int sp3d_spi_open_init(struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	pr_info("[CAM]%s: %d\n", __func__, __LINE__);
	if(data == NULL){
		pr_info("data pointer is NULL");
		return -1;
	}
	sp3d_ctrl = kzalloc(sizeof(struct sp3d_spi_ctrl), GFP_KERNEL);
	if (!sp3d_ctrl) {
		pr_err("[CAM]sp3d_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	sp3d_ctrl->fps_divider = 1 * 0x00000400;
	sp3d_ctrl->pict_fps_divider = 1 * 0x00000400;
	sp3d_ctrl->fps = 30 * Q8;
	sp3d_ctrl->set_test = TEST_OFF;
	sp3d_ctrl->prev_res = QTR_SIZE;
	sp3d_ctrl->pict_res = FULL_SIZE;
	sp3d_ctrl->curr_res = INVALID_SIZE;
	config_csi = 0;
	sp3d_vreg_enable(data);
	data->pdata->camera_gpio_on();
	pr_info("[CAM]sp3d: sp3d_sensor_probe: switch clk\n");
	if(data->camera_clk_switch != NULL)
		data->camera_clk_switch();

	sp3d_ctrl->sensordata = data;

	/* enable mclk first */
	msm_camio_clk_rate_set(24000000);
	msleep(20);
	if(sp3d_spi_write_addr == NULL){
		sp3d_spi_write_addr = kcalloc(SP3D_MAX_ALLOCATE*8, sizeof(uint8_t), GFP_KERNEL);
		pr_info("sp3d_spi_write_addr start:%p end:%p",
			sp3d_spi_write_addr,(sp3d_spi_write_addr+SP3D_MAX_ALLOCATE*8));
	}
	pr_info("sp3d_spi_write_addr:0x%p",sp3d_spi_write_addr);
	rc = sp3d_probe_init_sensor(data);
	if (rc < 0) {
		CDBG("Calling sp3d_spi_open_init fail\n");
		goto init_fail;
	}

	rc = sp3d_sensor_setting(REG_INIT, RES_PREVIEW);
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;

init_fail:
	CDBG(" sp3d_spi_open_init fail\n");
	sp3d_probe_init_done(data);
	kfree(sp3d_ctrl);
init_done:
	CDBG("sp3d_spi_open_init done\n");
	return rc;
}

static int __exit sp3d_spi_remove(struct spi_device *spi)
{
	if (sp3d_spi_ctrl != spi_get_drvdata(spi))
		return -EFAULT;
	spin_lock_irq(&sp3d_spi_ctrl->spinlock);
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&sp3d_spi_ctrl->spinlock);

	kfree(sp3d_spi_ctrl);
	return 0;
}

static int sp3d_auto_focus(void)
{
	int rc = 0;
	pr_info("[CAM]---------- sp3d_auto_focus");
	sp3d_wait_INT(1);
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_pat_koj_af,
                               sp3d_com_feature_regs.reg_pat_koj_af_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	pr_info("[CAM]%s",__func__);
	/*delay 2 frame in case we get worng status from sharp AF status register*/
	sp3d_wait_INT(2);
	return rc;
}

static int sp3d_cancel_focus(void)
{
	int rc = 0;
	pr_info("[CAM]---------- sp3d_cancel_focus");
	sp3d_wait_INT(1);
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_pat_koj_cancel_af,
                               sp3d_com_feature_regs.reg_pat_koj_cancel_af_size);
	if (SP3D_CHECK_SPI(rc) < 0) return rc;

	pr_info("[CAM]%s",__func__);
	return rc;

}
static int sp3d_get_af_state(void){
	int rc = 0;
	uint16_t val,temp;
	sp3d_wait_INT(1);
	/*write*/
	sp3d_spi_write(0x0492,0x33F8);
	sp3d_spi_write(0x049A,0x020F);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0080);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0024);
	sp3d_spi_write(0x0490,0x0006);
	/*read*/
	rc = sp3d_spi_read(0x04A2, &val);

	pr_info("[CAM]sp3d get AF state val:0x%x ",val);
	if(rc < 0)
		pr_err("[CAM]sp3d get AF state faile");
	/*AF status*/
	if(val == 0x10){
		pr_info("[CAM]sp3d AF done");
		goto finish;
	}
	temp = val &(0x0001<<5);
	/*lens position*/
	if(temp){
		goto faile;
	}else{
		pr_err("[CAM]sp3d lens not moving");
	}
	temp = val &(0x0001<<6);
	/*lens position*/
	if(temp){
		pr_info("[CAM]sp3d SAF is moving");
		goto faile;
	}else{
		pr_info("[CAM]sp3d stop SAF");
	}
finish:
	return 0;
faile:
	return -EFAULT;
}

static int sp3d_set_effect(int8_t effect_type)
{
	int rc = 0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	switch (effect_type) {
	case CAMERA_EFFECT_OFF:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_effect_normal,
                               sp3d_com_feature_regs.reg_koj_effect_normal_size);
		break;
 	case CAMERA_EFFECT_MONO:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_effect_BW,
                               sp3d_com_feature_regs.reg_koj_effect_BW_size);
		break;
 	case CAMERA_EFFECT_NEGATIVE:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_effect_negative,
                               sp3d_com_feature_regs.reg_koj_effect_negative_size);
		break;
 	case CAMERA_EFFECT_SOLARIZE:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_effect_solarize,
                               sp3d_com_feature_regs.reg_koj_effect_solarize_size);
		break;
 	case CAMERA_EFFECT_SEPIA:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_effect_sepia,
                               sp3d_com_feature_regs.reg_koj_effect_sepia_size);
		break;
 	case CAMERA_EFFECT_POSTERIZE:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_effect_posterize,
                               sp3d_com_feature_regs.reg_koj_effect_posterize_size);
		break;
 	case CAMERA_EFFECT_WHITEBOARD:
		rc = -EFAULT;
		break;
 	case CAMERA_EFFECT_BLACKBOARD:
		rc = -EFAULT;
		break;
 	case CAMERA_EFFECT_AQUA:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_effect_blue_tone,
                               sp3d_com_feature_regs.reg_koj_effect_blue_tone_size);
		break;
	default:
	rc = -EFAULT;
		break;
	}
	pr_info("[CAM]%s",__func__);
	return rc;

}


static int sp3d_get_iso(uint16_t *real_iso_value)
{
    int rc=0;
    uint16_t rval, AgcGain;

    /*write*/
    sp3d_spi_write(0x0492,0x33F8);
    sp3d_spi_write(0x049A,0x020F);
    sp3d_spi_write(0x0490,0x0402);
    //read 0xE0 for hight byte
    sp3d_spi_write(0x049A,0x00E0);
    sp3d_spi_write(0x0490,0x0402);
    sp3d_spi_write(0x049A,0x0024);
    sp3d_spi_write(0x0490,0x0006);
    /*read*/
    rc = sp3d_spi_read(0x04A2, &rval);
    if (SP3D_CHECK_SPI(rc) < 0) return rc;
    AgcGain = rval << 8;

    /*write*/
    sp3d_spi_write(0x0492,0x33F8);
    sp3d_spi_write(0x049A,0x020F);
    sp3d_spi_write(0x0490,0x0402);
	//read 0xDF for low byte
    sp3d_spi_write(0x049A,0x00DF);
    sp3d_spi_write(0x0490,0x0402);
    sp3d_spi_write(0x049A,0x0024);
    sp3d_spi_write(0x0490,0x0006);
    /*read*/
    rc = sp3d_spi_read(0x04A2, &rval);
    if (SP3D_CHECK_SPI(rc) < 0) return rc;

    AgcGain = AgcGain | (rval & 0x00ff);
    *real_iso_value = AgcGain * 32 / 16;
    pr_info("%s real_iso_value: %d AgcGain:%d",__func__, *real_iso_value,AgcGain);
    return rc;
}

static int sp3d_set_iso(enum iso_mode iso_type)
{
	int rc=0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	switch (iso_type) {
	case CAMERA_ISO_AUTO:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_iso_auto,
                               sp3d_com_feature_regs.reg_koj_iso_auto_size);
		break;
 	case CAMERA_ISO_100:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_iso_100,
                               sp3d_com_feature_regs.reg_koj_iso_100_size);
		break;
 	case CAMERA_ISO_200:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_iso_200,
                               sp3d_com_feature_regs.reg_koj_iso_200_size);	
		break;
 	case CAMERA_ISO_400:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_iso_400,
                               sp3d_com_feature_regs.reg_koj_iso_400_size);
		break;
 	case CAMERA_ISO_800:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_iso_800,
                               sp3d_com_feature_regs.reg_koj_iso_800_size);
		break;
	default:
	rc = -EFAULT;
		break;
	}
	pr_info("[CAM]%s",__func__);
	return rc;

}



static int sp3d_set_wb(enum wb_mode wb_type)
{
	int rc=0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	switch (wb_type) {
	case CAMERA_AWB_AUTO:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_wb_auto,
                               sp3d_com_feature_regs.reg_koj_wb_auto_size);
		break;
 	case CAMERA_AWB_CLOUDY:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_wb_cloudy,
                               sp3d_com_feature_regs.reg_koj_wb_cloudy_size);
		break;
 	case CAMERA_AWB_INDOOR_HOME:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_wb_fluorescent,
                               sp3d_com_feature_regs.reg_koj_wb_fluorescent_size);
		break;
 	case CAMERA_AWB_INDOOR_OFFICE:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_wb_incandescent,
                               sp3d_com_feature_regs.reg_koj_wb_incandescent_size);
		break;
 	case CAMERA_AWB_SUNNY:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_wb_daylight,
                               sp3d_com_feature_regs.reg_koj_wb_daylight_size);
		break;
	default:
	rc = -EFAULT;
		break;
	}
	pr_info("[CAM]%s",__func__);
	return rc;
}


static int sp3d_set_ev(enum brightness_t ev_type)
{
	int rc =0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	switch (ev_type) {
	case CAMERA_BRIGHTNESS_N2:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_ev_neg2,
                               sp3d_com_feature_regs.reg_koj_ev_neg2_size);
		break;
 	case CAMERA_BRIGHTNESS_N1:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_ev_neg1,
                               sp3d_com_feature_regs.reg_koj_ev_neg1_size);
		break;
 	case CAMERA_BRIGHTNESS_D:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_ev_0,
                               sp3d_com_feature_regs.reg_koj_ev_0_size);
		break;
 	case CAMERA_BRIGHTNESS_P1:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_ev_1,
                               sp3d_com_feature_regs.reg_koj_ev_1_size);
		break;
 	case CAMERA_BRIGHTNESS_P2:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_ev_2,
                               sp3d_com_feature_regs.reg_koj_ev_2_size);
		break;
	default:
	rc = -EFAULT;	
		break;
	}
	pr_info("[CAM]%s",__func__);
	return rc;
}


static int sp3d_set_antibanding(enum antibanding_mode antibanding_type)
{
	int rc =0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	switch (antibanding_type) {
	case CAMERA_ANTI_BANDING_50HZ:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_antibanding_50,
                               sp3d_com_feature_regs.reg_koj_antibanding_50_size);
		break;
 	case CAMERA_ANTI_BANDING_60HZ:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_antibanding_60,
                               sp3d_com_feature_regs.reg_koj_antibanding_60_size);
		break;
 	case CAMERA_ANTI_BANDING_AUTO:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_antibanding_auto,
                               sp3d_com_feature_regs.reg_koj_antibanding_auto_size);
		break;
	default:
	rc = -EFAULT;	
		break;
	}
	pr_info("[CAM]%s",__func__);
	return rc;
}


static int sp3d_set_contrast(enum contrast_mode contrast_type)
{
	int rc =0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	switch (contrast_type) {
	case CAMERA_CONTRAST_P2:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_contrast_2,
                               sp3d_com_feature_regs.reg_koj_contrast_2_size);
		break;
 	case CAMERA_CONTRAST_P1:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_contrast_1,
                               sp3d_com_feature_regs.reg_koj_contrast_1_size);
		break;
 	case CAMERA_CONTRAST_D:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_contrast_0,
                               sp3d_com_feature_regs.reg_koj_contrast_0_size);
		break;
 	case CAMERA_CONTRAST_N1:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_contrast_neg1,
                               sp3d_com_feature_regs.reg_koj_contrast_neg1_size);
		break;
 	case CAMERA_CONTRAST_N2:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_contrast_neg2,
                               sp3d_com_feature_regs.reg_koj_contrast_neg2_size);
		break;
	default:
	rc = -EFAULT;	
		break;
	}
	pr_info("[CAM]%s",__func__);
	return rc;
}


static int sp3d_set_sharpness(enum sharpness_mode sharpness_type)
{
	int rc =0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;

	switch (sharpness_type) {
	case CAMERA_SHARPNESS_X0:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_sharpness_neg2,
                               sp3d_com_feature_regs.reg_koj_sharpness_neg2_size);
		break;
 	case CAMERA_SHARPNESS_X1:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_sharpness_neg1,
                               sp3d_com_feature_regs.reg_koj_sharpness_neg1_size);
		break;
 	case CAMERA_SHARPNESS_X2:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_sharpness_0,
                               sp3d_com_feature_regs.reg_koj_sharpness_0_size);
		break;
 	case CAMERA_SHARPNESS_X3:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_sharpness_1,
                               sp3d_com_feature_regs.reg_koj_sharpness_1_size);
		break;
 	case CAMERA_SHARPNESS_X4:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_sharpness_2,
                               sp3d_com_feature_regs.reg_koj_sharpness_2_size);
		break;
	case CAMERA_SHARPNESS_X5:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_sharpness_3,
                               sp3d_com_feature_regs.reg_koj_sharpness_2_size);
		break;
	case CAMERA_SHARPNESS_X6:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_sharpness_4,
                               sp3d_com_feature_regs.reg_koj_sharpness_2_size);
		break;
	default:
	rc = -EFAULT;	
		break;
	}
	pr_info("[CAM]%s sharpness_type:%d",__func__,sharpness_type);
	return rc;
}

static int sp3d_set_saturation(enum saturation_mode saturation_type)
{
	int rc =0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	switch (saturation_type) {
	case CAMERA_SATURATION_X0:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_saturation_neg2,
                               sp3d_com_feature_regs.reg_koj_saturation_neg2_size);
		break;
 	case CAMERA_SATURATION_X05:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_saturation_neg1,
                               sp3d_com_feature_regs.reg_koj_saturation_neg1_size);
		break;
 	case CAMERA_SATURATION_X1:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_saturation_0,
                               sp3d_com_feature_regs.reg_koj_saturation_0_size);	
		break;
 	case CAMERA_SATURATION_X15:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_saturation_1,
                               sp3d_com_feature_regs.reg_koj_saturation_1_size);
		break;
 	case CAMERA_SATURATION_X2:
	rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_saturation_2,
                               sp3d_com_feature_regs.reg_koj_saturation_2_size);	
		break;
	default:
	rc = -EFAULT;	
		break;
	}
	pr_info("[CAM]%s",__func__);
	return rc;
}


static void sp3d_set_mode(int mode){
	pr_info("[CAM]sp3d_set_mode:%d",mode);
	dmode = mode;
}


static int sp3d_set_af_mode( enum sensor_af_mode af_mode_value)
{
	int rc =0;
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	pr_info("[CAM]-------  sp3d_set_af_mode:%d",af_mode_value);
	switch (af_mode_value) {
	case SENSOR_AF_MODE_NORMAL:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_af_mode_normal,
                               sp3d_com_feature_regs.reg_koj_af_mode_normal_size);
		break;
	case SENSOR_AF_MODE_MACRO:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_af_mode_macro,
                               sp3d_com_feature_regs.reg_koj_af_mode_macro_size);
		break;
	case SENSOR_AF_MODE_AUTO:
		rc = sp3d_spi_write_table2(sp3d_com_feature_regs.reg_koj_af_mode_allrange,
                               sp3d_com_feature_regs.reg_koj_af_mode_allrange_size);
		break;
	default:
	rc = -EFAULT;
		break;
	}
	pr_info("[CAM]%s",__func__);
	return rc;
}

static int  sp3d_set_af_area(int af_area){
	uint16_t area = 0;
	int16_t rc = 0;		/* HTC Glenn 20110721 for klockwork issue */
	if(atomic_read(&snapshot_flag) || !atomic_read(&preview_flag))
		return rc;
	sp3d_wait_INT(1);
	area = 0x0500 | (uint16_t)af_area;
	/*# SSDCTL2 (master camera)*/
	rc = sp3d_spi_write(0x0492, 0x33F8); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x049A, 0x020E); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x049A, 0x0000); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	/*# 0E00:01  #1st AF window*/
	rc = sp3d_spi_write(0x049A, area); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x049A, 0x020D); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x049A, 0x000D); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	/*# 0D0D:22*/
	rc = sp3d_spi_write(0x049A, 0x0522); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	/*# SSDCTL2 (slave camera)*/
	rc = sp3d_spi_write(0x0492, 0x37F8); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x049A, 0x020E); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x049A, 0x0000); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	/*# 0E00:01  #1st AF window*/
	rc = sp3d_spi_write(0x049A, area); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x049A, 0x020D); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x049A, 0x000D); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	/*# 0D0D:22*/
	rc = sp3d_spi_write(0x049A, 0x0522); if(SP3D_CHECK_SPI(rc)<0) return rc;
	rc = sp3d_spi_write(0x0490, 0x0402); if(SP3D_CHECK_SPI(rc)<0) return rc;
	pr_info("[CAM]%s",__func__);
	return rc;
}

/*OPT*/
static void sp3d_switch_to_slave(void){
	uint16_t rval = 0;
	pr_info("[3D calibration] switch to slave");
	/*switch from master to slave*/
	sp3d_spi_write(0x49A,0x020C);
	sp3d_spi_write(0x490,0x0402);
	sp3d_spi_write(0x49A,0x003A);
	sp3d_spi_write(0x490,0x0402);
	sp3d_spi_write(0x49A,0x0132);
	sp3d_spi_write(0x490,0x0402);
	/*wait slave ready*/
	while(1){
		sp3d_spi_write(0x492,0x37F8);
		sp3d_spi_write(0x49A,0x020D);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x0001);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x011C);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x020C);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x003A);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x0133);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x020F);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x00FC);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x0020);
		sp3d_spi_write(0x490,0x0006);
		sp3d_spi_read(0x4A2,&rval);
		if(rval == 0x80){
			pr_info("[3D calibration]slave ready");
			break;
		}
	}
}

static void sp3d_dump_OTP(struct Sp3d_OTP *otp_data){
	pr_info("[3D calibration]otp_data.coefA1:0x%llx",otp_data->coefA1);
	pr_info("[3D calibration]otp_data.coefB1:0x%llx",otp_data->coefB1);
	pr_info("[3D calibration]otp_data.coefC1:0x%llx",otp_data->coefC1);
	pr_info("[3D calibration]otp_data.coefA2:0x%llx",otp_data->coefA2);
	pr_info("[3D calibration]otp_data.coefB2:0x%llx",otp_data->coefB2);
	pr_info("[3D calibration]otp_data.coefC2:0x%llx",otp_data->coefC2);
	pr_info("[3D calibration]otp_data.coefA3:0x%llx",otp_data->coefA3);
	pr_info("[3D calibration]otp_data.coefB3:0x%llx",otp_data->coefB3);
	pr_info("[3D calibration]otp_data.coefC3:0x%llx",otp_data->coefC3);
	
}

static void sp3d_switch_endian(uint8_t *otp_in_array){
	int i,j,k;
	uint8_t otp_out_array[0x50];
	for(i=0;i<=0x47;i=i+8){
		k=0;
		for(j=7;j>=0;j--){
			otp_out_array[i+k]=otp_in_array[i+j];
			k++;
		}
	}
	memcpy(otp_in_array,otp_out_array,sizeof(otp_out_array));
}

static void sp3d_read_OTP(struct Sp3d_OTP *otp_data,struct otp_cfg *sp3d_otp_cfg){
	uint16_t rval,count=0x00,otp_version,id_avaliabe;
	uint8_t otp_array[0x50];
	uint8_t otp_id[11];
	/*read coefficient*/
	while(count < 0x50){
		sp3d_spi_write(0x49A,0x020D);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x0001);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x011C);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x020F);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,count);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x0020);
		sp3d_spi_write(0x490,0x0006);
		sp3d_spi_read(0x4A2,&rval);
		//pr_info("[3D calibration]rval:0x%x",rval);
		if(count == 0x48){
			otp_version = rval;
			sp3d_otp_cfg->sp3d_otp_version = otp_version;
			pr_info("[3D calibration]otp_version:0x%x",otp_version);
		}
		otp_array[count] = (uint8_t)rval;
		count ++;
	}
	/*read id avaliable*/
	sp3d_spi_write(0x49A,0x020D);
	sp3d_spi_write(0x490,0x0402);
	sp3d_spi_write(0x49A,0x0001);
	sp3d_spi_write(0x490,0x0402);
	sp3d_spi_write(0x49A,0x011C);
	sp3d_spi_write(0x490,0x0402);
	sp3d_spi_write(0x49A,0x020F);
	sp3d_spi_write(0x490,0x0402);
	sp3d_spi_write(0x49A,0x50);/*0x50 is id avaliable*/
	sp3d_spi_write(0x490,0x0402);
	sp3d_spi_write(0x49A,0x0020);
	sp3d_spi_write(0x490,0x0006);
	sp3d_spi_read(0x4A2,&rval);
	id_avaliabe = rval;
	pr_info("[3D calibration]id_avaliabe:0x%x",id_avaliabe);
	if(id_avaliabe == 0x80)
		pr_info("[3D calibration]id is avaliabe");
	if(id_avaliabe == 0x81)
		pr_info("[3D calibration]id is not avaliabe");

	sp3d_switch_endian(otp_array);
	memcpy(&otp_data->coefA1,otp_array,sizeof(unsigned long long));
	memcpy(&otp_data->coefB1,otp_array+8,sizeof(unsigned long long));
	memcpy(&otp_data->coefC1,otp_array+16,sizeof(unsigned long long));
	memcpy(&otp_data->coefA2,otp_array+24,sizeof(unsigned long long));
	memcpy(&otp_data->coefB2,otp_array+32,sizeof(unsigned long long));
	memcpy(&otp_data->coefC2,otp_array+40,sizeof(unsigned long long));
	memcpy(&otp_data->coefA3,otp_array+48,sizeof(unsigned long long));
	memcpy(&otp_data->coefB3,otp_array+56,sizeof(unsigned long long));
	memcpy(&otp_data->coefC3,otp_array+64,sizeof(unsigned long long));
	/*read module ID*/
	count = 0x0054;
	while(count <= 0x005E){
		sp3d_spi_write(0x49A,0x020D);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x0001);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x011C);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x020F);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,count);
		sp3d_spi_write(0x490,0x0402);
		sp3d_spi_write(0x49A,0x0020);
		sp3d_spi_write(0x490,0x0006);
		sp3d_spi_read(0x4A2,&rval);
		otp_id[count - 0x0054] = rval;
		pr_info("[3D calibration]Module ID:0x%x,count 0x%x",rval,count);
		memcpy(sp3d_otp_cfg->sp3d_id,otp_id,sizeof(otp_id));
		count ++;
	}
	pr_info("[3D calibration]read OTP finish");
}


static void sp3d_calibration_ready(void){
	
	uint16_t rval;
	int count = 0;
	while(1){
		count ++;
		sp3d_spi_write(0x0492,0x33F8);
		sp3d_spi_write(0x049A,0x020F);
		sp3d_spi_write(0x0490,0x0402);
		sp3d_spi_write(0x049A,0x00FC);
		sp3d_spi_write(0x0490,0x0402);
		sp3d_spi_write(0x049A,0x0024);
		sp3d_spi_write(0x0490,0x0006);
		sp3d_spi_read(0x04A2, &rval);
		if(rval == 0x80){
			pr_info("[3D Calibration]master ready");
			break;
		}
		if(count > 200){
			pr_info("[3D Calibration]%s check reg time out",__func__);
			break;
		}
	}
}

static int sp3d_set_calibration(struct otp_cfg *sp3d_otp_cfg){
	int rc = 0;
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para1_koj_mixer_cal_3d ,
                               sp3d_calibration_regs.reg_para1_koj_mixer_cal_3d_size);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	udelay(100);
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para1_koj_mixer_cal_3d_1 ,
                               sp3d_calibration_regs.reg_para1_koj_mixer_cal_3d_size_1);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	mdelay(1);
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para2_koj_init_cal_3d_s ,
                               sp3d_calibration_regs.reg_para2_koj_init_cal_3d_s_size);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	udelay(100);
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para2_koj_init_cal_3d_s_1 ,
                               sp3d_calibration_regs.reg_para2_koj_init_cal_3d_s_size_1);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para2_koj_init_cal_3d_s_2 ,
                               sp3d_calibration_regs.reg_para2_koj_init_cal_3d_s_size_2);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para3_koj_scs_cal_3d_s ,
                               sp3d_calibration_regs.reg_para3_koj_scs_cal_3d_s_size);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para4_koj_start_cal_3d_s ,
                               sp3d_calibration_regs.reg_para4_koj_start_cal_3d_s_size);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para5_koj_init_cal_3d_m ,
                               sp3d_calibration_regs.reg_para5_koj_init_cal_3d_m_size);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	msleep(1);
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para5_koj_init_cal_3d_m_1 ,
                               sp3d_calibration_regs.reg_para5_koj_init_cal_3d_m_size_1);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	usleep(100);
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para5_koj_init_cal_3d_m_2 ,
                               sp3d_calibration_regs.reg_para5_koj_init_cal_3d_m_size_2);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para6_koj_scs_cal_3d_m ,
                               sp3d_calibration_regs.reg_para6_koj_scs_cal_3d_m_size);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	rc = sp3d_spi_write_table(sp3d_calibration_regs.reg_para7_koj_cal_3d_m ,
                               sp3d_calibration_regs.reg_para7_koj_cal_3d_m_size);
	if(rc < 0){
		pr_info("[SP3D calibration] reg_para1_koj_mixer_cal_3d");
		goto faile;
	}
	sp3d_calibration_ready();
	sp3d_read_OTP(&sp3d_otp_cfg->master_otp,sp3d_otp_cfg);
	sp3d_dump_OTP(&sp3d_otp_cfg->master_otp);
	sp3d_switch_to_slave();
	sp3d_read_OTP(&sp3d_otp_cfg->slave_otp,sp3d_otp_cfg);
	sp3d_dump_OTP(&sp3d_otp_cfg->slave_otp);
	return rc;
faile:
	pr_err("[SP3D calibration]calibration setting faile");
       return rc;                        
}

static uint16_t sp3d_read_Ext_mode(uint16_t mode){
	uint16_t rval = 0;
	sp3d_spi_write(0x0492,0x33F8);
	sp3d_spi_write(0x049A,0x020F);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,mode);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0024);
	sp3d_spi_write(0x0490,0x0006);
	sp3d_spi_read(0x04A2,&rval);
	 //pr_info("%s mode:0x%x rval:0x%x",__func__,mode,rval);
	return rval;
}

static int sp3d_check_Ext_Stable(struct flash_cfg* flash_data){
	uint16_t ExtTblMax=0xDC;
	uint16_t ExtTblIndx=0xDD;
	uint16_t IrisRatio=0x00DE;
	uint8_t Alpha=0x40, beta=0x46;
	uint16_t ExtMax = 0;
	uint8_t timeout=10,i;/*600ms*/
	ExtMax = sp3d_read_Ext_mode(ExtTblMax);
	for(i=0;i<timeout;i++){
		exp_pre = sp3d_read_Ext_mode(ExtTblIndx);
		iris_pre = sp3d_read_Ext_mode(IrisRatio);
		pr_info("[CAM]sp3d_flashlight check stable IrisR:0x%x ExtIndx:0x%x ExtMax:0x%x",iris_pre,exp_pre,ExtMax);
		if(i >= 2 && iris_pre > Alpha && iris_pre < beta){/*checking after first 2 frame*/
			pr_info("[CAM]sp3d_flashlight Exposure stable");
			break;
		}else
			mdelay(100);
	}
	if (i == timeout)
		pr_info("[CAM]sp3d_flashlight pre flash time out");
	flash_data->exp_off = exp_off;
	flash_data->exp_pre = exp_pre;
	flash_data->luma_off = iris_off;
	flash_data->luma_pre = iris_pre;
	return 0;
}

static int sp3d_change_framerate(struct sensor_cfg_data *cdata){
	int rc = 0;
	//cdata->mode = RES_PREVIEW;
	pr_info("sp3d_change_framerate cdata->cfg.down_framerate:%d",cdata->cfg.down_framerate);
	pr_info("sp3d_change_framerate cdata->mode:%d",cdata->mode);
	switch(cdata->mode){
		case SENSOR_VIDEO_MODE:
			if(cdata->cfg.down_framerate == 1){
				pr_info("%s 2D video mode 15fps",__func__);
				rc = sp3d_spi_write_table2(sp3d_2D_regs.reg_koj_2d_720p_video_15fps,
                               sp3d_2D_regs.reg_koj_2d_720p_video_15fps_size);
			}else{
				pr_info("%s 2D video mode 24fps",__func__);
				rc = sp3d_spi_write_table2(sp3d_2D_regs.reg_koj_2d_720p_video,
                               sp3d_2D_regs.reg_koj_2d_720p_video_size);
			}
			break;
		case SENSOR_PREVIEW_MODE:
			if(cdata->cfg.down_framerate == 1){
				pr_info("%s 2D preview mode 15fps",__func__);
				rc = sp3d_spi_write_table2(sp3d_2D_regs.reg_koj_2d_preview_15fps,
                               sp3d_2D_regs.reg_koj_2d_preview_15fps_size);
			}else{
				pr_info("%s 2D preview mode 24fps",__func__);
				rc = sp3d_spi_write_table2(sp3d_2D_regs.reg_koj_2d_preview,
                               sp3d_2D_regs.reg_koj_2d_preview_size);
			}
			break;
		default:
			pr_err("%s wrong mode",__func__);
			break;
	}
	return rc;
}



static int sp3d_get_exp_gain(struct exp_cfg *exp_data)
{
    int rc=0;
    uint16_t agcgain0=0xDF;
    uint16_t agcgain1=0xE0;
    uint16_t exptimeNum0=0xE4;
    uint16_t exptimeNum1=0xE5;
    uint16_t exptimeNum2=0xE6;
    uint16_t exptimeNum3=0xE7;
    uint16_t exptimeDen0=0xE8;
    uint16_t exptimeDen1=0xE9;
    uint16_t exptimeDen2=0xEA;
    uint16_t exptimeDen3=0xEB;
    uint16_t af_area=0xB0;
    uint16_t rval;
    exp_data->AGC_Gain1 = sp3d_read_Ext_mode(agcgain0);
    exp_data->AGC_Gain2 = sp3d_read_Ext_mode(agcgain1);
    exp_data->ExposureTimeNum0 = sp3d_read_Ext_mode(exptimeNum0);
    exp_data->ExposureTimeNum1 = sp3d_read_Ext_mode(exptimeNum1);
    exp_data->ExposureTimeNum2 = sp3d_read_Ext_mode(exptimeNum2);
    exp_data->ExposureTimeNum3 = sp3d_read_Ext_mode(exptimeNum3);
    exp_data->ExposureTimeDen0 = sp3d_read_Ext_mode(exptimeDen0);
    exp_data->ExposureTimeDen1 = sp3d_read_Ext_mode(exptimeDen1);
    exp_data->ExposureTimeDen2= sp3d_read_Ext_mode(exptimeDen2);
    exp_data->ExposureTimeDen3 = sp3d_read_Ext_mode(exptimeDen3);
    exp_data->AF_area= sp3d_read_Ext_mode(af_area);
	/*flicker compansation*/
	sp3d_spi_write(0x0492,0x33F8);
	sp3d_spi_write(0x049A,0x0204);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0071);
	sp3d_spi_write(0x0490,0x0402);
	sp3d_spi_write(0x049A,0x0024);
	sp3d_spi_write(0x0490,0x0006);
	sp3d_spi_read(0x04A2,&rval);
	pr_info("flicker_compansation:%0x",rval);
	exp_data->flicker_compansation = rval;
    return rc;
}




#define SP3D_EXP_AVERAGE 3
static int sp3d_make_decision(void){
	uint16_t ExtTblMax=0xDC;
	uint16_t ExtTblIndx=0xDD;
	uint16_t IrisRatio=0x00DE;
	uint16_t Alpha=0x38;
	uint16_t ExtMax = 0;
	uint16_t i = 0,iris_ave = 0;
	for(i = 1; i <= SP3D_EXP_AVERAGE;i ++){
		ExtMax = sp3d_read_Ext_mode(ExtTblMax);
		exp_off = sp3d_read_Ext_mode(ExtTblIndx);
		iris_off = sp3d_read_Ext_mode(IrisRatio);
		pr_info("[CAM]sp3d_flashlight make decision iris_off:0x%x exp_off:0x%x\n",iris_off,exp_off);
		iris_ave = iris_ave + iris_off;
	}
	iris_off = iris_ave/SP3D_EXP_AVERAGE;
	pr_info("[CAM]sp3d_flashlight make decision IrisR:0x%x ExtIndx:0x%x ExtMax:0x%x",
		iris_off,exp_off,ExtMax);
	if(iris_off >= Alpha){
		pr_info("[CAM]sp3d_flashlight make decision turn OFF flash");
		return false;/*do not need to use flash*/
	}else{
		pr_info("[CAM]sp3d_flashlight make decision turn ON flash");
		return true;
	}
}

static int sp3d_set_flashlight(int flash_mode, struct flash_cfg* flash_data){
	int rc = 0;
	struct msm_camera_sensor_info *sdata = sp3d_pdev->dev.platform_data;
	flashlight_exp_div = 0;
	if(flash_mode == 0){/*flash off*/
		pr_info("sp3d_set_flashlight camera_flash FL_MODE_OFF");
		sdata->flash_data->flash_src->camera_flash(FL_MODE_OFF);
		pr_info("sp3d_set_flashlight done");
		flash_data->flash_enable = flash_state = false;
		return rc;
	}
	flash_state = sp3d_make_decision();
	if((flash_mode == 1/*flash auto*/ && flash_state)||flash_mode == 2/*flash on*/){
		flash_state = true;
		pr_info("[CAM]sp3d_flashlight flash on");
		sdata->flash_data->flash_src->camera_flash(FL_MODE_PRE_FLASH);
		rc = sp3d_check_Ext_Stable(flash_data);
	} else
		flash_state = false;

	sdata->flash_data->flash_src->camera_flash(FL_MODE_OFF);
	flash_data->flash_enable = flash_state;
	return rc;
}

static int sp3d_set_flashlight_exp_div(uint16_t flash_exp_div)
{
	flashlight_exp_div = flash_exp_div;
	return 0;
}
int sp3d_spi_config_temp(void *data)
{
	struct sensor_cfg_data *cdata;
	long   rc = 0;

	cdata = (struct sensor_cfg_data *)data;
	
	mutex_lock(&sp3d_mut);

	switch (cdata->cfgtype) {
	case CFG_SET_MODE:	
		pr_info("[CAM]SP3D Send Capture command\n");
		rc = sp3d_set_sensor_mode(cdata->mode,
			cdata->rs);
		break;

	case CFG_GET_SP3D_L_FRAME:
		pr_info("[CAM]SP3D Send L Frame command\n");
		rc = sp3d_3D_get_L_frame();
		break;

	case CFG_GET_SP3D_R_FRAME:
		pr_info("[CAM]SP3D Send R Frame command\n");
		rc = sp3d_3D_get_R_frame();
		break;

	default:
		rc = -EFAULT;
		break;
	}
	mutex_unlock(&sp3d_mut);

	return rc;	
}

int sp3d_spi_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&sp3d_mut);
	switch (cdata.cfgtype) {
	case CFG_SET_MODE:
		if ((dmode == CAMERA_3D_MODE) && (cdata.mode == SENSOR_SNAPSHOT_MODE))
		{
			pr_info("[CAM]bypass snapshot here\n");
			rc = 0;
		}
		else
			rc = sp3d_set_sensor_mode(cdata.mode,
			cdata.rs);
			break;
	case CFG_PWR_DOWN:
		rc = sp3d_power_down();
			break;
	case CFG_RUN_AUTO_FOCUS:
		rc = sp3d_auto_focus();
		break;
	case CFG_GET_AF_STATE:
		rc = sp3d_get_af_state();
		break;
	case CFG_SET_EFFECT:
		rc = sp3d_set_effect(cdata.cfg.effect);
		break;
	case CFG_SET_WB:
		rc = sp3d_set_wb(cdata.cfg.wb_value);
		break;
	case CFG_SET_ISO:
		rc = sp3d_set_iso(cdata.cfg.iso_value);
		break;
	case CFG_SET_DMODE:
		sp3d_set_mode(cdata.mode);
		break;
	case CFG_SET_SHARPNESS:
		rc = sp3d_set_sharpness(cdata.cfg.sharpness_value);
		break;
	case CFG_SET_SATURATION:
		rc = sp3d_set_saturation(cdata.cfg.saturation_value);
		break;
	case CFG_SET_CONTRAST:
		rc = sp3d_set_contrast(cdata.cfg.contrast_value);
		break;
	case CFG_SET_ANTIBANDING:
		rc = sp3d_set_antibanding(cdata.cfg.antibanding_value);
		break;	
	case CFG_SET_BRIGHTNESS:
		rc = sp3d_set_ev(cdata.cfg.brightness_value);
		break;
	case CFG_SET_COORDINATE:
		rc = sp3d_set_af_area(cdata.cfg.af_area);
		break;
	case CFG_SET_CALIBRATION:
		rc = sp3d_set_calibration(&cdata.cfg.sp3d_otp_cfg);
		if(copy_to_user((void *)argp,
			&cdata,sizeof(struct sensor_cfg_data))){
			pr_err("[CAM]%s copy to user error",__func__);
		}
		break;
	case CFG_SET_AF_MODE:
		rc = sp3d_set_af_mode(cdata.cfg.af_mode_value);
		break;
	case CFG_GET_SP3D_L_FRAME:
		rc = sp3d_3D_get_L_frame();
		break;
	case CFG_GET_SP3D_R_FRAME:
		rc = sp3d_3D_get_R_frame();
		break;
	case CFG_SET_FLASHLIGHT:
		rc = sp3d_set_flashlight(cdata.mode, &cdata.cfg.flash_data);
		if(copy_to_user((void *)argp,
			&cdata,sizeof(struct sensor_cfg_data))){
			pr_err("[CAM]%s copy to user error",__func__);
		}
		break;
	case CFG_SET_FLASHLIGHT_EXP_DIV:
		rc = sp3d_set_flashlight_exp_div(cdata.cfg.flash_exp_div);
		break;
	case CFG_CANCEL_AUTO_FOCUS:
		rc = sp3d_cancel_focus();
		break;
    	case CFG_GET_ISO:
		rc = sp3d_get_iso(&cdata.cfg.real_iso_value);
		if(copy_to_user((void *)argp,
			&cdata,sizeof(struct sensor_cfg_data))){
			pr_err("[CAM]%s copy to user error",__func__);
		}
        break;
	case CFG_GET_EXP_GAIN:
		rc = sp3d_get_exp_gain(&cdata.cfg.exp_info);
		if(copy_to_user((void *)argp,
			&cdata,sizeof(struct sensor_cfg_data))){
			pr_err("[CAM]%s copy to user error",__func__);
		}
		break;
	case CFG_SET_FRAMERATE:
		rc = sp3d_change_framerate(&cdata);
		break;
	default:
		rc = -EFAULT;
		break;
	}
	mutex_unlock(&sp3d_mut);

	return rc;
}


static int sp3d_spi_release(void)
{
	int rc = -EBADF;
	mutex_lock(&sp3d_mut);
	free_irq(MSM_GPIO_TO_INT(106),0);
	sp3d_power_down();
	if(sp3d_ctrl  != NULL){
		kfree(sp3d_ctrl);
		sp3d_ctrl =NULL;
	}
	pr_info("[CAM]sp3d_release completed\n");
	atomic_set(&preview_flag,false);
	atomic_set(&snapshot_flag,false);
	mutex_unlock(&sp3d_mut);
	return rc;
}

static int sp3d_spi_probe(struct spi_device *spi)
{
	/* int rc = 0; */
	pr_info("[CAM]sp3d_spi_probe called!\n");

	sp3d_spi_ctrl =
		kzalloc(sizeof(*sp3d_spi_ctrl), GFP_KERNEL);
	if (!sp3d_spi_ctrl)
		return -ENOMEM;

	sp3d_spi_ctrl->spi = spi;
	spin_lock_init(&sp3d_spi_ctrl->spinlock);

	spi_set_drvdata(spi, sp3d_spi_ctrl);

	pr_info("[CAM]SP3D SPI register successed! \n");
	pr_info("[CAM]master bus_num = %d\n", spi->master->bus_num);
	return 0;

#if 0
probe_failure:
	CDBG("sp3d_spi_probe failed! rc = %d\n", rc);
	return rc;
#endif
}


static struct spi_driver sp3d_spi_driver = {
	.driver = {
		.name =		"sp3d_spi",
		.owner =	THIS_MODULE,
	},
	.probe = sp3d_spi_probe,
	.remove = __exit_p(sp3d_spi_remove),
};

static const char *Sp3dVendor = "Sharp";
static const char *Sp3dNAME = "sp3d";
static const char *Sp3dSize = "5M";
static uint32_t cam3Dmode_value;

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	sprintf(buf, "%s %s %s\n", Sp3dVendor, Sp3dNAME, Sp3dSize);
	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t cam3Dmode_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length = 0;
	length = sprintf(buf, "%d\n", cam3Dmode_value);
	return length;
}

static ssize_t cam3Dmode_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;
	tmp = buf[0] - 0x30; /* only get the first char */

#if 0
	if (strcmp(current->comm,"com.android.camera")!=0){
		pr_info("[CAM]No permission : not camera ap\n");
		return -EINVAL;
	}
#endif

	cam3Dmode_value = tmp;
	pr_info("[CAM]cam3Dmode_value = %d\n", cam3Dmode_value);
	return count;
}

static ssize_t stereo_low_cap_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	struct msm_camera_sensor_info *sdata = sp3d_pdev->dev.platform_data;
	length = sprintf(buf, "%d\n", sdata->stereo_low_cap_limit);
	return length;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static DEVICE_ATTR(cam3Dmode, 0644, cam3Dmode_get, cam3Dmode_set);
static DEVICE_ATTR(stereo_low_cap_limit, 0444, stereo_low_cap_limit_get, NULL);

static struct kobject *android_sp3d;

static int sp3d_sysfs_init(void)
{
	int ret = 0;
	pr_info("[CAM]sp3d:kobject creat and add\n");
	android_sp3d = kobject_create_and_add("android_camera", NULL);
	if (android_sp3d == NULL) {
		pr_info("[CAM]sp3d_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM]sp3d:sysfs_create_file\n");
	ret = sysfs_create_file(android_sp3d, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM]sp3d_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_sp3d);
	}

	ret = sysfs_create_file(android_sp3d, &dev_attr_cam3Dmode.attr);
	if (ret) {
		pr_info("[CAM]sp3d_sysfs_init: sysfs_create_file cam3Dmode failed\n");
		kobject_del(android_sp3d);
	}

	ret = sysfs_create_file(android_sp3d, &dev_attr_stereo_low_cap_limit.attr);
	if (ret) {
		pr_info("[CAM]sp3d_sysfs_init: sysfs_create_file stereo_low_cap_limit failed\n");
		kobject_del(android_sp3d);
	}

	return 0 ;
}


static int sp3d_sensor_probe(struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	pr_info("[CAM]sp3d_sensor_probe\n");

	rc = spi_register_driver(&sp3d_spi_driver);
	if (rc < 0)
		return -EFAULT;

	msm_camio_clk_rate_set(24000000);
	mdelay(300);

	if(!sp3d_spi_ctrl)
	{
		pr_err("[CAM] sp3d_spi_ctrl is NULL!\n");
		goto probe_fail;
	}

	rc = sp3d_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	s->s_init = sp3d_spi_open_init;
	s->s_release = sp3d_spi_release;
	s->s_config  = sp3d_spi_config;
	s->temp_s_config = sp3d_spi_config_temp;
	s->s_camera_type  = BACK_CAMERA_3D;
	sp3d_probe_init_done(info);
	sp3d_sysfs_init();

	return rc;

probe_fail:
	sp3d_probe_init_done(info);
	pr_err("[CAM]sp3d_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}


static int __sp3d_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	sp3d_pdev = pdev;
	pr_info("[CAM]sp3d_probe");
	/*turn on power*/
	rc = sp3d_vreg_enable(sdata);
	sdata->pdata->camera_gpio_on();
	/* switch PCLK and MCLK to Main cam */
	pr_info("[CAM]sp3d: sp3d_sensor_probe: switch clk\n");
	if(sdata->camera_clk_switch != NULL)
		sdata->camera_clk_switch();
	mdelay(5);
	/*probe up*/
	return msm_camera_drv_start(pdev, sp3d_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sp3d_probe,
	.driver = {
		.name = "msm_camera_sp3d",
		.owner = THIS_MODULE,
	},
};

static int __init sp3d_init(void)
{
	pr_info("[CAM]sp3d_init\n");
	return platform_driver_register(&msm_camera_driver);
}

module_init(sp3d_init);

MODULE_DESCRIPTION("Sharp 3D sensor driver");
MODULE_LICENSE("GPL v2");
