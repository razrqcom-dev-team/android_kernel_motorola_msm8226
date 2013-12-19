/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/err.h>
#include <linux/dropbox.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>

#include "mdss_dsi.h"

#define DT_CMD_HDR 6
#define ESD_DROPBOX_MSG "ESD event detected"
#define ESD_TE_DROPBOX_MSG "ESD TE event detected"

/* MDSS_PANEL_ESD_SELFTEST is used to run ESD detection/recovery stress test */
/* #define MDSS_PANEL_ESD_SELFTEST 1 */
#define MDSS_PANEL_ESD_CNT_MAX 3
#define MDSS_PANEL_ESD_TE_TRIGGER (MDSS_PANEL_ESD_CNT_MAX * 2)

/* ESD spec require 10s, select 8s */
#ifdef MDSS_PANEL_ESD_SELFTEST
/*
 * MDSS_PANEL_ESD_SELF_TRIGGER is triggered ESD recovery depending how many
 * times of MDSS_PANEL_ESD_CNT_MAX detection
 */
#define MDSS_PANEL_ESD_SELF_TRIGGER  1

#define MDSS_PANEL_ESD_CHECK_PERIOD     msecs_to_jiffies(200)
#else
/* ESD spec require 10ms, select 8ms */
#define MDSS_PANEL_ESD_CHECK_PERIOD     msecs_to_jiffies(8000)
#endif

#define MDSS_PANEL_DEFAULT_VER 0xffffffffffffffff
#define MDSS_PANEL_UNKNOWN_NAME "unknown"

#define TE_MONITOR_TO  68
#define TE_PULSE_USEC  100

#define PWR_MODE_DISON 0x4

DEFINE_LED_TRIGGER(bl_led_trigger);

static struct mdss_dsi_phy_ctrl phy_params;

void mdss_dsi_panel_lock_mutex(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	mutex_lock(&ctrl->panel_config.panel_mutex);
}

void mdss_dsi_panel_unlock_mutex(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	mutex_unlock(&ctrl->panel_config.panel_mutex);
}

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;

	if (!gpio_is_valid(ctrl->pwm_pmic_gpio)) {
		pr_err("%s: pwm_pmic_gpio=%d Invalid\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ret = gpio_request(ctrl->pwm_pmic_gpio, "disp_pwm");
	if (ret) {
		pr_err("%s: pwm_pmic_gpio=%d request failed\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: lpg_chan=%d pwm request failed", __func__,
				ctrl->pwm_lpg_chan);
		gpio_free(ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
	}
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	ret = pwm_config(ctrl->pwm_bl, duty, ctrl->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_dcs_read(struct mdss_dsi_ctrl_pdata *ctrl,
			char cmd0, char cmd1, fxn call_back)
{
	struct dcs_cmd_req cmdreq;

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = call_back; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: level=%d\n", __func__, level);

	led_pwm1[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

void mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	int i;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (enable) {
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
			gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}

		for (i = 0; i < ctrl_pdata->rst_seq_len; i++) {
			gpio_set_value((ctrl_pdata->rst_gpio),
				ctrl_pdata->rst_seq[i++]);
			if (ctrl_pdata->rst_seq[i])
				usleep_range(ctrl_pdata->rst_seq[i] * 1000,
					ctrl_pdata->rst_seq[i] * 1000);
		}
	} else {
		for (i = 0; i < ctrl_pdata->dis_rst_seq_len; i++) {
			gpio_set_value((ctrl_pdata->rst_gpio),
				ctrl_pdata->dis_rst_seq[i++]);
			if (ctrl_pdata->dis_rst_seq[i])
				usleep_range(ctrl_pdata->dis_rst_seq[i] * 1000,
					ctrl_pdata->dis_rst_seq[i] * 1000);
		}

		if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
	}
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

static u8 power_mode;

static void mdss_dsi_get_pwr_mode_cb(u32 data)
{
	u8 *pdata = (u8 *)data;
	power_mode = *pdata;
}

u32 mdss_dsi_dcs_read(struct mdss_dsi_ctrl_pdata *ctrl,
			char cmd0, char cmd1, fxn call_back);

static int  mdss_dsi_get_pwr_mode(struct mdss_panel_data *pdata, u8 *pwr_mode)
{
	struct mdss_dsi_ctrl_pdata *ctrl;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if (ctrl->panel_config.bare_board == true) {
		*pwr_mode = 0;
		goto end;
	}

	mdss_dsi_dcs_read(ctrl, DCS_CMD_GET_POWER_MODE, 0x00,
						mdss_dsi_get_pwr_mode_cb);
	*pwr_mode = power_mode;
end:
	pr_debug("%s: panel power mode = 0x%x\n", __func__, *pwr_mode);
	return 0;
}

static int mdss_dsi_panel_regulator_init(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct platform_device *pdev;

	pr_debug("%s: is called\n", __func__);

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl) {
		pr_err("%s: Invalid ctrl\n", __func__);
		return -EINVAL;
	}

	pdev = ctrl->pdev;
	if (!pdev) {
		pr_err("%s: Invalid pdev\n", __func__);
		return -EINVAL;
	}

	if (ctrl->panel_vregs.num_vreg > 0) {
		ret = msm_dss_config_vreg(&pdev->dev,
					ctrl->panel_vregs.vreg_config,
					ctrl->panel_vregs.num_vreg, 1);
		if (ret)
			pr_err("%s:fail to init regs. ret=%d\n", __func__, ret);
	}

	return ret;
}

static int mdss_dsi_panel_regulator_on(struct mdss_panel_data *pdata,
						int enable)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl;
	static bool is_reg_inited;

	pr_debug("%s(%d) is called\n", __func__, enable);

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if (is_reg_inited == false) {
		ret = mdss_dsi_panel_regulator_init(pdata);
		if (ret)
			goto error;
		else
			is_reg_inited = true;
	}

	if (enable) {
		if (ctrl->panel_vregs.num_vreg > 0) {
			ret = msm_dss_enable_vreg(ctrl->panel_vregs.vreg_config,
						ctrl->panel_vregs.num_vreg, 1);
			if (ret)
				pr_err("%s:Failed to enable regulators.rc=%d\n",
							__func__, ret);
		} else
			pr_err("%s(%d): No panel regulators in the list\n",
							__func__, enable);
	} else {

		if (ctrl->panel_vregs.num_vreg > 0) {
			ret = msm_dss_enable_vreg(ctrl->panel_vregs.vreg_config,
						ctrl->panel_vregs.num_vreg, 0);
			if (ret)
				pr_err("%s: Failed to disable regs.rc=%d\n",
						__func__, ret);
		} else
			pr_err("%s(%d): No panel regulators in the list\n",
							__func__, enable);
	}

error:
	return ret;
}

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata);
static int mdss_dsi_panel_off(struct mdss_panel_data *pdata);

static irqreturn_t mdss_panel_esd_te_irq_handler(int irq, void *ctrl_ptr)
{
	struct mdss_dsi_ctrl_pdata *ctrl =
				(struct mdss_dsi_ctrl_pdata *)ctrl_ptr;
	pr_debug("%s: is called\n", __func__);

	complete(&ctrl->panel_esd_data.te_detected);
	disable_irq_nosync(ctrl->panel_esd_data.te_irq);
	return IRQ_HANDLED;
}

static int mdss_panel_esd_recovery_start(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	pr_warning("MDSS PANEL: ESD recovering is started\n");

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if (!pdata->panel_info.panel_power_on) {
		pr_warning("%s:%d Panel already off.\n", __func__, __LINE__);
		 mdss_dsi_panel_unlock_mutex(pdata);
		goto end;
	}
	ctrl->panel_esd_data.esd_recovery_run = true;
	mdss_dsi_panel_off(pdata);
	msleep(200);
	mdss_dsi_panel_on(pdata);
	ctrl->panel_esd_data.esd_recovery_run = false;
end:
	pr_warning("MDSS PANEL: ESD recovering is done\n");

	return 0;
}

static void mdss_panel_esd_te_monitor(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;
	static bool dropbox_sent;

	INIT_COMPLETION(ctrl->panel_esd_data.te_detected);

	enable_irq(ctrl->panel_esd_data.te_irq);

	ret = wait_for_completion_timeout(&ctrl->panel_esd_data.te_detected,
					msecs_to_jiffies(TE_MONITOR_TO));
	if (ret == 0) {
		pr_warning("%s: No TE sig for %d usec. Trigger ESD recovery\n",
				__func__, TE_MONITOR_TO);
		mdss_panel_esd_recovery_start(&ctrl->panel_data);
		if (!dropbox_sent) {
			dropbox_queue_event_text("display_issue",
						ESD_TE_DROPBOX_MSG,
						strlen(ESD_TE_DROPBOX_MSG));
			dropbox_sent = true;
		}
	}

}

#ifdef MDSS_PANEL_ESD_SELF_TRIGGER
static char disp_off[2] = {0x28, 0x0};  /* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc dispoff_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(disp_off)}, disp_off
};

static void mdss_dsi_panel_dispoff_dcs(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dcs_cmd_req cmdreq;

	pr_debug("%s+:\n", __func__);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dispoff_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}
#endif

static void mdss_panel_esd_work(struct work_struct *work)
{
	u8 pwr_mode = 0;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	static bool dropbox_sent;
	struct mdss_panel_esd_pdata *esd_data;
#ifdef MDSS_PANEL_ESD_SELF_TRIGGER
	static int esd_count;
	static int esd_trigger_cnt;
#endif

	ctrl = container_of(work, struct mdss_dsi_ctrl_pdata, esd_work.work);
	if (ctrl == NULL) {
		pr_err("%s: invalid ctrl\n", __func__);
		goto end;
	} else
		pr_debug("%s: ctrl = %p\n", __func__, ctrl);

	esd_data = &ctrl->panel_esd_data;
	mdss_dsi_panel_lock_mutex(&ctrl->panel_data);
	if (!ctrl->panel_data.panel_info.panel_power_on) {
		mdss_dsi_panel_unlock_mutex(&ctrl->panel_data);
		return;
	}
	mdss_dsi_get_pwr_mode(&ctrl->panel_data, &pwr_mode);

	pr_debug("%s: is called. pwr_mode = 0x%x\n", __func__, pwr_mode);

#ifdef MDSS_PANEL_ESD_SELF_TRIGGER
	if (esd_count > MDSS_PANEL_ESD_TE_TRIGGER)
		esd_count = 0;
	esd_count++;
	if (esd_count == MDSS_PANEL_ESD_CNT_MAX) {
		pwr_mode = 0x00;
		pr_info("%s(%d): start to ESD test\n", __func__,
							esd_trigger_cnt++);
	}
#endif
	if ((pwr_mode & esd_data->esd_pwr_mode_chk) !=
					esd_data->esd_pwr_mode_chk) {
		pr_warning("%s: ESD detected pwr_mode =0x%x expected = 0x%x\n",
					__func__, pwr_mode,
					esd_data->esd_pwr_mode_chk);
		mdss_panel_esd_recovery_start(&ctrl->panel_data);
		if (!dropbox_sent) {
			dropbox_queue_event_text("display_issue",
						ESD_DROPBOX_MSG,
						strlen(ESD_DROPBOX_MSG));
			dropbox_sent = true;
		}
	} else {
		pr_debug("%s. esd_det_mode=%d\n",
				__func__, esd_data->esd_detect_mode);
		if (esd_data->esd_detect_mode == ESD_TE_DET &&
			(esd_data->esd_pwr_mode_chk & PWR_MODE_DISON)) {
#ifdef MDSS_PANEL_ESD_SELF_TRIGGER
			if (esd_count == MDSS_PANEL_ESD_TE_TRIGGER) {
				pr_warning("%s(%d): start to ESD test. "
					"turn off display\n", __func__,
					esd_trigger_cnt++);
				mdss_dsi_panel_dispoff_dcs(ctrl);
				esd_count = 0;
			}
#endif
			mdss_panel_esd_te_monitor(ctrl);
		}
	}
end:
	if (ctrl->panel_data.panel_info.panel_power_on)
		queue_delayed_work(esd_data->esd_wq, &ctrl->esd_work,
						MDSS_PANEL_ESD_CHECK_PERIOD);
	mdss_dsi_panel_unlock_mutex(&ctrl->panel_data);
}

static int mdss_dsi_panel_esd_init(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;
	struct mdss_panel_esd_pdata *esd_data = &ctrl->panel_esd_data;

	INIT_DELAYED_WORK_DEFERRABLE(&ctrl->esd_work, mdss_panel_esd_work);

	if (esd_data->esd_detect_mode == ESD_TE_DET) {
		init_completion(&esd_data->te_detected);
		esd_data->te_irq = gpio_to_irq(ctrl->disp_te_gpio);
		ret = request_irq(esd_data->te_irq,
				mdss_panel_esd_te_irq_handler,
				IRQF_TRIGGER_RISING, "mdss_panel_esd_te", ctrl);
		if (ret < 0) {
			pr_err("%s: unable to request IRQ %d\n",
						__func__, ctrl->disp_te_gpio);
			return -EAGAIN;
		}
	}

	return 0;
}

static int mdss_dsi_panel_esd(struct mdss_panel_data *pdata)
{
	int ret;
	static bool esd_work_queue_init;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	pr_debug("%s is called.\n", __func__);
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (ctrl->panel_config.esd_enable &&
			ctrl->panel_esd_data.esd_detection_run == false &&
			ctrl->panel_esd_data.esd_recovery_run == false) {
		if (esd_work_queue_init == false) {
			ret = mdss_dsi_panel_esd_init(ctrl);
			if (ret)
				return ret;
			esd_work_queue_init = true;
		}

		queue_delayed_work(ctrl->panel_esd_data.esd_wq, &ctrl->esd_work,
						MDSS_PANEL_ESD_CHECK_PERIOD);

		ctrl->panel_esd_data.esd_detection_run = true;

		pr_debug("%s: start the  ESD work queue\n", __func__);
	}

	return 0;
}

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	u8 pwr_mode = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_info("%s+: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mdss_dsi_panel_regulator_on(pdata, 1);

	mdss_dsi_panel_reset(pdata, 1);

	if (ctrl->panel_config.bare_board == true) {
		pr_warning("%s: This is bare_board configuration\n", __func__);
		goto end;
	}

	if (ctrl->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);

	mdss_dsi_get_pwr_mode(pdata, &pwr_mode);
	/* validate screen is actually on */
	if ((pwr_mode & 0x04) != 0x04) {
		pr_err("%s: Display failure: DISON (0x04) bit not set\n",
			__func__);
		dropbox_queue_event_empty("display_issue");
	}

	mdss_dsi_panel_esd(pdata);
end:
	pr_info("%s-. Pwr_mode(0x0A) = 0x%x\n", __func__, pwr_mode);

	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_info("%s+: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi  = &pdata->panel_info.mipi;

	if (ctrl->panel_config.bare_board == true)
		goto disable_regs;

	if (ctrl->panel_config.esd_enable &&
			ctrl->panel_esd_data.esd_detection_run == true &&
			ctrl->panel_esd_data.esd_recovery_run == false) {
		cancel_delayed_work(&ctrl->esd_work);
		ctrl->panel_esd_data.esd_detection_run = false;
		pr_debug("%s: cancel the  ESD work queue\n", __func__);
	}

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

	if (ctrl->off_cmds_1.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds_1);

disable_regs:
	mdss_dsi_panel_reset(pdata, 0);
	mdss_dsi_panel_regulator_on(pdata, 0);

	pr_info("%s-:\n", __func__);

	return 0;
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			return -ENOMEM;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		kfree(buf);
		return -ENOMEM;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		return -ENOMEM;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	data = of_get_property(np, link_key, NULL);
	if (!strncmp(data, "dsi_hs_mode", 11))
		pcmds->link_state = DSI_HS_MODE;
	else
		pcmds->link_state = DSI_LP_MODE;

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;
}

static int mdss_panel_parse_panel_reg_dt(struct platform_device *pdev,
				struct mdss_panel_common_pdata *panel_data)
{
	int rc;

	pr_debug("%s is called\n", __func__);

	/* Parse the regulator information */
	rc = mdss_dsi_get_dt_vreg_data(&pdev->dev, &panel_data->vregs);
	if (rc) {
		pr_err("%s: failed to get vreg data from dt. rc=%d\n",
								__func__, rc);
		goto error_vreg;
	}

	return 0;

error_vreg:
	mdss_dsi_put_dt_vreg_data(&pdev->dev, &panel_data->vregs);

	return rc;
}

static int mdss_panel_dt_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}

static int mdss_panel_parse_reset_seq(struct device_node *np, const char *name,
				int rst_seq[MDSS_DSI_RST_SEQ_LEN], int *rst_len)
{
	int num_u32 = 0;
	int rc;
	struct property *pp;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];

	*rst_len = 0;

	pp = of_find_property(np, name, &num_u32);
	num_u32 /= sizeof(u32);
	if (!pp || num_u32 == 0 || num_u32 > MDSS_DSI_RST_SEQ_LEN ||
		num_u32 % 2)
		pr_err("%s:%d, error reading property %s, num_u32 = %d\n",
			__func__, __LINE__, name, num_u32);
	else {
		rc = of_property_read_u32_array(np, name, tmp, num_u32);
		if (rc)
			pr_err("%s:%d, unable to read %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			memcpy(rst_seq, tmp, num_u32 * sizeof(u32));
			*rst_len = num_u32;
		}
	}
	return 0;
}

static int mdss_panel_parse_panel_config_dt(struct platform_device *pdev,
				struct mdss_panel_common_pdata *panel_data)
{
	struct device_node *np;
	const char *pname;
	u32 panel_ver;

	if (panel_data->panel_config.is_panel_config_loaded)
		return 0;

	np = of_find_node_by_path("/chosen");
	panel_data->panel_config.esd_disable_bl =
				of_property_read_bool(np, "mmi,esd");

	if (of_property_read_bool(np, "mmi,bare_board") == true &&
		of_property_read_bool(np, "mmi,factory-cable") == true)
			panel_data->panel_config.bare_board = true;

	panel_data->panel_config.panel_ver = MDSS_PANEL_DEFAULT_VER;
	of_property_read_u64(np, "mmi,panel_ver",
					&panel_data->panel_config.panel_ver);

	pname = of_get_property(np, "mmi,panel_name", NULL);
	if (!pname || strlen(pname) == 0) {
		pr_warning("Failed to get mmi,panel_name\n");
		strlcpy(panel_data->panel_config.panel_name,
			MDSS_PANEL_UNKNOWN_NAME,
			sizeof(panel_data->panel_config.panel_name));
	} else
		strlcpy(panel_data->panel_config.panel_name, pname,
			sizeof(panel_data->panel_config.panel_name));

	pr_debug("%s: esd_disable_bl=%d bare_board_bl=%d, factory_cable=%d bare_board=%d\n",
		__func__, panel_data->panel_config.esd_disable_bl,
		of_property_read_bool(np, "mmi,bare_board"),
		of_property_read_bool(np, "mmi,factory-cable"),
		panel_data->panel_config.bare_board);

	panel_ver = (u32)panel_data->panel_config.panel_ver;
	pr_info("BL: panel=%s, manufacture_id(0xDA)= 0x%x controller_ver(0xDB)= 0x%x controller_drv_ver(0XDC)= 0x%x, full=0x%016llx\n",
		panel_data->panel_config.panel_name,
		panel_ver & 0xff, (panel_ver & 0xff00) >> 8,
		(panel_ver & 0xff0000) >> 16,
		panel_data->panel_config.panel_ver);

	panel_data->panel_config.is_panel_config_loaded = true;

	of_node_put(np);

	return 0;
}

static int mdss_dsi_panel_reg_read(struct mdss_panel_data *pdata, u8 reg,
				int mode, size_t size, u8 *buffer)
{
	int old_tx_mode;
	int ret;
	struct dcs_cmd_req cmdreq;
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct dsi_cmd_desc reg_read_cmd = {
		.dchdr.dtype = DTYPE_DCS_READ,
		.dchdr.last = 1,
		.dchdr.vc = 0,
		.dchdr.ack = 1,
		.dchdr.wait = 1,
		.dchdr.dlen = 1,
		.payload = &reg
	};

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (size > MDSS_DSI_LEN) {
		pr_warning("%s: size %d, max rx length is %d.\n", __func__,
				size, MDSS_DSI_LEN);
		return -EINVAL;
	}

	pr_debug("%s: Reading %d bytes from 0x%02x\n", __func__, size, reg);

	old_tx_mode = mdss_get_tx_power_mode(pdata);
	if (mode != old_tx_mode)
		mdss_set_tx_power_mode(mode, pdata);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &reg_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = size;
	cmdreq.cb = NULL; /* call back */
	cmdreq.rdata = kmalloc(MDSS_DSI_LEN, GFP_KERNEL);
	if (!cmdreq.rdata) {
		ret = -ENOMEM;
		goto err1;
	}
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	if (ret <= 0) {
		pr_err("%s: Error reading %d bytes from reg 0x%02x error.\n",
			__func__, size, (unsigned int) reg);
		ret = -EFAULT;
	} else {
		memcpy(buffer, cmdreq.rdata, size);
		ret = 0;
	}
	kfree(cmdreq.rdata);

err1:
	if (mode != old_tx_mode)
		mdss_set_tx_power_mode(old_tx_mode, pdata);
	return ret;
}

static int mdss_dsi_panel_reg_write(struct mdss_panel_data *pdata, int mode,
				size_t size, u8 *buffer)
{
	int old_tx_mode;
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct dsi_cmd_desc reg_write_cmd = {
		.dchdr.dtype = DTYPE_DCS_LWRITE,
		.dchdr.last = 1,
		.dchdr.vc = 0,
		.dchdr.ack = 0,
		.dchdr.wait = 0,
		.dchdr.dlen = size,
		.payload = buffer
	};

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: Writing %d bytes to 0x%02x\n", __func__, size, buffer[0]);

	old_tx_mode = mdss_get_tx_power_mode(pdata);
	if (mode != old_tx_mode)
		mdss_set_tx_power_mode(mode, pdata);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &reg_write_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (ret <= 0) {
		pr_err("%s: Failed writing %d bytes to 0x%02x.\n",
			__func__, size, buffer[0]);
		ret = -EFAULT;
	} else
		ret = 0;

	if (mode != old_tx_mode)
		mdss_set_tx_power_mode(old_tx_mode, pdata);

	return ret;
}

static int mdss_panel_parse_dt(struct platform_device *pdev,
			      struct mdss_panel_common_pdata *panel_data)
{
	struct device_node *np = pdev->dev.of_node;
	u32 tmp;
	int rc, i, len;
	const char *data;
	static const char *pdest;

	pr_debug("%s is called\n", __func__);
	rc = mdss_panel_parse_panel_reg_dt(pdev, panel_data);
	if (rc)
		return rc;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);

	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	panel_data->panel_info.xres = (!rc ? tmp : 640);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	panel_data->panel_info.yres = (!rc ? tmp : 480);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	panel_data->panel_info.physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	panel_data->panel_info.physical_height = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	panel_data->panel_info.lcdc.xres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc.xres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	panel_data->panel_info.lcdc.yres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc.yres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	panel_data->panel_info.bpp = (!rc ? tmp : 24);
	panel_data->panel_info.mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		panel_data->panel_info.mipi.mode = DSI_CMD_MODE;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-pixel-packing", &tmp);
	tmp = (!rc ? tmp : 0);
	rc = mdss_panel_dt_get_dst_fmt(panel_data->panel_info.bpp,
		panel_data->panel_info.mipi.mode, tmp,
		&(panel_data->panel_info.mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		panel_data->panel_info.mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(pdev->dev.of_node,
		"qcom,mdss-dsi-panel-destination", NULL);
	if (strlen(pdest) != 9) {
		pr_err("%s: Unknown pdest specified\n", __func__);
		return -EINVAL;
	}
	if (!strncmp(pdest, "display_1", 9))
		panel_data->panel_info.pdest = DISPLAY_1;
	else if (!strncmp(pdest, "display_2", 9))
		panel_data->panel_info.pdest = DISPLAY_2;
	else {
		pr_debug("%s: pdest not specified. Set Default\n",
							__func__);
		panel_data->panel_info.pdest = DISPLAY_1;
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	panel_data->panel_info.lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	panel_data->panel_info.lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	panel_data->panel_info.lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	panel_data->panel_info.lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	panel_data->panel_info.lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	panel_data->panel_info.lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	panel_data->panel_info.lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	panel_data->panel_info.lcdc.underflow_clr = (!rc ? tmp : 0xff);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	panel_data->panel_info.lcdc.border_clr = (!rc ? tmp : 0);
	panel_data->panel_info.bklt_ctrl = UNKNOWN_CTRL;

	/* lcdc_tune is the same as lcdc by default, but can be overridden */
	memcpy(&panel_data->panel_info.lcdc_tune, &panel_data->panel_info.lcdc,
		sizeof(struct lcd_panel_info));

	rc = of_property_read_u32(np, "qcom,mdss-dsi-tune-h-front-porch", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc_tune.h_front_porch = tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-tune-h-back-porch", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc_tune.h_back_porch = tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-tune-h-pulse-width", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc_tune.h_pulse_width = tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-tune-h-sync-skew", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc_tune.hsync_skew = tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-tune-v-back-porch", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc_tune.v_back_porch = tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-tune-v-front-porch", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc_tune.v_front_porch = tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-tune-v-pulse-width", &tmp);
	if (!rc)
		panel_data->panel_info.lcdc_tune.v_pulse_width = tmp;

	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			panel_data->panel_info.bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			panel_data->panel_info.bklt_ctrl = BL_PWM;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			panel_data->panel_info.pwm_period = tmp;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-bank-select", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, dsi lpg channel\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			panel_data->panel_info.pwm_lpg_chan = tmp;
			tmp = of_get_named_gpio(np,
				"qcom,mdss-dsi-pwm-gpio", 0);
			panel_data->panel_info.pwm_pmic_gpio = tmp;
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			panel_data->panel_info.bklt_ctrl = BL_DCS_CMD;
		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	panel_data->panel_info.bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	panel_data->panel_info.bl_max = (!rc ? tmp : 255);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	panel_data->panel_info.mipi.interleave_mode = (!rc ? tmp : 0);

	panel_data->panel_info.mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	panel_data->panel_info.mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	panel_data->panel_info.mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	panel_data->panel_info.mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	panel_data->panel_info.mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	panel_data->panel_info.mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	panel_data->panel_info.mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	panel_data->panel_info.mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-traffic-mode", &tmp);
	panel_data->panel_info.mipi.traffic_mode =
			(!rc ? tmp : DSI_NON_BURST_SYNCH_PULSE);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	panel_data->panel_info.mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-v-sync-continue-lines", &tmp);
	panel_data->panel_info.mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-v-sync-rd-ptr-irq-line", &tmp);
	panel_data->panel_info.mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	panel_data->panel_info.mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	panel_data->panel_info.mipi.vc = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-color-order", &tmp);
	panel_data->panel_info.mipi.rgb_swap = (!rc ? tmp : DSI_RGB_SWAP_RGB);
	panel_data->panel_info.mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	panel_data->panel_info.mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	panel_data->panel_info.mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	panel_data->panel_info.mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-lane-map", &tmp);
	panel_data->panel_info.mipi.dlane_swap = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	panel_data->panel_info.mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	panel_data->panel_info.mipi.t_clk_post = (!rc ? tmp : 0x03);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	panel_data->panel_info.mipi.stream = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-mdp-trigger", &tmp);
	panel_data->panel_info.mipi.mdp_trigger =
			(!rc ? tmp : DSI_CMD_TRIGGER_SW);
	if (panel_data->panel_info.mipi.mdp_trigger > 6) {
		pr_err("%s:%d, Invalid mdp trigger. Forcing to sw trigger",
						 __func__, __LINE__);
		panel_data->panel_info.mipi.mdp_trigger =
					DSI_CMD_TRIGGER_SW;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-dma-trigger", &tmp);
	panel_data->panel_info.mipi.dma_trigger =
			(!rc ? tmp : DSI_CMD_TRIGGER_SW);
	if (panel_data->panel_info.mipi.dma_trigger > 6) {
		pr_err("%s:%d, Invalid dma trigger. Forcing to sw trigger",
						 __func__, __LINE__);
		panel_data->panel_info.mipi.dma_trigger =
					DSI_CMD_TRIGGER_SW;
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-frame-rate", &tmp);
	panel_data->panel_info.mipi.frame_rate = (!rc ? tmp : 60);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clock-rate", &tmp);
	panel_data->panel_info.clk_rate = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		phy_params.timing[i] = data[i];

	panel_data->panel_info.mipi.dsi_phy_db = &phy_params;

	mdss_dsi_parse_fbc_params(np, &panel_data->panel_info);

	mdss_dsi_parse_dcs_cmds(np, &panel_data->on_cmds,
		"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &panel_data->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");
	mdss_dsi_parse_dcs_cmds(np, &panel_data->off_cmds_1,
		"qcom,mdss-dsi-off-1-command",
		"qcom,mdss-dsi-off-command-state");

	if (panel_data->panel_config.bare_board == true ||
				panel_data->panel_config.esd_disable_bl == true)
		panel_data->panel_config.esd_enable = false;
	else
		panel_data->panel_config.esd_enable = !of_property_read_bool(np,
					"qcom,panel-esd-detect-disable");

	of_property_read_u32(np, "qcom,panel-esd-power-mode-chk",
				&panel_data->panel_esd_data.esd_pwr_mode_chk);

	of_property_read_u32(np, "qcom,mdss-dsi-esd-det-mode",
				&panel_data->panel_esd_data.esd_detect_mode);


	pr_debug("%s: esd_enable=%d esd-detect-disable=%d "
		"esd_pwr_mode_chk=0x%x esd_detect_mode=%d\n", __func__,
		panel_data->panel_config.esd_enable,
		of_property_read_bool(np, "qcom,panel-esd-detect-disable"),
		panel_data->panel_esd_data.esd_pwr_mode_chk,
		panel_data->panel_esd_data.esd_detect_mode);

	mdss_panel_parse_reset_seq(np, "qcom,panel-en-reset-sequence",
				panel_data->rst_seq,
				&panel_data->rst_seq_len);

	mdss_panel_parse_reset_seq(np, "qcom,panel-dis-reset-sequence",
				panel_data->dis_rst_seq,
				&panel_data->dis_rst_seq_len);

	return 0;

error:
	return -EINVAL;
}

static int mdss_dsi_match_chosen_panel(struct device_node *np,
					struct mdss_panel_common_pdata *pdata)
{
	static bool panel_found;
	const char *panel_name;
	u64 panel_ver_min;
	u64 panel_ver_max;

	if (panel_found)
		return -ENODEV;

	if (of_property_read_u64(np, "mmi,panel_ver_min", &panel_ver_min)) {
		pr_err("%s: Failed to read panel version min\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_u64(np, "mmi,panel_ver_max", &panel_ver_max)) {
		pr_err("%s: Failed to read panel version max\n", __func__);
		return -ENODEV;
	}

	panel_name = of_get_property(np, "mmi,panel_name", NULL);
	if (!panel_name) {
		pr_err("%s: Panel name not set\n", __func__);
		return -ENODEV;
	}

	/* If we are in bare board mode or if the panel name isn't set, use first default
	version panel we find */
	if ((pdata->panel_config.bare_board ||
		!strcmp(pdata->panel_config.panel_name,
			MDSS_PANEL_UNKNOWN_NAME)) &&
		panel_ver_max == MDSS_PANEL_DEFAULT_VER)
		pr_warning("%s: In bare board mode, using panel %s [0x%016llx, 0x%016llx]\n",
			__func__, panel_name, panel_ver_min, panel_ver_max);
	else if (!strcmp(panel_name, pdata->panel_config.panel_name) &&
			pdata->panel_config.panel_ver >= panel_ver_min &&
			pdata->panel_config.panel_ver <= panel_ver_max)
		pr_info("%s: Found match for panel %s, version 0x%016llx [0x%016llx, 0x%016llx]\n",
			__func__, panel_name, pdata->panel_config.panel_ver,
			panel_ver_min, panel_ver_max);
	else {
		pr_debug("%s: Panel %s [0x%016llx, 0x%016llx] did not match\n",
			__func__, panel_name, panel_ver_min, panel_ver_max);
		return -ENODEV;
	}

	panel_found = 1;
	return 0;
}

static int __devinit mdss_dsi_panel_probe(struct platform_device *pdev)
{
	int rc = 0;
	static struct mdss_panel_common_pdata vendor_pdata;
	static const char *panel_name;
	pr_debug("%s:%d, debug info id=%d", __func__, __LINE__, pdev->id);
	if (!pdev->dev.of_node)
		return -ENODEV;

	rc = mdss_panel_parse_panel_config_dt(pdev, &vendor_pdata);
	if (rc)
		return rc;

	rc = mdss_dsi_match_chosen_panel(pdev->dev.of_node, &vendor_pdata);
	if (rc)
		return rc;

	panel_name = of_get_property(pdev->dev.of_node,
		"qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name)
		pr_info("%s:%d, panel name not specified\n",
						__func__, __LINE__);
	else
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);

	rc = mdss_panel_parse_dt(pdev, &vendor_pdata);
	if (rc)
		goto cleanup;

	vendor_pdata.on = mdss_dsi_panel_on;
	vendor_pdata.off = mdss_dsi_panel_off;
	vendor_pdata.esd = mdss_dsi_panel_esd;
	vendor_pdata.bl_fnc = mdss_dsi_panel_bl_ctrl;
	vendor_pdata.lock_mutex = mdss_dsi_panel_lock_mutex;
	vendor_pdata.unlock_mutex = mdss_dsi_panel_unlock_mutex;
	vendor_pdata.reg_read = mdss_dsi_panel_reg_read;
	vendor_pdata.reg_write = mdss_dsi_panel_reg_write;

	rc = dsi_panel_device_register(pdev, &vendor_pdata);
	if (rc)
		goto cleanup;

	return 0;
cleanup:
	led_trigger_unregister_simple(bl_led_trigger);

	return rc;
}

static const struct of_device_id mdss_dsi_panel_match[] = {
	{.compatible = "qcom,mdss-dsi-panel"},
	{}
};

static struct platform_driver this_driver = {
	.probe  = mdss_dsi_panel_probe,
	.driver = {
		.name   = "dsi_panel",
		.of_match_table = mdss_dsi_panel_match,
	},
};

static int __init mdss_dsi_panel_init(void)
{
	return platform_driver_register(&this_driver);
}
module_init(mdss_dsi_panel_init);
