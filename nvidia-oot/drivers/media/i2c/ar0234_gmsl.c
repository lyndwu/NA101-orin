/*
 * cam_gmsl.c - cam_gmsl sensor driver
 *
 * Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//#define DEBUG

#include <nvidia/conftest.h>

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/max9295.h>
#include <media/max9296.h>

#include <linux/firmware.h>
#include <linux/regmap.h>

#include <media/tegracam_core.h>
#include "cam_gmsl_mode_tbls.h"
#include "linux/delay.h"

#define CHANNEL_N 13
#define MAX_RADIAL_COEFFICIENTS         6
#define MAX_TANGENTIAL_COEFFICIENTS     2
#define MAX_FISHEYE_COEFFICIENTS        6

#define CAM_GMSL_MIN_GAIN         (1)
#define CAM_GMSL_MAX_GAIN         (8)
#define CAM_GMSL_MAX_GAIN_REG     (0x40)
#define CAM_GMSL_DEFAULT_FRAME_LENGTH    (1224)
#define CAM_GMSL_COARSE_TIME_SHS1_ADDR    0x3012
#define CAM_GMSL_ANALOG_GAIN    0x3060

const struct of_device_id cam_gmsl_of_match[] = {
	{.compatible = "nvidia,ar0234_gmsl",},
	{ },
};
MODULE_DEVICE_TABLE(of, cam_gmsl_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

// Coefficients as per distortion model (wide FOV) being used
typedef struct
{
	// Radial coefficients count
	u32 coeff_count;
	// Radial coefficients
	float k[MAX_FISHEYE_COEFFICIENTS];
	// 0 -> equidistant, 1 -> equisolid, 2 -> orthographic, 3 -> stereographic
	u32 mapping_type;
} fisheye_lens_distortion_coeff;

// Coefficients as per distortion model being used
typedef struct
{
	// Radial coefficients count
	u32 radial_coeff_count;
	// Radial coefficients
	float k[MAX_RADIAL_COEFFICIENTS];
	// Tangential coefficients count
	u32 tangential_coeff_count;
	// Tangential coefficients
	float p[MAX_TANGENTIAL_COEFFICIENTS];
} polynomial_lens_distortion_coeff;

/*
 * Stereo Eeprom Data
 */
typedef struct
{
	// Width and height of image in pixels
	u32 width, height;
	// Focal length in pixels
	float fx, fy;
	float skew;
	// Principal point (optical center) in pixels
	float cx, cy;
	/*
	 * Structure for distortion coefficients as per the model being used
	 * 0: pinhole, assuming polynomial distortion
	 * 1: fisheye, assuming fisheye distortion)
	 * 2: ocam (omini-directional)
	 */
	u32 distortion_type;
	union distortion_coefficients {
		polynomial_lens_distortion_coeff poly;
		fisheye_lens_distortion_coeff fisheye;
	} dist_coeff;
} camera_intrinsics;

/*
 * Extrinsic parameters shared by camera and IMU.
 * All rotation + translation with respect to the same reference point
 */
typedef struct
{
	/*
	 * Rotation parameter expressed in Rodrigues notation
	 * angle = sqrt(rx^2+ry^2+rz^2)
	 * unit axis = [rx,ry,rz]/angle
	 */
	float rx, ry, rz;
	// Translation parameter from one camera to another parameter
	float tx, ty, tz;
} camera_extrinsics;

typedef struct
{
	// 3D vector to add to accelerometer readings
	float linear_acceleration_bias[3];
	// 3D vector to add to gyroscope readings
	float angular_velocity_bias[3];
	// gravity acceleration
	float gravity_acceleration[3];
	// Extrinsic structure for IMU device
	camera_extrinsics extr;
} imu_params;

struct cam_gmsl {
	struct i2c_client	*i2c_client;
	const struct i2c_device_id *id;
	unsigned short def_addr;		/* def - NOTE: 7bit	*/
	struct v4l2_subdev	*subdev;
	u32	frame_length;
	struct regmap *regmap;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
	u32 	dser_ser_init;
	int cam_powen_gpio;
	int dser_reset_gpio;
};

// static const struct regmap_config sensor_regmap_config = {
// 	.reg_bits = 16,
// 	.val_bits = 8,
// 	.cache_type = REGCACHE_RBTREE,
// };


static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
	.val_format_endian = REGMAP_ENDIAN_BIG
};

static const struct regmap_config my_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
	.val_format_endian = REGMAP_ENDIAN_BIG
};


struct cam_gmsl_frmfmt_map {
    const char *name;
    const struct camera_common_frmfmt *table;
    u32 num;
};

static const struct cam_gmsl_frmfmt_map cam_gmsl_frmfmt_maps[] = {
	{
		.name  = "cam_gmsl_30fps_1920x1536_frmfmt",
		.table = cam_gmsl_30fps_1920x1536_frmfmt,
		.num   = ARRAY_SIZE(cam_gmsl_30fps_1920x1536_frmfmt),
	},
	{
		.name  = "cam_gmsl_60fps_1920x1536_frmfmt",
		.table = cam_gmsl_60fps_1920x1536_frmfmt,
		.num   = ARRAY_SIZE(cam_gmsl_60fps_1920x1536_frmfmt),
	},
	{
		.name  = "cam_gmsl_60fps_2592x1944_frmfmt",
		.table = cam_gmsl_60fps_2592x1944_frmfmt,
		.num   = ARRAY_SIZE(cam_gmsl_60fps_2592x1944_frmfmt),
	},
	{
		.name  = "cam_gmsl_50fps_2560x1440_frmfmt",
		.table = cam_gmsl_50fps_2560x1440_frmfmt,
		.num   = ARRAY_SIZE(cam_gmsl_50fps_2560x1440_frmfmt),
	},
 	{
		.name  = "cam_gmsl_30fps_1920x1280_frmfmt",
		.table = cam_gmsl_30fps_1920x1280_frmfmt,
		.num   = ARRAY_SIZE(cam_gmsl_30fps_1920x1280_frmfmt),
	},
	{
		.name  = "cam_gmsl_30fps_1920x1200_frmfmt",
		.table = cam_gmsl_30fps_1920x1200_frmfmt,
		.num   = ARRAY_SIZE(cam_gmsl_30fps_1920x1200_frmfmt),
	},
};

/*
static int cam_gmsl_read_reg(struct cam_gmsl *priv,
		u8 slave_addr, u16 addr, u8 *val)
{
	struct camera_common_data *s_data = priv->s_data;
	struct i2c_client *i2c_client = priv->i2c_client;
	struct device *dev = s_data->dev;
	int err;
	u32 reg_val = 0;
	int addr_bak;

	addr_bak = i2c_client->addr;

	i2c_client->addr = slave_addr;
	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;
	if (err)
	{
		dev_err(dev, "%s:i2c read failed, addr:0x%02x, reg:0x%04x = 0x%02x\n",
				__func__, slave_addr, addr, *val);
		i2c_client->addr = addr_bak;
	}
	else
	{
		dev_dbg(dev, "%s:i2c read success, addr:0x%02x, reg:0x%04x = 0x%02x\n",
				__func__, slave_addr, addr, *val);
		i2c_client->addr = addr_bak;
		return 0;
	}

	return err;
}
*/
static int cam_gmsl_write_reg(struct cam_gmsl *priv,
		u8 slave_addr, u16 addr, u8 val)
{
	struct camera_common_data *s_data = priv->s_data;
	struct i2c_client *i2c_client = priv->i2c_client;
	struct device *dev = s_data->dev;
	int err;
	int addr_bak;

	addr_bak = i2c_client->addr;

	i2c_client->addr = slave_addr;
	err = regmap_write(s_data->regmap, addr, val);
	if (err)
	{
		dev_dbg(dev, "%s:i2c write failed, addr:0x%02x, reg:0x%04x = 0x%02x\n",
				__func__, slave_addr, addr, val);
		i2c_client->addr = addr_bak;
	}
	else
	{
		dev_dbg(dev, "%s:i2c write success, addr:0x%02x, reg:0x%04x = 0x%02x\n",
				__func__, slave_addr, addr, val);
		i2c_client->addr = addr_bak;
		return 0;
	}

	return err;
}

static int cam_gmsl_write_table(struct cam_gmsl *priv,
		const struct index_reg_8 table[])
{
	struct tegracam_device *tc_dev = priv->tc_dev;
	struct device *dev = tc_dev->dev;
	int i = 0;
	int ret = 0;
	int retry = 0;

	dev_dbg(dev, "%s++\n", __func__);
	while (table[i].source != 0x00)
	{
		if (table[i].source == 0x06)
		{
			if (table[i].addr == CAM_GMSL_TABLE_WAIT_MS)
			{
				dev_dbg(dev, "%s: sleep %d\n", __func__, table[i].val);
				msleep(table[i].val);
				i++;
				continue;
			}
		}
		else
		{
			retry = 0;
retry_serdes:
			ret = cam_gmsl_write_reg(priv, table[i].source/2, table[i].addr, (u8)table[i].val);
			if (ret && (table[i].addr != 0x0000))
			{
				retry--;
				if (retry > 0) {
					dev_warn(dev, "cam_gmsl_write_reg: try %d\n", retry);
					msleep(4);
					goto retry_serdes;
				}
			}
		}
		i++;
	}
	return 0;
}

static int cam_gmsl_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	usleep_range(10000, 20000);
	pw->state = SWITCH_ON;

	return 0;
}

static int cam_gmsl_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s:\n", __func__);
	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return err;
	}

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static int cam_gmsl_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);
	mclk_name = pdata->mclk_name ?
		pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clcok %s",
					parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	pw->state = SWITCH_OFF;

	return err;
}

static int cam_gmsl_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static int cam_gmsl_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct device *dev = tc_dev->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);
	if (err) {
		dev_err(dev, "%s: error\n", __func__);
		return err;
	}

	return 0;
}

static int cam_gmsl_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);
	if (err) {
		dev_err(dev, "%s: error\n", __func__);
		return err;
	}

	return 0;
}

static int cam_gmsl_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);
	if (err) {
		dev_err(dev, "%s: error\n", __func__);
		return err;
	}

	return 0;
}

static int cam_gmsl_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);
	if (err) {
		dev_err(dev, "%s: error\n", __func__);
		return err;
	}

	return 0;
}

static struct tegracam_ctrl_ops cam_gmsl_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = cam_gmsl_set_gain,
	.set_exposure = cam_gmsl_set_exposure,
	.set_frame_rate = cam_gmsl_set_frame_rate,
	.set_group_hold = cam_gmsl_set_group_hold,
};

static struct camera_common_pdata *cam_gmsl_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	dev_dbg(dev, "%s++\n", __func__);
	match = of_match_device(cam_gmsl_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev, sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
			&board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	return board_priv_pdata;
}

static int cam_gmsl_set_mode(struct tegracam_device *tc_dev)
{
//	struct cam_gmsl *priv = (struct cam_gmsl *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	const struct of_device_id *match;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);
	match = of_match_device(cam_gmsl_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	dev_dbg(dev, "%s: mode index:%d\n", __func__,s_data->mode_prop_idx);
//	err = cam_gmsl_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	return 0;
}

static int cam_gmsl_start_streaming(struct tegracam_device *tc_dev)
{
	struct cam_gmsl *priv = (struct cam_gmsl *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err = 0;

	// struct i2c_client *i2c_client = priv->i2c_client;

	if(0)
	{
		dev_dbg(dev, "%s++\n", __func__);
		err = cam_gmsl_write_table(priv, mode_table[CAM_GMSL_MODE_START_STREAM]);
		if (err) {
			dev_err(dev, "%s: error\n", __func__);
			return err;
		}
	}
	


	dev_dbg(dev, "%s++\n", __func__);	

	printk("cam_gmsl_start_streaming Starting stream.22\n");

	/* 向从机地址 0x3c 写寄存器 0x601a = 0x0380 (stream on) */
	{
		// int addr_bak = i2c_client->addr;
		// i2c_client->addr = 0x3c;
		err = regmap_write(priv->regmap, 0x601a, 0x0380);
		if (err)
			dev_err(dev, "Failed to write stream on to 0x3c: %d\n", err);
		else
			dev_info(dev, "Write stream on (0x601a=0x0380) to 0x3c success\n");
		// i2c_client->addr = addr_bak;
	}
	return 0;
}

static int cam_gmsl_stop_streaming(struct tegracam_device *tc_dev)
{
	struct cam_gmsl *priv = (struct cam_gmsl *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err = 0;
	// struct i2c_client *i2c_client = priv->i2c_client;

	dev_dbg(dev, "%s++\n", __func__);
//	err = cam_gmsl_write_table(priv, mode_table[CAM_GMSL_MODE_STOP_STREAM]);
	if (err) {
		dev_err(dev, "%s: error\n", __func__);
		return err;
	}


	/* 向从机地址 0x3c 写寄存器 0x601a = 0x0380 (stream on) */
	{
		// int addr_bak = i2c_client->addr;
		// i2c_client->addr = 0x3c;
		err = regmap_write(priv->regmap, 0x601a, 0x0180);
		if (err)
			dev_err(dev, "Failed to write stream on to 0x3c: %d\n", err);
		else
			dev_info(dev, "Write stream on (0x601a=0x0380) to 0x3c success\n");
		// i2c_client->addr = addr_bak;
	}

	return 0;
}

static struct camera_common_sensor_ops cam_gmsl_common_ops = {
	.numfrmfmts = ARRAY_SIZE(cam_gmsl_frmfmt),
	.frmfmt_table = cam_gmsl_frmfmt,
	.power_on = cam_gmsl_power_on,
	.power_off = cam_gmsl_power_off,
	.parse_dt = cam_gmsl_parse_dt,
	.power_get = cam_gmsl_power_get,
	.power_put = cam_gmsl_power_put,
	.set_mode = cam_gmsl_set_mode,
	.start_streaming = cam_gmsl_start_streaming,
	.stop_streaming = cam_gmsl_stop_streaming,
};

static int cam_gmsl_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_dbg(&client->dev, "%s:\n", __func__);
	if (err) {
		dev_err(&client->dev, "%s: error\n", __func__);
		return err;
	}

	return 0;
}

static const struct v4l2_subdev_internal_ops cam_gmsl_subdev_internal_ops = {
	.open = cam_gmsl_open,
};

static int cam_gmsl_board_setup(struct cam_gmsl *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);
	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev, "Error %d turning on mclk\n", err);
		goto error;
	}

	err = cam_gmsl_power_on(s_data);
	if (err) {
		dev_err(dev, "Error %d during power on sensor\n", err);
		goto error;
	}

	return 0;
error:
	cam_gmsl_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
}

static int cam_gmsl_update_fmt_by_dt(struct device *dev, struct device_node *node)
{
	int i;
       	const char *v4l2_frmfmt;
       
	if (of_property_read_string(node, "v4l2_frmfmt", &v4l2_frmfmt)) {
	       	dev_info(dev, "%s: no v4l2_frmfmt in dt, keep default frmfmt\n", __func__);
	       	return 0;
       	}
	//添加打印v4l2_frmfmt 

	for (i = 0; i < ARRAY_SIZE(cam_gmsl_frmfmt_maps); i++) {
		if (!strcmp(v4l2_frmfmt, cam_gmsl_frmfmt_maps[i].name)) {
			cam_gmsl_common_ops.frmfmt_table = cam_gmsl_frmfmt_maps[i].table;
			cam_gmsl_common_ops.numfrmfmts  = cam_gmsl_frmfmt_maps[i].num;
			dev_info(dev, "%s: use frmfmt table %s\n", __func__, v4l2_frmfmt);
			return 0;
		}
	}
	
	dev_warn(dev, "%s: unknown v4l2_frmfmt=%s, keep default frmfmt\n", __func__, v4l2_frmfmt);
	return 0;
}

#define AP1302_START_ADDR 0X8000
#define AR0234_ISP_BULK_SIZE 4096
#define AR0234_ISP_FILE_NAME "ar0234_cam_fw.bin"

static int iic_write_array(struct i2c_client *client, u8 slaveaddr, u16 regaddr,
                           const u16 *data, u16 length)
{
    struct i2c_msg msg[1];
    u8 *buf;
    int err;
    int i;

    if (data == NULL || length == 0) {
        dev_err(&client->dev, "%s: data is NULL or length is 0\n", __func__);
        return -1;
    }

    buf = kzalloc(2 + (length * 2), GFP_KERNEL);
    if (!buf) {
        dev_err(&client->dev, "%s: failed to allocate memory\n", __func__);
        return -ENOMEM;
    }

    msg[0].addr = slaveaddr;
    msg[0].flags = 0;
    msg[0].buf = buf;
    msg[0].len = 2 + (length * 2);

    buf[0] = regaddr >> 8;
    buf[1] = regaddr & 0xff;

    for (i = 0; i < length; i++) {
        buf[2 + (i * 2)] = data[i] >> 8;
        buf[3 + (i * 2)] = data[i] & 0xff;
    }

    err = i2c_transfer(client->adapter, msg, 1);
    usleep_range(100, 110);

    kfree(buf);

    if (err != 1) {
        dev_err(&client->dev, "%s: i2c write array failed, slave=0x%02x / length=%d\n",
                __func__, slaveaddr, length);
        return -1;
    }

    return 0;
}

typedef struct {
	u16* data;
	u32 datalength;

}ap1302_config_t;

static int lh_ar0234_isp_firmware_load(struct cam_gmsl *priv)
{
    struct i2c_client *client = priv->i2c_client;
    const struct firmware *fw = NULL;
    unsigned char *isp_fw_buf = NULL;
    ap1302_config_t isp_bulk_config;
    u32 isp_written_bytes = 0;
    u32 isp_remain_words = 0;
    int err = 0;

	// client->addr = 0x3c;
	printk("run lh_ar0234_isp_firmware_load");
    err = request_firmware(&fw, AR0234_ISP_FILE_NAME, &client->dev);
    if (err < 0) {
        dev_err(&client->dev, "Failed to request firmware2222[%s]: %d\n",
                AR0234_ISP_FILE_NAME, err);
        goto isp_fw_load_fail;
    }

    if (fw->size == 0 || (fw->size % 2) != 0) {
        dev_err(&client->dev, "Invalid firmware size: %zu (must be non-zero and even)\n",
                fw->size);
        err = -EINVAL;
        goto isp_fw_load_fail;
    }

    isp_fw_buf = kmalloc(fw->size, GFP_KERNEL);
    if (!isp_fw_buf) {
        dev_err(&client->dev, "Failed to allocate memory for firmware buffer\n");
        err = -ENOMEM;
        goto isp_fw_load_fail;
    }

    memcpy(isp_fw_buf, fw->data, fw->size);

    isp_bulk_config.data = (u16 *)isp_fw_buf;
    isp_remain_words = fw->size / 2;
    isp_bulk_config.datalength = (isp_remain_words >= AR0234_ISP_BULK_SIZE) ?
                                  AR0234_ISP_BULK_SIZE : isp_remain_words;

    while (isp_bulk_config.datalength != 0) {
        err = iic_write_array(client, client->addr, AP1302_START_ADDR,
                              isp_bulk_config.data, isp_bulk_config.datalength);
        if (err) {
            dev_err(&client->dev, "ap1302 device register_array write reg failed.\n");
            goto isp_fw_load_fail;
        }

        isp_written_bytes += isp_bulk_config.datalength * 2;
        msleep(10);

        if (isp_written_bytes < fw->size) {
            isp_remain_words = (fw->size - isp_written_bytes) / 2;
            isp_bulk_config.data = (u16 *)(isp_fw_buf + isp_written_bytes);

            if (isp_remain_words >= AR0234_ISP_BULK_SIZE) {
                isp_bulk_config.datalength = AR0234_ISP_BULK_SIZE;
            } else {
                isp_bulk_config.datalength = isp_remain_words;
            }
        } else {
            isp_bulk_config.data = NULL;
            isp_bulk_config.datalength = 0;
        }
    }

    dev_info(&client->dev, "Successfully loaded ISP firmware (%zu bytes)\n", fw->size);

isp_fw_load_fail:
    if (fw) {
        release_firmware(fw);
        fw = NULL;
    }
    if (isp_fw_buf) {
        kfree(isp_fw_buf);
        isp_fw_buf = NULL;
    }

    return err;
}


void my_camer_setup(struct cam_gmsl *priv);
void my_camer_setup(struct cam_gmsl *priv)
{
	struct i2c_client *client = priv->i2c_client;
	//读 0x0000 寄存器，
	u16 reg_addr = 0x0000;
	u32 reg_value;
	s32 ret;  // 注意：返回值类型是 s32，因为要返回读取的数据或错误码

	/* 切换到 I2C 从机地址 0x3c 进行读写操作 */
	//client->addr = 0x3c;
	dev_info(&client->dev, "new I2C slave addr: new=0x%02x\n", client->addr);

	
	/* 直接读取 16 位寄存器值 */
	//ret = regmap_raw_read(priv->regmap, reg_addr, &reg_value, 2);
	ret = regmap_read(priv->regmap, reg_addr, &reg_value);
	if (ret) 
	{
    	dev_err(&client->dev, "Failed to read register 0x%04x: %d\n", reg_addr, ret);
		return;
	} else {
		dev_info(&client->dev, "Read register 0x%04x = 0x%04x\n", reg_addr, reg_value);
	}

	//写寄存器（数组形式）- 使用 regmap_write 直接写入
	{
		int i;
		static const struct {
			u16 addr;
			u16 val;
		} init_regs[] = {
			{ 0x60a8, 0x0000 },
			{ 0xf05a, 0x0014 },
			{ 0x6024, 0x0018 },
			{ 0x6026, 0x0000 },
			{ 0x6034, 0x027c },
			{ 0x6036, 0x0000 },
			{ 0x2030, 0x0010 },
			{ 0x2030, 0x0010 }, /*test*/
		};
		for (i = 0; i < ARRAY_SIZE(init_regs); i++) {
			ret = regmap_write(priv->regmap, init_regs[i].addr, init_regs[i].val);
			if (ret)
				dev_err(&client->dev, "Failed to write register 0x%04x via regmap_write\n",
					init_regs[i].addr);
			else
				dev_info(&client->dev, "Write register 0x%04x = 0x%04x\n",
					 init_regs[i].addr, init_regs[i].val);
		}

		msleep(200);
	}

	//调用 lh_ar0234_isp_firmware_load 加载 ISP 固件
	ret = lh_ar0234_isp_firmware_load(priv);
	if (ret)
		dev_err(&client->dev, "Failed to load ISP firmware\n");
	else
		dev_info(&client->dev, "ISP firmware loaded successfully\n");

	//写寄存器 0x6002 = 0xffff，然后轮询等待确认（超时 3s）
	{
		int poll_cnt;
		u32 poll_val;

		ret = regmap_write(priv->regmap, 0x6002, 0xffff);
		if (ret)
			dev_err(&client->dev, "Failed to write register 0x6002\n");
		else
			dev_info(&client->dev, "Write register 0x6002 = 0xffff\n");

		/* 轮询 0x6002，每 100ms 读一次，最多 30 次（3s 超时） */
		for (poll_cnt = 0; poll_cnt < 30; poll_cnt++) {
			msleep(100);
			ret = regmap_read(priv->regmap, 0x6002, &poll_val);
			if (ret) {
				dev_err(&client->dev, "Failed to poll register 0x6002\n");
				break;
			}
			if (poll_val == 0xffff) {
				dev_info(&client->dev, "Register 0x6002 confirmed = 0x%04x (poll %d)\n",
					 poll_val, poll_cnt + 1);
				break;
			}
			dev_dbg(&client->dev, "Poll 0x6002 = 0x%04x, retry...\n", poll_val);
		}

		if (poll_cnt >= 30)
			dev_err(&client->dev, "Timeout waiting for register 0x6002 to become 0xffff\n");
	}

	//轮询 0x6134，等待值变为 0xffff（超时 3s，只读不写）
	{
		int poll_cnt;
		u32 poll_val;

		for (poll_cnt = 0; poll_cnt < 30; poll_cnt++) {
			msleep(100);
			ret = regmap_read(priv->regmap, 0x6134, &poll_val);
			if (ret) {
				dev_err(&client->dev, "Failed to poll register 0x6134\n");
				break;
			}
			if (poll_val == 0xffff) {
				dev_info(&client->dev, "Register 0x6134 confirmed = 0x%04x (poll %d)\n",
					 poll_val, poll_cnt + 1);
				break;
			}
			dev_dbg(&client->dev, "Poll 0x6134 = 0x%04x, retry...\n", poll_val);
		}

		if (poll_cnt >= 30)
			dev_err(&client->dev, "Timeout waiting for register 0x6134 to become 0xffff\n");
	}

	//写寄存器二 - 使用 regmap_write 直接写入
	{
		int i;
		static const struct {
			u16 addr;
			u16 val;
		} init_regs2[] = {
			{ 0x2030, 0x0034 },
			{ 0x2010, 0x0000 },
			{ 0x600c, 0x1405 },
			{ 0x2020, 0x3c00 },
			{ 0x2022, 0x0300 },
			{ 0x2024, 0x0000 },
			{ 0x2026, 0x1f40 },
			{ 0x5034, 0x0600 },
			{ 0x2028, 0x0000 },
			/*{ 0x202a, 0x3e80 },*/
			{ 0x1186, 0x038a },
			{ 0x1186, 0x0381 },
			{ 0x100c, 0x0003 },
			{ 0x601a, 0x0140 },
		};

		for (i = 0; i < ARRAY_SIZE(init_regs2); i++) {
			ret = regmap_write(priv->regmap, init_regs2[i].addr, init_regs2[i].val);
			if (ret)
				dev_err(&client->dev, "Failed to write register 0x%04x via regmap_write\n",
					init_regs2[i].addr);
			else
				dev_info(&client->dev, "Write register 0x%04x = 0x%04x\n",
					 init_regs2[i].addr, init_regs2[i].val);
		}
	}

	/* 恢复原始 I2C 从机地址 */
	//dev_info(&client->dev, "Restore I2C slave addr: 0x%02x\n", addr_bak);
	//client->addr = addr_bak;

}


/* ========== 合并配置表: 寄存器写 + 延时 统一为一个数组 ========== */
/* slave_addr == 0 表示延时项, val 为毫秒数 */
struct serdes_cfg {
	u8  slave_addr;	/* 0=延时, 否则 I2C 地址 */
	u8  reg_hi;
	u8  reg_lo;
	u16 val;	/* 写值, 或延时 ms */
};

#define _WR(a, h, l, v)	{ (a), (h), (l), (v) }
#define _DLY(ms)	{ 0, 0, 0, (ms) }

static const struct serdes_cfg serdes_init_seq[] = {
	/* ---- 解串器初始化 ---- */
	_WR(0x29, 0x00, 0x13, 0x40),	/* Device Reset */
	_DLY(300),
	_WR(0x29, 0x04, 0x0b, 0x00),	/* disabled csi out */
	_WR(0x29, 0x00, 0x06, 0xff),	/* enabled all port and config GMSL2 */
	_DLY(300),

	/* ---- 外部触发初始化 ---- */
	_WR(0x29, 0x04, 0xa0, 0x08),
	_WR(0x29, 0x04, 0xaf, 0x9f),
	_DLY(300),
	_WR(0x29, 0x03, 0x06, 0x83),
	_WR(0x29, 0x03, 0x07, 0xA7),
	_WR(0x29, 0x03, 0x3d, 0xA7),
	_WR(0x29, 0x03, 0x74, 0xA7),
	_WR(0x29, 0x03, 0xaa, 0xA7),



	/* ---- LINKA pipe 配置 ---- */
	_WR(0x29, 0x00, 0xf0, 0x20),
	_WR(0x29, 0x00, 0xf1, 0x64),
	_WR(0x29, 0x00, 0xf2, 0xA8),
	_WR(0x29, 0x00, 0xf3, 0xEC),
	_WR(0x29, 0x00, 0xf4, 0xFF),

	/* ---- YUV422 8bit pipe 0-3 ---- */
	_WR(0x29, 0x09, 0x0b, 0x07),	/* SRC_0 -> DES_0 */
	_WR(0x29, 0x09, 0x2d, 0x15),	/* DES_0 -> CSI2 control */
	_WR(0x29, 0x09, 0x0d, 0x1e),	/* vc */
	_WR(0x29, 0x09, 0x0e, 0x1e),	/* vc */
	_WR(0x29, 0x09, 0x0f, 0x00),	/* vc */
	_WR(0x29, 0x09, 0x10, 0x00),	/* vc */
	_WR(0x29, 0x09, 0x11, 0x01),	/* vc */
	_WR(0x29, 0x09, 0x12, 0x01),	/* vc */

	_WR(0x29, 0x09, 0x4b, 0x07),
	_WR(0x29, 0x09, 0x6d, 0x15),
	_WR(0x29, 0x09, 0x4d, 0x1e),
	_WR(0x29, 0x09, 0x4e, 0x5e),
	_WR(0x29, 0x09, 0x4f, 0x00),
	_WR(0x29, 0x09, 0x50, 0x40),
	_WR(0x29, 0x09, 0x51, 0x01),
	_WR(0x29, 0x09, 0x52, 0x41),

	_WR(0x29, 0x09, 0x8b, 0x07),
	_WR(0x29, 0x09, 0xad, 0x15),
	_WR(0x29, 0x09, 0x8d, 0x1e),
	_WR(0x29, 0x09, 0x8e, 0x9e),
	_WR(0x29, 0x09, 0x8f, 0x00),
	_WR(0x29, 0x09, 0x90, 0x80),
	_WR(0x29, 0x09, 0x91, 0x01),
	_WR(0x29, 0x09, 0x92, 0x81),
	
	_WR(0x29, 0x09, 0xcb, 0x07),
	_WR(0x29, 0x09, 0xed, 0x15),
	_WR(0x29, 0x09, 0xcd, 0x1e),
	_WR(0x29, 0x09, 0xce, 0xde),
	_WR(0x29, 0x09, 0xcf, 0x00),
	_WR(0x29, 0x09, 0xd0, 0xc0),
	_WR(0x29, 0x09, 0xd1, 0x01),
	_WR(0x29, 0x09, 0xd2, 0xc1),

	/* ---- YUV422 8bit pipe 4-7 ---- */
	_WR(0x29, 0x0a, 0x0b, 0x07),
	_WR(0x29, 0x0a, 0x2d, 0x2a),
	_WR(0x29, 0x0a, 0x0d, 0x1e),
	_WR(0x29, 0x0a, 0x0e, 0x1e),
	_WR(0x29, 0x0a, 0x0f, 0x00),
	_WR(0x29, 0x0a, 0x10, 0x00),
	_WR(0x29, 0x0a, 0x11, 0x01),
	_WR(0x29, 0x0a, 0x12, 0x01),
	_WR(0x29, 0x0a, 0x4b, 0x07),
	_WR(0x29, 0x0a, 0x6d, 0x2a),
	_WR(0x29, 0x0a, 0x4d, 0x1e),
	_WR(0x29, 0x0a, 0x4e, 0x5e),
	_WR(0x29, 0x0a, 0x4f, 0x00),
	_WR(0x29, 0x0a, 0x50, 0x40),
	_WR(0x29, 0x0a, 0x51, 0x01),
	_WR(0x29, 0x0a, 0x52, 0x41),
	_WR(0x29, 0x0a, 0x8b, 0x07),
	_WR(0x29, 0x0a, 0xad, 0x2a),
	_WR(0x29, 0x0a, 0x8d, 0x1e),
	_WR(0x29, 0x0a, 0x8e, 0x9e),
	_WR(0x29, 0x0a, 0x8f, 0x00),
	_WR(0x29, 0x0a, 0x90, 0x80),
	_WR(0x29, 0x0a, 0x91, 0x01),
	_WR(0x29, 0x0a, 0x92, 0x81),
	_WR(0x29, 0x0a, 0xcb, 0x07),
	_WR(0x29, 0x0a, 0xed, 0x2a),
	_WR(0x29, 0x0a, 0xcd, 0x1e),
	_WR(0x29, 0x0a, 0xce, 0xde),
	_WR(0x29, 0x0a, 0xcf, 0x00),
	_WR(0x29, 0x0a, 0xd0, 0xc0),
	_WR(0x29, 0x0a, 0xd1, 0x01),
	_WR(0x29, 0x0a, 0xd2, 0xc1),
	_DLY(300),

	/* ---- MIPI PHY 设置 ---- */
	_WR(0x29, 0x08, 0xa0, 0x04),	/* 2x4lane */
	_WR(0x29, 0x08, 0xa3, 0xe4),	/* lane map */
	_WR(0x29, 0x08, 0xa4, 0xe4),	/* lane map */
	_WR(0x29, 0x09, 0x03, 0x80),	/* 4lane Dphy */
	_WR(0x29, 0x09, 0x43, 0x80),	/* 4lane Dphy */
	_WR(0x29, 0x09, 0x83, 0x80),	/* 4lane Dphy */
	_WR(0x29, 0x09, 0xc3, 0x80),	/* 4lane Dphy */
	_WR(0x29, 0x09, 0x0a, 0xc0),	/* 4lane Dphy */
	_WR(0x29, 0x09, 0x4a, 0xc0),	/* 4lane Dphy */
	_WR(0x29, 0x09, 0x8a, 0xc0),	/* 4lane Dphy */
	_WR(0x29, 0x09, 0xca, 0xc0),	/* 4lane Dphy */
	_WR(0x29, 0x08, 0xa2, 0xf0),	/* enable all mipi */
	_WR(0x29, 0x1C, 0x00, 0xF4),	/* hold DPLL reset */
	_WR(0x29, 0x1D, 0x00, 0xF4),
	_WR(0x29, 0x1E, 0x00, 0xF4),
	_WR(0x29, 0x1F, 0x00, 0xF4),
	_WR(0x29, 0x04, 0x15, 0x2F),	/* set data rate */
	_WR(0x29, 0x04, 0x18, 0x2F),
	_WR(0x29, 0x04, 0x1b, 0x2F),
	_WR(0x29, 0x04, 0x1e, 0x2F),
	_WR(0x29, 0x1C, 0x00, 0xF5),	/* release DPLL reset */
	_WR(0x29, 0x1D, 0x00, 0xF5),
	_WR(0x29, 0x1E, 0x00, 0xF5),
	_WR(0x29, 0x1F, 0x00, 0xF5),

	_WR(0x41, 0x00, 0x10, 0x80),	/* Device Reset */
	_WR(0x42, 0x00, 0x10, 0x80),	/* Device Reset */
	_WR(0x43, 0x00, 0x10, 0x80),	/* Device Reset */
	_WR(0x44, 0x00, 0x10, 0x80),	/* Device Reset */
	_DLY(300),

	/* ======== linkA ======== */
	_WR(0x29, 0x00, 0x06, 0xf1),	/* select link A */
	_DLY(300),
	_WR(0x40, 0x00, 0x10, 0x21),	/* power down */
	_DLY(300),
	_WR(0x40, 0x00, 0x02, 0x43),	/* Enable all pipes */
	_WR(0x40, 0x03, 0x30, 0x06),
	_WR(0x40, 0x03, 0x31, 0x33),
	_WR(0x40, 0x03, 0x32, 0x4E),
	_WR(0x40, 0x03, 0x33, 0xE4),
	_WR(0x40, 0x03, 0x11, 0x41),
	_WR(0x40, 0x03, 0x14, 0x5E),	/* Pipe X pulls (DT 0x1E) */
	_WR(0x40, 0x03, 0x18, 0x5E),	/* Pipe Z pulls (DT 0x1E) */
	_WR(0x40, 0x02, 0xC7, 0x84),
	_WR(0x40, 0x02, 0xC9, 0x47),
	_WR(0x40, 0x02, 0xE8, 0x84),
	_WR(0x40, 0x02, 0xEA, 0x47),
	_WR(0x40, 0x02, 0xBE, 0x00),
	_DLY(300),
	_WR(0x40, 0x02, 0xBE, 0x10),
	_WR(0x40, 0x02, 0xD9, 0x00),
	_DLY(300),
	_WR(0x40, 0x02, 0xD9, 0x10),
	_WR(0x40, 0x00, 0x02, 0x53),
	_WR(0x40, 0x00, 0x42, 0x16),	/* 0x0b << 1 */
	_WR(0x40, 0x00, 0x43, 0x78),	/* 0x3c << 1 */
	_WR(0x40, 0x00, 0x44, 0x18),	/* 0x0c << 1 */
	_WR(0x40, 0x00, 0x45, 0x7A),	/* 0x3d << 1 */
	_WR(0x40, 0x00, 0x00, 0x82),	/* change addr to 0x41 */

	/* ======== linkB ======== */
	_WR(0x29, 0x00, 0x06, 0xf2),	/* select link B */
	_DLY(300),
	_WR(0x40, 0x00, 0x10, 0x21),
	_DLY(300),
	_WR(0x40, 0x00, 0x02, 0x43),
	_WR(0x40, 0x03, 0x30, 0x06),
	_WR(0x40, 0x03, 0x31, 0x33),
	_WR(0x40, 0x03, 0x32, 0x4E),
	_WR(0x40, 0x03, 0x33, 0xE4),
	_WR(0x40, 0x03, 0x11, 0x41),
	_WR(0x40, 0x03, 0x14, 0x5E),
	_WR(0x40, 0x03, 0x18, 0x5E),
	_WR(0x40, 0x02, 0xC7, 0x84),
	_WR(0x40, 0x02, 0xC9, 0x47),
	_WR(0x40, 0x02, 0xE8, 0x84),
	_WR(0x40, 0x02, 0xEA, 0x47),
	_WR(0x40, 0x02, 0xBE, 0x00),
	_DLY(300),
	_WR(0x40, 0x02, 0xBE, 0x10),
	_WR(0x40, 0x02, 0xD9, 0x00),
	_DLY(300),
	_WR(0x40, 0x02, 0xD9, 0x10),
	_WR(0x40, 0x00, 0x02, 0x53),
	_WR(0x40, 0x00, 0x42, 0x1a),	/* 0x0d << 1 */
	_WR(0x40, 0x00, 0x43, 0x78),	/* 0x3c << 1 */
	_WR(0x40, 0x00, 0x44, 0x1c),	/* 0x0e << 1 */
	_WR(0x40, 0x00, 0x45, 0x7A),	/* 0x3d << 1 */
	_WR(0x40, 0x00, 0x00, 0x84),	/* change addr to 0x42 */

	/* ======== linkC ======== */
	_WR(0x29, 0x00, 0x06, 0xf4),	/* select link C */
	_DLY(300),
	_WR(0x40, 0x00, 0x10, 0x21),
	_DLY(300),
	_WR(0x40, 0x00, 0x02, 0x43),
	_WR(0x40, 0x03, 0x30, 0x06),
	_WR(0x40, 0x03, 0x31, 0x33),
	_WR(0x40, 0x03, 0x32, 0x4E),
	_WR(0x40, 0x03, 0x33, 0xE4),
	_WR(0x40, 0x03, 0x11, 0x41),
	_WR(0x40, 0x03, 0x14, 0x5E),
	_WR(0x40, 0x03, 0x18, 0x5E),
	_WR(0x40, 0x02, 0xC7, 0x84),
	_WR(0x40, 0x02, 0xC9, 0x47),
	_WR(0x40, 0x02, 0xE8, 0x84),
	_WR(0x40, 0x02, 0xEA, 0x47),
	_WR(0x40, 0x02, 0xBE, 0x00),
	_DLY(300),
	_WR(0x40, 0x02, 0xBE, 0x10),
	_WR(0x40, 0x02, 0xD9, 0x00),
	_DLY(300),
	_WR(0x40, 0x02, 0xD9, 0x10),
	_WR(0x40, 0x00, 0x02, 0x53),
	_WR(0x40, 0x00, 0x42, 0x36),	/* 0x1b << 1 */
	_WR(0x40, 0x00, 0x43, 0x78),	/* 0x3c << 1 */
	_WR(0x40, 0x00, 0x44, 0x38),	/* 0x1c << 1 */
	_WR(0x40, 0x00, 0x45, 0x7A),	/* 0x3d << 1 */
	_WR(0x40, 0x00, 0x00, 0x86),	/* change addr to 0x43 */

	/* ======== linkD ======== */
	_WR(0x29, 0x00, 0x06, 0xf8),	/* select link D */
	_DLY(300),
	_WR(0x40, 0x00, 0x10, 0x21),
	_DLY(300),
	_WR(0x40, 0x00, 0x02, 0x43),
	_WR(0x40, 0x03, 0x30, 0x06),
	_WR(0x40, 0x03, 0x31, 0x33),
	_WR(0x40, 0x03, 0x32, 0x4E),
	_WR(0x40, 0x03, 0x33, 0xE4),
	_WR(0x40, 0x03, 0x11, 0x41),
	_WR(0x40, 0x03, 0x14, 0x5E),
	_WR(0x40, 0x03, 0x18, 0x5E),
	_WR(0x40, 0x02, 0xC7, 0x84),
	_WR(0x40, 0x02, 0xC9, 0x47),
	_WR(0x40, 0x02, 0xE8, 0x84),
	_WR(0x40, 0x02, 0xEA, 0x47),
	_WR(0x40, 0x02, 0xBE, 0x00),
	_DLY(300),
	_WR(0x40, 0x02, 0xBE, 0x10),
	_WR(0x40, 0x02, 0xD9, 0x00),
	_DLY(300),
	_WR(0x40, 0x02, 0xD9, 0x10),
	_WR(0x40, 0x00, 0x02, 0x53),
	_WR(0x40, 0x00, 0x42, 0x3a),	/* 0x1d << 1 */
	_WR(0x40, 0x00, 0x43, 0x78),	/* 0x3c << 1 */
	_WR(0x40, 0x00, 0x44, 0x3c),	/* 0x1e << 1 */
	_WR(0x40, 0x00, 0x45, 0x7A),	/* 0x3d << 1 */
	_WR(0x40, 0x00, 0x00, 0x88),	/* change addr to 0x44 */

	/* ---- 最终设置 ---- */
	_DLY(300),
	_WR(0x29, 0x00, 0x06, 0xff),	/* enable all links */
	_WR(0x29, 0x00, 0x18, 0x0f),
	_WR(0x29, 0x04, 0x0b, 0x02),	/* MIPI CSI out */
	_WR(0x29, 0x08, 0xa0, 0x84),
};


void my_set_up_ser_dser(struct cam_gmsl *priv);
void my_set_up_ser_dser(struct cam_gmsl *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct i2c_adapter *adapter = client->adapter;
	struct device *dev = priv->s_data->dev;
	static struct i2c_adapter *last_adapter;
	static bool done;
	int i;

	/* 同一 adapter 仅执行一次 */
	if (done && last_adapter == adapter)
		return;

	done = false;

	for (i = 0; i < ARRAY_SIZE(serdes_init_seq); i++) {
		const struct serdes_cfg *cfg = &serdes_init_seq[i];

		if (cfg->slave_addr == 0) {
			msleep(cfg->val);
		} else {
			u8 buf[3] = { cfg->reg_hi, cfg->reg_lo, (u8)cfg->val };
			struct i2c_msg msg = {
				.addr = cfg->slave_addr,
				.flags = 0,
				.len   = 3,
				.buf   = buf,
			};
			if (i2c_transfer(adapter, &msg, 1) != 1)
				dev_err(dev, "cfg[%d] i2c-%u wr 0x%02x [0x%02x%02x]=0x%02x failed\n",
					i, adapter->nr, cfg->slave_addr,
					cfg->reg_hi, cfg->reg_lo, cfg->val);
			else
				dev_info(dev, "cfg[%d] i2c-%u wr 0x%02x [0x%02x%02x]=0x%02x ok\n",
					i, adapter->nr, cfg->slave_addr,
					cfg->reg_hi, cfg->reg_lo, cfg->val);
		}
	}

	last_adapter = adapter;
	done = true;
}

uint32_t count = 0;
static int cam_gmsl_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct tegracam_device *tc_dev;
	struct cam_gmsl *priv;
	int err;
//	u8 link;

	dev_info(dev, "probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct cam_gmsl), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}
	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	dev_info(&client->dev, "probe I2C slave addr: new=0x%02x\n", client->addr);

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ar0234_gmsl", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
  cam_gmsl_update_fmt_by_dt(dev, node);
	tc_dev->sensor_ops = &cam_gmsl_common_ops;
	tc_dev->v4l2sd_internal_ops = &cam_gmsl_subdev_internal_ops;
	tc_dev->tcctrl_ops = &cam_gmsl_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
				&my_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}


	err = cam_gmsl_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	//SRC_A_1
	my_set_up_ser_dser(priv);
	msleep(500);
	my_camer_setup(priv);
/*
        err = of_property_read_u32(node, "dser-ser-init", &priv->dser_ser_init);
        if (priv->dser_ser_init)
        {
                cam_gmsl_write_table(priv, mode_table[MAX96712_INIT]);
        }
        switch(client->addr){
                case 0x0b:
                        cam_gmsl_write_reg(priv,0x29,0x0006,0xf1);
                        usleep_range(500000, 1000000);
                        err = cam_gmsl_read_reg(priv, 0x40, 0x0000, &check);
                        if(err){
                                dev_err(dev, "camera : %x not found \n",client->addr);
                                goto done;
                        }
                        cam_gmsl_write_table(priv, mode_table[MAX9295_INIT]);
                        cam_gmsl_write_reg(priv,0x40,0x0000,0x82);//wrete addr 0x41
                        break;
                case 0x0d:
                        cam_gmsl_write_reg(priv,0x29,0x0006,0xf2);
                        usleep_range(500000, 1000000);
                        err = cam_gmsl_read_reg(priv, 0x40, 0x0000, &check);
                        if(err){
                                dev_err(dev, "camera : %x not found \n",client->addr);
                                goto done;
                        }
                        cam_gmsl_write_table(priv, mode_table[MAX9295_INIT]);
                        cam_gmsl_write_reg(priv,0x40,0x0000,0x84);//wrete addr 0x42
                        break;
                case 0x1b:
                        cam_gmsl_write_reg(priv,0x29,0x0006,0xf4);
                        usleep_range(500000, 1000000);
                        err = cam_gmsl_read_reg(priv, 0x40, 0x0000, &check);
                        if(err){
                                dev_err(dev, "camera : %x not found \n",client->addr);
                                goto done;
                        }
                        cam_gmsl_write_table(priv, mode_table[MAX9295_INIT]);
                        cam_gmsl_write_reg(priv,0x40,0x0000,0x86);//wrete addr 0x43
                        break;
                case 0x1d:
                        cam_gmsl_write_reg(priv,0x29,0x0006,0xf8);
                        usleep_range(500000, 1000000);
                        err = cam_gmsl_read_reg(priv, 0x40, 0x0000, &check);
                        if(err){
                                dev_err(dev, "camera : %x not found \n",client->addr);
                                goto done;
                        }
                        cam_gmsl_write_table(priv, mode_table[MAX9295_INIT]);
                        cam_gmsl_write_reg(priv,0x40,0x0000,0x88);//wrete addr 0x44
                        break;
                default :
                        dev_err(dev,"camera reg error\n");
                        return 0;
        }

        cam_gmsl_write_table(priv, mode_table[SENSOR_MODE_2560X1984_4LANE_RAW10_30FPS_LINEAR]); 
        cam_gmsl_write_table(priv, mode_table[SENSOR_MODE_2560X1984_4LANE_RAW10_30FPS_SLAVE]); 
        usleep_range(100000, 200000);
        cam_gmsl_write_table(priv, mode_table[SENSOR_START_STREAM]); 
        dev_info(&client->dev, "Probe is successful!\n");
done:
        cam_gmsl_write_reg(priv,0x29,0x0006,0xff);
        if(err){
        
                dev_err(dev,"camera init error\n");
                return 0;
        }
        return 0;
*/

	dev_info(&client->dev, "Probe is successful!\n");
	return 0;
}


static const struct serdes_cfg serdes_reset_seq[] = {
	/* ---- 解串器初始化 ---- */
	_WR(0x41, 0x00, 0x10, 0x80),	/* Device Reset */
	_WR(0x42, 0x00, 0x10, 0x80),	/* Device Reset */
	_WR(0x43, 0x00, 0x10, 0x80),	/* Device Reset */
	_WR(0x44, 0x00, 0x10, 0x80),	/* Device Reset */
	_DLY(300),

};

void my_reset_ser_dser(struct cam_gmsl *priv);
void my_reset_ser_dser(struct cam_gmsl *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct i2c_adapter *adapter = client->adapter;
	struct device *dev = priv->s_data->dev;

	static struct i2c_adapter *last_adapter;
	static bool done;
	int i;

	/* 同一 adapter 仅执行一次 */
	if (done && last_adapter == adapter)
		return;

	done = false;

	for (i = 0; i < ARRAY_SIZE(serdes_reset_seq); i++) {
		const struct serdes_cfg *cfg = &serdes_reset_seq[i];

		if (cfg->slave_addr == 0) {
			msleep(cfg->val);
		} else {
			u8 buf[3] = { cfg->reg_hi, cfg->reg_lo, (u8)cfg->val };
			struct i2c_msg msg = {
				.addr = cfg->slave_addr,
				.flags = 0,
				.len   = 3,
				.buf   = buf,
			};
			if (i2c_transfer(adapter, &msg, 1) != 1)
				dev_err(dev, "cfg[%d] i2c-%u wr 0x%02x [0x%02x%02x]=0x%02x failed\n",
					i, adapter->nr, cfg->slave_addr,
					cfg->reg_hi, cfg->reg_lo, cfg->val);
			else
				dev_info(dev, "cfg[%d] i2c-%u wr 0x%02x [0x%02x%02x]=0x%02x ok\n",
					i, adapter->nr, cfg->slave_addr,
					cfg->reg_hi, cfg->reg_lo, cfg->val);
		}
	}

	last_adapter = adapter;
	done = true;
}

static void cam_gmsl_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam_gmsl *priv = (struct cam_gmsl *)s_data->priv;
	my_reset_ser_dser(priv);
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	dev_info(&client->dev, "%s\n",__func__);
	//return 0;
}

static const struct i2c_device_id cam_gmsl_id[] = {
	{ "ar0234_gmsl", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, cam_gmsl_id);

static struct i2c_driver cam_gmsl_i2c_driver = {
	.driver = {
		.name = "ar0234_gmsl",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cam_gmsl_of_match),
	},
	.probe = cam_gmsl_probe,
	.remove = cam_gmsl_remove,
	.id_table = cam_gmsl_id,
};

module_i2c_driver(cam_gmsl_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for CAM_GMSL");
MODULE_AUTHOR("zhang");
MODULE_LICENSE("GPL v2");
