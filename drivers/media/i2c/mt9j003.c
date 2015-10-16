/*
 * Driver for mt9j003 image sensor
 *
 * Copyright (C) 2015 Antmicro Ltd.
 *
 * Author(s):
 *      Szymon Sobczak
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/of_graph.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-of.h>
#include "mt9j003.h"

#define I2C_SEND_AND_CHECK(client, reg, val) \
	do {\
		err = write_word(client, reg, val); \
		if (err < 0) \
			return err; } while (0)

#define I2C_READ_AND_CHECK(client, reg, val) \
	do {\
		err = read_register(client, reg, val); \
		if (err < 0) \
			return err; } while (0)

MODULE_DESCRIPTION("MT9J003 image sensor support");
MODULE_AUTHOR("Szymon Sobczak");
MODULE_LICENSE("GPL v2");

enum mt9j003_type {
	mt9j003,
};

struct mt9j003_chip_info {};
static const struct mt9j003_chip_info mt9j003_chip_info[] = {{},};

struct mt9j003_state {
	struct v4l2_ctrl_handler hdl;
	struct v4l2_subdev sd;
	bool gpio_set;
	u32 reset_gpio_number;
	struct i2c_client *client;
	int reset_mode;
};

struct my_gpio {
	u8 port_number;
	unsigned long flag;
	const char *label;
};

static int write_word(struct i2c_client *client, u16 reg_addr, u16 value)
{
	struct i2c_data;
	struct i2c_msg msg[2];
	unsigned char data[4];
	int err;

	data[0] = (reg_addr >> 8) & 0xff;
	data[1] = reg_addr & 0xff;
	data[2] = (value>>8) & 0xff;
	data[3] = value & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_STOP;
	msg[0].len = 4;
	msg[0].buf = data;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err < 0)
		dev_err(&client->dev, "i2c transfer error: %d\n", err);
	return err;

}

static int read_register(struct i2c_client *client, u16 reg_addr, u16 *reg)
{
	struct i2c_data;
	struct i2c_msg msg[2];
	unsigned char reg_address[2];
	int err;

	reg_address[0] = (reg_addr >> 8) & 0xff;
	reg_address[1] = reg_addr & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = reg_address;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD | I2C_M_STOP;
	msg[1].len = 2;
	msg[1].buf = (void *)reg;
	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		dev_err(&client->dev, "i2c transfer error: %d\n", err);
	return err;

}

static int configure_default(struct i2c_client *client)
{
	int err;
	u16 reg;
	u32 skip_val;
	unsigned int x_addr_start = (INIT_X_OFFSET-INIT_X_RES*INIT_SKIP)/2;
	unsigned int y_addr_start = (INIT_Y_OFFSET-INIT_Y_RES*INIT_SKIP)/2;

	dev_dbg(&client->dev,
		"Configure sensor with default registers...\n");
	if (INIT_SKIP == 2)
		skip_val = MT_SKIP_2;
	else if (INIT_SKIP == 4)
		skip_val = MT_SKIP_4;
	else
		skip_val = MT_SKIP_1;

	I2C_SEND_AND_CHECK(client, MT_REG_VT_PIX_CLK_DIV,
			   MT_DVAL_VT_PIX_CLK_DIV);
	I2C_SEND_AND_CHECK(client, MT_REG_VT_SYS_CLK_DIV,
			   MT_DVAL_VT_SYS_CLK_DIV);
	I2C_SEND_AND_CHECK(client, MT_REG_PRE_PLL_CLK_DIV,
			   MT_DVAL_PRE_PLL_CLK_DIV);
	I2C_SEND_AND_CHECK(client, MT_REG_PLL_MULTIPLIER,
			   MT_DVAL_PLL_MULTIPLIER);
	I2C_SEND_AND_CHECK(client, MT_REG_OP_PIX_CLK_DIV,
			   MT_DVAL_OP_PIX_CLK_DIV);
	I2C_SEND_AND_CHECK(client, MT_REG_OP_SYS_CLK_DIV,
			   MT_DVAL_OP_SYS_CLK_DIV);
	I2C_READ_AND_CHECK(client, MT_REG_ROW_SPEED, &reg);
	reg = (reg & MT_PC_SPEED_MASK_001) | MT_OP_SPEED_MASK_001;
	I2C_SEND_AND_CHECK(client, MT_REG_ROW_SPEED, reg);
	I2C_SEND_AND_CHECK(client, MT_REG_RST_REG, MT_RST_PWR_UP);
	I2C_SEND_AND_CHECK(client, MT_REG_DIS_EM_DATA, MT_VAL_MAGIC_01);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_02, MT_VAL_MAGIC_02);
	I2C_READ_AND_CHECK(client,  MT_REG_DIS_EM_DATA, &reg);
	reg = (reg & MT_DIS_SMIA_ENC_MASK);
	I2C_SEND_AND_CHECK(client, MT_REG_DIS_EM_DATA, reg);
	I2C_SEND_AND_CHECK(client, MT_REG_DIG_GAIN_GREEN, MT_DVAL_DIG_GAIN);
	I2C_SEND_AND_CHECK(client, MT_REG_DIG_GAIN_RED, MT_DVAL_DIG_GAIN);
	I2C_SEND_AND_CHECK(client, MT_REG_DIG_GAIN_BLUE, MT_DVAL_DIG_GAIN);
	I2C_SEND_AND_CHECK(client, MT_REG_DIG_GAIN_GREENB, MT_DVAL_DIG_GAIN);
	I2C_SEND_AND_CHECK(client, MT_REG_CCP_DATA_FMT, MT_FMT_BAY_10b);

	I2C_SEND_AND_CHECK(client, MT_REG_X_ADDR_START, x_addr_start);
	I2C_SEND_AND_CHECK(client, MT_REG_X_ADDR_END, INIT_X_OFFSET);
	I2C_SEND_AND_CHECK(client, MT_REG_Y_ADDR_START, y_addr_start);
	I2C_SEND_AND_CHECK(client, MT_REG_Y_ADDR_END, INIT_Y_OFFSET);
	I2C_SEND_AND_CHECK(client, MT_REG_X_OUT_SIZE, INIT_X_RES);
	I2C_SEND_AND_CHECK(client, MT_REG_Y_OUT_SIZE, INIT_Y_RES);

	I2C_SEND_AND_CHECK(client, MT_REG_Y_OUT_SIZE, INIT_Y_RES);
	I2C_SEND_AND_CHECK(client, MT_REG_LINE_LENGTH_PCK,
			   MT_DVAL_LINE_LENGTH_PIX);
	I2C_SEND_AND_CHECK(client, MT_REG_FRAME_LENGTH_LINES,
			   MT_DVAL_FRAME_LENGTH_LINES);
	I2C_SEND_AND_CHECK(client, MT_REG_FINE_CORRECTION,
			   MT_DVAL_FINE_CORRECTION);
	I2C_SEND_AND_CHECK(client, MT_REG_READ_MODE, skip_val);
	I2C_SEND_AND_CHECK(client, MT_REG_SCALE_MODE, 2);
	I2C_SEND_AND_CHECK(client, MT_REG_DATA_PATH_STATUS,
			   MT_DVAL_DATA_PATH_STATUS);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_03, MT_VAL_MAGIC_03);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_04, MT_VAL_MAGIC_04);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_05, MT_VAL_MAGIC_05);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_06 , MT_VAL_MAGIC_06);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_07 , MT_VAL_MAGIC_07);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_08 , MT_VAL_MAGIC_08);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_09 , MT_VAL_MAGIC_09);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_0A , MT_VAL_MAGIC_0A);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_0B , MT_VAL_MAGIC_0B);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_0C , MT_VAL_MAGIC_0C);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_0D , MT_VAL_MAGIC_0D);
	I2C_SEND_AND_CHECK(client, MT_REG_MAGIC_0E , MT_VAL_MAGIC_0E);
	I2C_READ_AND_CHECK(client, MT_REG_ADC_CAL , &reg);
	reg = reg  | MT_ADC_CAL_MASK;
	I2C_SEND_AND_CHECK(client, MT_REG_ADC_CAL , reg);
	I2C_READ_AND_CHECK(client, MT_REG_XY_SUM_CTRL , &reg);
	reg = (reg & MT_7FPS_FULLARRAY_MASK);
	I2C_SEND_AND_CHECK(client, MT_REG_XY_SUM_CTRL , reg);
	I2C_SEND_AND_CHECK(client, MT_REG_COARSE_INT_TIME,
			   MT_DVAL_COARSE_INT_TIME);
	I2C_SEND_AND_CHECK(client, MT_REG_FINE_INT_TIME,
			   MT_DVAL_FINE_INT_TIME);
	I2C_SEND_AND_CHECK(client, MT_REG_EXTRA_DELAY, 0);
	I2C_SEND_AND_CHECK(client, MT_REG_PWR_OPT , MT_VAL_MAGIC_11);
	I2C_SEND_AND_CHECK(client, MT_REG_COLUMN_CORR , MT_VAL_MAGIC_12);
	I2C_SEND_AND_CHECK(client, MT_REG_HISPI_CTRL_STAT, 0x00 |
			   MT_HISPI_SQUARE_WAVE  | MT_HISPI_PACKETIZED |
			   MT_HISPI_MSB_FISRT);
	I2C_SEND_AND_CHECK(client, MT_REG_TST_PATTERN_MODE, 0);
	I2C_SEND_AND_CHECK(client, MT_REG_RST_REG, 0x00 | MT_RST_STREAM |
			   MT_RST_LOCK_REG | MT_RST_STDBY_EOF);
	return 0;
}

static inline struct mt9j003_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9j003_state, sd);
}



static int mt9j003_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct mt9j003_state *state = to_state(sd);
	u16 reg;
	int err, ret;

	if (format->which ==  V4L2_SUBDEV_FORMAT_ACTIVE) {
		I2C_READ_AND_CHECK(state->client, MT_REG_CCP_DATA_FMT, &reg);
		switch (reg) {
		case MT_FMT_BAY_10b:
			format->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
			break;
		case MT_FMT_BAY_12b:
			format->format.code = V4L2_MBUS_FMT_SBGGR12_1X12;
			break;
		case MT_FMT_BAY_08b:
			format->format.code = V4L2_MBUS_FMT_SBGGR8_1X8;
			break;
		}
	}
	if (format->format.width > MAX_X || format->format.width < MIN_X ||
	    format->format.height > MAX_Y || format->format.height < MIN_Y)
		ret = EINVAL;
	else
		ret = 0;
	return ret;
}

static int mt9j003_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	int x_res, y_res, x_offset, y_offset, err, skip_val;
	struct mt9j003_state *state = to_state(sd);
	unsigned int x_addr_start, y_addr_start;

	x_res = format->format.width;
	y_res = format->format.height;
	x_offset = INIT_X_OFFSET;
	y_offset = INIT_Y_OFFSET;
	x_addr_start = (x_offset-x_res*INIT_SKIP)/2;
	y_addr_start = (y_offset-y_res*INIT_SKIP)/2;

	if (INIT_SKIP == 2)
		skip_val = MT_SKIP_2;
	else if (INIT_SKIP == 4)
		skip_val = MT_SKIP_4;
	else
		skip_val = MT_SKIP_1;

	I2C_SEND_AND_CHECK(state->client, MT_REG_X_ADDR_START, x_addr_start);
	I2C_SEND_AND_CHECK(state->client, MT_REG_Y_ADDR_START, y_addr_start);
	I2C_SEND_AND_CHECK(state->client, MT_REG_X_OUT_SIZE, x_res);
	I2C_SEND_AND_CHECK(state->client, MT_REG_Y_OUT_SIZE, y_res);
	I2C_SEND_AND_CHECK(state->client, MT_REG_READ_MODE, skip_val);

	return 0;
}


static int mt9j003_reset(struct v4l2_subdev *sd, u32 val)
{
	struct my_gpio gp;
	int err = 0;
	struct mt9j003_state *state = to_state(sd);

	state->reset_mode = val;

	if (val != 1)
		goto esc;

	dev_dbg(sd->dev, "Reseting sensor from the gpio\n");
	gp.flag = GPIOF_OUT_INIT_LOW;
	gp.label = "gp_reset";

	if (!state->gpio_set) {
		state->reset_gpio_number = of_get_named_gpio(sd->dev->of_node,
							     "ant,reset-gpio",
							     0);
		if (!gpio_is_valid(state->reset_gpio_number)) {
			dev_err(&state->client->dev,
				"Failed to read property \"ant,reset-gpio\"\n");
			goto error;
		}
		gp.port_number = state->reset_gpio_number;
		err = gpio_request_one(gp.port_number, gp.flag, gp.label);
		if (err) {
			dev_err(&state->client->dev, "Gpio request error\n");
			goto error;
		} else
			state->gpio_set = true;
	}
	if (gpio_get_value(gp.port_number) != 0)
		gpio_set_value(gp.port_number, 0);
	mdelay(100);
	gpio_set_value(gp.port_number, 1);
	mdelay(1);
esc:
	return 0;
error:
	return err;
}

static int mt9j003_init(struct v4l2_subdev *sd, u32 val)
{
	struct mt9j003_state *state = to_state(sd);

	configure_default(state->client);
	return 0;
}
static int mt9j003_registered(struct v4l2_subdev *sd)
{
	struct mt9j003_state *state = to_state(sd);

	if (!state->reset_mode) {
		mt9j003_reset(sd, 1);
		configure_default(state->client);
	}
	return 0;
}
static int mt9j003_s_stream(struct v4l2_subdev *sd, int enable)
{
	/*
	 * TODO:
	 * function is implemented but empty
	 * because streaming must be enabled in
	 * registered function for properly syncing
	 */
	return 0;
}

static const struct v4l2_subdev_core_ops mt9j003_core_ops = {
	.reset = mt9j003_reset,
	.init = mt9j003_init,
};
static const struct v4l2_subdev_internal_ops mt9j003_internal_ops = {
	.registered = mt9j003_registered,
};

static const struct v4l2_subdev_pad_ops mt9j003_pad_ops = {
	.get_fmt = mt9j003_get_format,
	.set_fmt = mt9j003_set_format,
};
static const struct v4l2_subdev_video_ops mt9j003_video_ops = {
	.s_stream = &mt9j003_s_stream,
};
static const struct v4l2_subdev_ops mt9j003_ops = {
	.core = &mt9j003_core_ops,
	.pad = &mt9j003_pad_ops,
	.video = &mt9j003_video_ops,
};



static struct i2c_device_id mt9j003_i2c_id[] = {
	{ "mt9j003", (kernel_ulong_t)&mt9j003_chip_info[mt9j003] },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9j003_i2c_id);
static struct of_device_id mt9j003_of_id[] = {
	{ .compatible = "apt,mt9j003", },
	{ }
};
MODULE_DEVICE_TABLE(of, mt9j003_of_id);
static int mt9j003_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{

	struct v4l2_ctrl_handler *hdl;
	struct v4l2_subdev *sd;
	struct mt9j003_state *state;

	int err;

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->gpio_set = false;
	state->client = client;
	sd = &state->sd;
	hdl = &state->hdl;
	state->reset_mode = 0;

	state->sd.internal_ops = &mt9j003_internal_ops;
	v4l2_i2c_subdev_init(sd, client, &mt9j003_ops);
	v4l2_ctrl_handler_init(hdl, 2);

	err = v4l2_async_register_subdev(sd);
	if (err)
		return -err;
	return 0;

}

static int mt9j003_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9j003_state *state = to_state(sd);

	if (state->gpio_set)
		gpio_free(state->reset_gpio_number);

	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);

	return 0;
}

static struct i2c_driver mt9j003_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mt9j003",
		.of_match_table = of_match_ptr(mt9j003_of_id),
	},
	.probe = mt9j003_probe,
	.remove = mt9j003_remove,
	.id_table = mt9j003_i2c_id,
};

module_i2c_driver(mt9j003_driver);
