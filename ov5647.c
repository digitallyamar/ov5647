/*
 * ov5647.c - ov5647 sensor driver
 * Author: Amarnath B R (@digitallyamar)
 * Copyright (c) 2021, GiraffAI.  All rights reserved.
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


#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/tegra-v4l2-camera.h>
#include <media/tegracam_core.h>

#include "ov5647_modes_tbls.h"

/* imx219 sensor register address */
#define OV5647_SC_CMMN_CHIP_ID_MSB		0x300A
#define OV5647_SC_CMMN_CHIP_ID_LSB		0x300B

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};


struct ov5647 {
    struct i2c_client *i2c_client;
    struct v4l2_subdev *subdev;
    struct camera_common_data *s_data;
    struct tegracam_device *tc_dev;
    struct regmap			*regmap;
};

static struct regmap_config ov5647_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
};

static const struct of_device_id ov5647_of_match[] = {
    {
        .compatible = "nvidia,ov5647",
    },
    { },
};

static int ov5647_write_table(struct ov5647 *priv,
			      const ov5647_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 OV5647_TABLE_WAIT_MS,
					 OV5647_TABLE_END);
}

static int ov5647_set_mode(struct tegracam_device *tc_dev)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err;

	err = ov5647_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	return 0;
}

static struct camera_common_pdata *ov5647_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int gpio;
	int err;
	struct camera_common_pdata *ret = NULL;

    match = of_match_device(ov5647_of_match, dev);
	if (!match) {
		dev_err(dev, "Error: Failed to find matching dt id\n");
		return NULL;
	}

    board_priv_pdata = devm_kzalloc(dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata) {
        printk("Amar::ov5647.c: Error: Failed to kzalloc board_priv_pdata memory!");
		return NULL;
    }

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found\n");
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	dev_info(dev, "Reset GPIO pin = %d\n", board_priv_pdata->reset_gpio);

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk name not present, "
			"assume sensor driven externally\n");

	err = of_property_read_string(np, "avdd-reg",
		&board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
		&board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
		&board_priv_pdata->regulators.dvdd);
	if (err)
		dev_err(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
			"assume sensor powered independently\n");

	board_priv_pdata->has_eeprom =
		of_property_read_bool(np, "has-eeprom");
	
	printk ("board_priv_pdata->has_eeprom = %d\n", board_priv_pdata->has_eeprom);



	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

static int ov5647_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct clk *parent;
	int err = 0;

	dev_info(dev, "%s++", __func__);

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	/* Sensor MCLK (aka. INCK) */
	if (pdata->mclk_name) {
		pw->mclk = devm_clk_get(dev, pdata->mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(dev, "unable to get clock %s\n",
				pdata->mclk_name);
			return PTR_ERR(pw->mclk);
		}

		if (pdata->parentclk_name) {
			parent = devm_clk_get(dev, pdata->parentclk_name);
			if (IS_ERR(parent)) {
				dev_err(dev, "unable to get parent clock %s",
					pdata->parentclk_name);
			} else
				clk_set_parent(pw->mclk, parent);
		}
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
				&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
				&pw->iovdd, pdata->regulators.iovdd);
	/* dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
				&pw->dvdd, pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	/* Reset or ENABLE GPIO */
	pw->reset_gpio = pdata->reset_gpio;
	err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (err < 0) {
		dev_err(dev, "%s: unable to request reset_gpio (%d)\n",
			__func__, err);
		goto done;
	}

done:
	pw->state = SWITCH_OFF;

	return err;
}


static int ov5647_power_put(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	dev_info(dev, "%s++", __func__);

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->dvdd))
		devm_regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		devm_regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (likely(pw->reset_gpio))
		gpio_free(pw->reset_gpio);

	return 0;	
}

static int ov5647_set_gain(struct tegracam_device *tc_dev, s64 val)
{
    return 0;
}

static int ov5647_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
    return 0;
}

static int ov5647_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
    return 0;
}

static int ov5647_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
    return 0;
}




static int ov5647_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	
	dev_info(dev, "%s: power on\n", __func__);

	if (pdata && pdata->power_on) {
		dev_info(dev, "%s: Amar: Calling pdata->power_on()\n", __func__);

		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	dev_info(dev, "%s: Amar: pdata->power_on Not present, going for gpio reset\n", __func__);

	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 0);
		else
			gpio_set_value(pw->reset_gpio, 0);
	}

	if (unlikely(!(pw->avdd || pw->iovdd || pw->dvdd)))
		goto skip_power_seqn;

	usleep_range(10, 20);

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto ov5647_avdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto ov5647_iovdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto ov5647_dvdd_fail;
	}

	usleep_range(10, 20);

skip_power_seqn:
	if (pw->reset_gpio) {
		dev_info(dev, "%s: Amar: In skip_power_seqn, going for gpio reset\n", __func__);
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 1);
		else
			gpio_set_value(pw->reset_gpio, 1);
	}

	/* Need to wait for t4 + t5 + t9 time as per the data sheet */
	/* t4 - 200us, t5 - 21.2ms, t9 - 1.2ms */
	usleep_range(23000, 23100);

	pw->state = SWITCH_ON;

	return 0;

ov5647_dvdd_fail:
	regulator_disable(pw->iovdd);

ov5647_iovdd_fail:
	regulator_disable(pw->avdd);

ov5647_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;	
}

static int ov5647_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	} else {
		if (pw->reset_gpio) {
			if (gpio_cansleep(pw->reset_gpio))
				gpio_set_value_cansleep(pw->reset_gpio, 0);
			else
				gpio_set_value(pw->reset_gpio, 0);
		}

		usleep_range(10, 10);

		if (pw->dvdd)
			regulator_disable(pw->dvdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->avdd)
			regulator_disable(pw->avdd);
	}

	pw->state = SWITCH_OFF;

	return 0;
}

static int ov5647_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
    return 0;
}

static int ov5647_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;

	return err;
}

static int ov5647_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    dev_err(&client->dev, "Amar: %s:\n", __func__);

    return 0;
}

static struct camera_common_sensor_ops ov5647_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ov5647_frmfmt),
	.frmfmt_table = ov5647_frmfmt,    
    .power_on = ov5647_power_on,
    .power_off = ov5647_power_off,
    .write_reg = ov5647_write_reg,
    .read_reg = ov5647_read_reg,
    .parse_dt = ov5647_parse_dt,
    .power_get = ov5647_power_get,
	.power_put = ov5647_power_put,
    .set_mode = ov5647_set_mode,
};

static const struct v4l2_subdev_internal_ops ov5647_subdev_internal_ops = {
    .open = ov5647_open,
};

static struct tegracam_ctrl_ops ov5647_ctrl_ops = {
    .numctrls = ARRAY_SIZE(ctrl_cid_list),
    .ctrl_cid_list = ctrl_cid_list,
    .set_gain = ov5647_set_gain,
    .set_exposure = ov5647_set_exposure,
    .set_frame_rate = ov5647_set_frame_rate,
    .set_group_hold = ov5647_set_group_hold,
};

MODULE_DEVICE_TABLE(of, ov5647_of_match);

static const struct i2c_device_id ov5647_id[] = {
    { "ov5647", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, ov5647_id);


static int ov5647_board_setup(struct ov5647 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	u8 reg_val[2];
	int err = 0;

	dev_info(dev, "%s++\n", __func__);

	dev_info(dev, "%s, s_data->priv = %p\n", __func__, s_data->priv);

	if (pdata->mclk_name) {
		dev_info(dev, "%s, calling camera_common_mclk_enable\n", __func__);
		err = camera_common_mclk_enable(s_data);
		if (err) {
			dev_err(dev, "error turning on mclk (%d)\n", err);
			goto done;
		}
	}

	dev_info(dev, "%s, calling ov5647_power_on\n", __func__);
	err = ov5647_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto err_power_on;
	}

	/* Probe sensor model id registers */
	err = ov5647_read_reg(s_data, OV5647_SC_CMMN_CHIP_ID_MSB, &reg_val[0]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}

	dev_info(dev, "%s, OV5647_SC_CMMN_CHIP_ID_MSB Register value = 0x%x\n", __func__, reg_val[0]);

	err = ov5647_read_reg(s_data, OV5647_SC_CMMN_CHIP_ID_LSB, &reg_val[1]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}

	dev_info(dev, "%s, OV5647_SC_CMMN_CHIP_ID_LSB Register value = 0x%x\n", __func__, reg_val[1]);

	if (!((reg_val[0] == 0x56) && reg_val[1] == 0x47))
		dev_err(dev, "%s: invalid sensor model id: %x%x\n",
			__func__, reg_val[0], reg_val[1]);
	
	dev_info(dev, "Amar: ov5647_board_setup++, *s_data = %p\n", s_data);

err_reg_probe:
	ov5647_power_off(s_data);

err_power_on:
	if (pdata->mclk_name)
		camera_common_mclk_disable(s_data);

done:
	return err;

}


static int ov5647_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct device_node *node = client->dev.of_node;
    struct ov5647 *priv;
    struct tegracam_device *tc_dev;
    int err;

    dev_err(dev, "Amar: ov5647_probe++");
    dev_err(dev, "Amar: (ov5647_probe) i2c_client->name = %s\n", client->name);
    dev_err(dev, "Amar: (ov5647_probe) i2c_client->dev = %s\n", dev->init_name);
    dev_err(dev, "Amar: (ov5647_probe) node = %p\n", node);

	dev_info(dev, "probing v4l2 sensor at addr 0x%0x\n", client->addr);

    if (!IS_ENABLED(CONFIG_OF) || !node)
    {
	    printk("Amar: CONFIG_OF NOT ENABLED, bailing out...\n");
	    return -EINVAL;
    }


    priv = devm_kzalloc(dev, sizeof(struct ov5647), GFP_KERNEL);

    if (!priv) {
	printk("Amar: priv kzalloc failed!\n");
	return -ENOMEM;
    }


    tc_dev = devm_kzalloc(dev, sizeof(struct tegracam_device), GFP_KERNEL);

    if (!tc_dev) {
	printk("Amar: tc_dev kzalloc failed\n");
	return -ENOMEM;
    }

    priv->i2c_client = tc_dev->client = client;
    tc_dev->dev = dev;
    strncpy(tc_dev->name, "ov5647", sizeof(tc_dev->name));
    tc_dev->dev_regmap_config = &ov5647_regmap_config;
    tc_dev->sensor_ops = &ov5647_common_ops;
    tc_dev->v4l2sd_internal_ops = &ov5647_subdev_internal_ops;
    tc_dev->tcctrl_ops = &ov5647_ctrl_ops;

    
    err = tegracam_device_register(tc_dev);
    if (err) {
	dev_err(dev, "tegra camera driver registration failed\n");
	return err;
    }
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);


	err = ov5647_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}            

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(dev, "%s: detected ov5647 sensor\n", __func__);

	return 0;
}

static int
ov5647_remove(struct i2c_client *client)
{
    struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);

	struct camera_common_data *amar_s_data = container_of(dev_get_drvdata(dev),
		    struct camera_common_data, subdev);

	// struct ov5647 *priv = (struct ov5647 *)s_data->priv;
	
	// *** AMAR *** : s_data pointer does not match with the value found in board setup
	// Hence segment core dumping!!!
	// Amar: ov5647_board_setup++, *s_data = ffffffc0702f6418
	// Amar: ov5647_remove++, *s_data = ffffffffffffffc8
	
	
	dev_info(dev, "Amar: ov5647_remove++, *s_data = %p\n", s_data);
	dev_info(dev, "Amar: ov5647_remove++, *amar_s_data = %p\n", amar_s_data);
    // dev_err(dev, "Amar: ov5647_remove++, priv = %p\n", priv);

	// tegracam_v4l2subdev_unregister(priv->tc_dev);

	dev_err(dev, "%s: Calling tegracam_device_unregister.. ", __func__);

	// tegracam_device_unregister(priv->tc_dev);

	dev_err(dev, "%s: Done tegracam_device_unregister.. ", __func__);

    return 0;
}

/*
static int __init ov5647_init(void)
{
    printk("Amar: ov5647_init++");

    return 0;
}
*/

static struct i2c_driver ov5647_i2c_driver = {
    .driver = {
        .name = "ov5647",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ov5647_of_match),
    },
    .probe = ov5647_probe,
    .remove = ov5647_remove,
    .id_table = ov5647_id
};

module_i2c_driver(ov5647_i2c_driver);

//module_init(ov5647_init);

MODULE_DESCRIPTION("Media Controller driver for OmniVision OV5647");
MODULE_AUTHOR("Amarnath B R (@digitallyamar) - GiraffAI");
MODULE_LICENSE("GPL v2");
