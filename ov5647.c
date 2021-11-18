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

static const struct of_device_id ov5647_of_match[] = {
    {
        .compatible = "nvidia,ov5693",
    },
    { },
};

MODULE_DEVICE_TABLE(of, ov5647_of_match);

static const struct i2c_device_id ov5647_id[] = {
    { "ov5647", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, ov5647_id);


static int ov5647_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;

    dev_info(dev, "Amar: ov56467_probe++");
    return 0;
}

static int
ov5647_remove(struct i2c_client *client)
{
    struct device *dev = &client->dev;

    dev_info(dev, "Amar: ov5647_remove++");
    return 0;
}

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

MODULE_DESCRIPTION("Media Controller driver for OmniVision OV5647");
MODULE_AUTHOR("Amarnath B R (@digitallyamar) - GiraffAI");
MODULE_LICENSE("GPL v2");