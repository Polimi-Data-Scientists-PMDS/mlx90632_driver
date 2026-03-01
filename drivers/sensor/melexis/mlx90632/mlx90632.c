/*
 * Copyright (c) Michele Tavecchio <tavecchiomichele03@gmail.com>, 
 * Alessandro Sola <alessandrosola03@gmail.com>,
 * Melexis Technologies NV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT melexis_mlx90632

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
// #include <zephyr/drivers/sensor/mlx90632.h>  // TODO: create public header

#include "mlx90632.h"

LOG_MODULE_REGISTER(MLX90632, CONFIG_SENSOR_LOG_LEVEL);

static int mlx90632_reg_read(const struct device *dev, uint16_t reg, uint16_t *val)
{
    const struct mlx90632_config *config = dev->config;
    uint8_t reg_addr[2] = { reg >> 8, reg & 0xFF };
    uint8_t data[2];
    int ret;

    /* Write register address */
    ret = i2c_write_dt(&config->i2c, reg_addr, sizeof(reg_addr));
    if (ret < 0) {
        LOG_ERR("Failed to write register address 0x%04X: %d", reg, ret);
        return ret;
    }

    /* Read data from register */
    ret = i2c_read_dt(&config->i2c, data, sizeof(data));
    if (ret < 0) {
        LOG_ERR("Failed to read data from register 0x%04X: %d", reg, ret);
        return ret;
    }

    *val = (data[0] << 8) | data[1];
    return 0;
}

static int mlx90632_init(const struct device *dev)
{
    const struct mlx90632_config *cfg = dev->config;
    uint16_t status;
    uint16_t p_code;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

    k_msleep(MLX90632_STARTUP_TIME_MS);

    int ret = mlx90632_reg_read (dev, MLX90632_REG_STATUS, &status);
    if (ret<0){
        LOG_ERR("Failed to read status register");
        return ret;
    }
    

    if (status & MLX90632_STAT_EE_BUSY){
        LOG_WRN("EEPROM busy (copy in progress)");
        return -EBUSY;
    }

    ret = mlx90632_reg_read(dev, MLX90632_EE_PRODUCT_CODE, &p_code);
    if (ret < 0) {
        LOG_ERR("Failed to Read product code");
        return ret;
    }

    LOG_INF("MLX90632 detected. Product Code: 0x%04X", p_code);

    return 0;
}


#define MLX90632_DEFINE(inst) \
    static struct mlx90632_data mlx90632_data_##inst; \
    static const struct mlx90632_config mlx90632_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
    }; \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, \
                         mlx90632_init, \
                         NULL, \
                         &mlx90632_data_##inst, \
                         &mlx90632_config_##inst, \
                         POST_KERNEL, \
                         CONFIG_SENSOR_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(MLX90632_DEFINE)