/*
 * Copyright (c) Michele Tavecchio <tavecchiomichele03@gmail.com>, 
 * Alessandro Sola <alessandrosola03@gmail.com>,
 * Melexis Technologies NV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MLX90632_MLX90632_H_
#define ZEPHYR_DRIVERS_SENSOR_MLX90632_MLX90632_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>


#define MLX90632_STARTUP_TIME_MS    64   
#define MLX90632_RESET_DELAY_US     150

#define MLX90632_EE_ID0             0x2405 
#define MLX90632_EE_ID1             0x2406 
#define MLX90632_EE_ID2             0x2407 
#define MLX90632_EE_PRODUCT_CODE    0x2409


#define MLX90632_REG_STATUS         0x3FFF 
#define MLX90632_STAT_EE_BUSY       BIT(9) 



struct mlx90632_config {
    struct i2c_dt_spec i2c;
};

struct mlx90632_data {

};

#endif /* ZEPHYR_DRIVERS_SENSOR_MLX90632_MLX90632_H_ */