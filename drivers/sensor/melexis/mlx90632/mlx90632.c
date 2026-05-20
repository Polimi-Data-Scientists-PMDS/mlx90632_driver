/*
 * SPDX-FileCopyrightText: Copyright (c) Michele Tavecchio <tavecchiomichele03@gmail.com>
 * SPDX-FileCopyrightText: Copyright (c) Alessandro Sola <alessandrosola03@gmail.com>
 * SPDX-FileCopyrightText: Copyright (c) 2017 Melexis N.V.
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

#include "mlx90632.h"
#include <math.h>

LOG_MODULE_REGISTER(MLX90632, CONFIG_SENSOR_LOG_LEVEL);

static int mlx90632_reg_read(const struct device *dev, uint16_t reg, uint16_t *val)
{
	const struct mlx90632_config *config = dev->config;
	uint8_t reg_addr[2] = {reg >> 8, reg & 0xFF};
	uint8_t data[2];
	int ret;

	ret = i2c_write_read_dt(&config->i2c, reg_addr, sizeof(reg_addr), data, sizeof(data));
	if (ret < 0) {
		LOG_ERR("Failed to read register 0x%04x (rc=%d)", reg, ret);
		return ret;
	}

	*val = sys_get_be16(data);
	return 0;
}

static uint16_t mlx90632_get_control_bits(void)
{
	uint16_t reg = 0;

#if defined(CONFIG_MLX90632_MODE_SLEEPING_STEP)
	reg |= (0x01 << 1);
#elif defined(CONFIG_MLX90632_MODE_STEP)
	reg |= (0x02 << 1);
#else
	reg |= (0x03 << 1);
#endif
	return reg;
}

static int mlx90632_read_32bit_param(const struct device *dev, uint16_t addr, int32_t *dest)
{
	uint16_t ls, ms;
	int ret;

	ret = mlx90632_reg_read(dev, addr, &ls);
	if (ret < 0) {
		return ret;
	}

	ret = mlx90632_reg_read(dev, addr + 1, &ms);
	if (ret < 0) {
		return ret;
	}

	*dest = (int32_t)((uint32_t)ms << 16 | ls);
	return 0;
}

static int mlx90632_reg_write(const struct device *dev, uint16_t reg, uint16_t val)
{
	const struct mlx90632_config *config = dev->config;
	uint8_t tx_data[4];
	int ret;

	tx_data[0] = reg >> 8;
	tx_data[1] = reg & 0xFF;
	tx_data[2] = (val >> 8) & 0xFF;
	tx_data[3] = val & 0xFF;

	ret = i2c_write_dt(&config->i2c, tx_data, sizeof(tx_data));
	if (ret < 0) {
		LOG_ERR("Failed to write register 0x%04x (rc=%d)", reg, ret);
		return ret;
	}

	return 0;
}

static int mlx90632_load_calibration(const struct device *dev)
{
	struct mlx90632_data *data = dev->data;
	uint16_t p_code;
	int ret;

	ret = mlx90632_reg_read(dev, MLX90632_EE_PRODUCT_CODE, &p_code);
	if (ret < 0) {
		return ret;
	}

	if ((p_code & 0x000F) == 0x0001) {
		data->is_medical = true;
	} else {
		data->is_medical = false;
	}

	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_P_R, &data->p_r);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_P_G, &data->p_g);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_P_T, &data->p_t);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_P_O, &data->p_o);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Aa, &data->aa);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Ab, &data->ab);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Ba, &data->ba);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Bb, &data->bb);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Ca, &data->ca);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Cb, &data->cb);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Da, &data->da);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Db, &data->db);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Ea, &data->ea);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Eb, &data->eb);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Fa, &data->fa);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Fb, &data->fb);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_read_32bit_param(dev, MLX90632_EE_Ga, &data->ga);
	if (ret < 0) {
		return ret;
	}

	ret = mlx90632_reg_read(dev, MLX90632_EE_Gb, (uint16_t *)&data->gb);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_EE_Ka, (uint16_t *)&data->ka);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_EE_Ha, (uint16_t *)&data->ha);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_EE_Hb, (uint16_t *)&data->hb);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int mlx90632_set_refresh_rate(const struct device *dev)
{
	uint16_t rate = CONFIG_MLX90632_REFRESH_RATE;
	uint16_t current_ee;
	uint16_t target_ee1, target_ee2, target_ee3;
	int ret;
	struct mlx90632_data *data = dev->data;
	int num_regs;

	if (data->is_extended) {
		target_ee1 = 0x8000 | (rate << 8);
		target_ee2 = 0x8012 | (rate << 8);
		target_ee3 = 0x800C | (rate << 8);
		num_regs = 3;
	} else {
		target_ee1 = 0x800D | (rate << 8);
		target_ee2 = 0x801D | (rate << 8);
		target_ee3 = 0;
		num_regs = 2;
	}

	ret = mlx90632_reg_read(dev, data->is_extended ? 0x24F1 : 0x24E1, &current_ee);
	if (ret == 0 && current_ee == target_ee1) {
		return 0;
	}

	ret = mlx90632_reg_write(dev, MLX90632_REG_CTRL, 0x0000);
	if (ret < 0) {
		return ret;
	}
	uint16_t addrs[3];
	uint16_t vals[3];

	if (data->is_extended) {
		addrs[0] = 0x24F1;
		vals[0] = target_ee1;
		addrs[1] = 0x24F2;
		vals[1] = target_ee2;
		addrs[2] = 0x24F3;
		vals[2] = target_ee3;
	} else {
		addrs[0] = 0x24E1;
		vals[0] = target_ee1;
		addrs[1] = 0x24E2;
		vals[1] = target_ee2;
	}

	for (int i = 0; i < num_regs; i++) {
		ret = mlx90632_reg_write(dev, MLX90632_REG_UNLOCK, MLX90632_EE_UNLOCK_KEY);
		if (ret < 0) {
			return ret;
		}

		ret = mlx90632_reg_write(dev, addrs[i], 0x0000);
		if (ret < 0) {
			return ret;
		}

		k_msleep(10);
		ret = mlx90632_reg_write(dev, MLX90632_REG_UNLOCK, MLX90632_EE_UNLOCK_KEY);
		if (ret < 0) {
			return ret;
		}

		ret = mlx90632_reg_write(dev, addrs[i], vals[i]);
		if (ret < 0) {
			return ret;
		}

		k_msleep(10);
	}

	ret = mlx90632_reg_write(dev, MLX90632_REG_UNLOCK, MLX90632_RESET_CMD);
	k_msleep(100);

	return ret;
}

static int mlx90632_init(const struct device *dev)
{
	const struct mlx90632_config *cfg = dev->config;
	struct mlx90632_data *data = dev->data;
	uint16_t ee_version;
	uint16_t reg_status;
	uint16_t dummy;
	uint16_t reg_ctrl;
	int ret;

	data->dev = dev;
	k_sem_init(&data->data_sem, 0, 1);
	k_work_init_delayable(&data->data_work, mlx90632_work_handler);

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR_DEVICE_NOT_READY(cfg->i2c.bus);
		return -ENODEV;
	}

	k_msleep(MLX90632_STARTUP_TIME_MS);

	/* dummy read required for 1V8 I2C communication variant */
	(void)mlx90632_reg_read(dev, MLX90632_REG_STATUS, &dummy);

	ret = mlx90632_reg_read(dev, MLX90632_EE_VERSION, &ee_version);
	if (ret < 0) {
		LOG_ERR("Device not responding at 0x%02x (rc=%d)", cfg->i2c.addr, ret);
		return -ENODEV;
	}

	if ((ee_version & 0x00FF) != MLX90632_DSPv5) {
		LOG_ERR("Unsupported DSP version 0x%02x", ee_version & 0x00FF);
		return -ENOTSUP;
	}

	ret = mlx90632_reg_read(dev, MLX90632_REG_STATUS, &reg_status);
	if (ret < 0) {
		return ret;
	}

	reg_status |= MLX90632_STAT_BROWN_OUT;
	reg_status &= ~MLX90632_STAT_DATA_RDY;

	ret = mlx90632_reg_write(dev, MLX90632_REG_STATUS, reg_status);
	if (ret < 0) {
		return ret;
	}

	if ((ee_version & MLX90632_XTD_RNG_BIT) == MLX90632_XTD_RNG_BIT) {
		data->is_extended = true;
	} else {
		data->is_extended = false;
	}

	ret = mlx90632_load_calibration(dev);
	if (ret < 0) {
		return ret;
	}

	int was_reset = mlx90632_set_refresh_rate(dev);

	if (was_reset < 0) {
		return was_reset;
	}

	ret = mlx90632_reg_read(dev, MLX90632_REG_CTRL, &reg_ctrl);
	if (ret < 0) {
		return ret;
	}

	reg_ctrl &= ~(0x03 << 1);
	reg_ctrl &= ~(0x07 << 8);
	reg_ctrl |= mlx90632_get_control_bits();

	ret = mlx90632_reg_write(dev, MLX90632_REG_CTRL, reg_ctrl);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

static int mlx90632_trigger_measurement(const struct device *dev)
{
	uint16_t reg_ctrl;
	int ret;

	ret = mlx90632_reg_read(dev, MLX90632_REG_CTRL, &reg_ctrl);
	if (ret < 0) {
		return ret;
	}

	reg_ctrl |= MLX90632_BIT_SOB;
	return mlx90632_reg_write(dev, MLX90632_REG_CTRL, reg_ctrl);
}

static double mlx90632_calc_amb(struct mlx90632_data *data)
{
	double vr_ta = data->ram_9 + ((data->gb / 1024.0) * (data->ram_6 / 12.0));
	double am_ta = data->ram_6 / 12.0;
	double tmp_ta = (am_ta / vr_ta) * 524288.0;

	double Asub, Bsub, Ablock, Bblock, Cblock;

	Asub = ((double)data->p_t) / (double)17592186044416.0;
	Bsub = (double)tmp_ta - ((double)data->p_r / (double)256.0);
	Ablock = Asub * (Bsub * Bsub);
	Bblock = (Bsub / (double)data->p_g) * (double)1048576.0;
	Cblock = (double)data->p_o / (double)256.0;

	return Bblock + Ablock + Cblock;
}

static double mlx90632_preprocess_object(struct mlx90632_data *data)
{
	double s_obj, v_ir;
	double t_amb = data->ambient_temp;
	double kKa = (double)data->ka / 1024.0;

	double vr_ir = (double)data->ram_9 + kKa * ((double)data->ram_6 / 12.0);

	if (data->is_medical) {
		s_obj = (double)(data->ram_4 + data->ram_5 + data->ram_7 + data->ram_8) / 4.0;
		v_ir = s_obj - ((double)data->aa + (double)data->ab * (t_amb - 25.0));
	} else {
		if (data->last_cycle == 1) {
			s_obj = (double)(data->ram_4 + data->ram_5) / 2.0;
		} else {
			s_obj = (double)(data->ram_7 + data->ram_8) / 2.0;
		}
		v_ir = s_obj;
	}
	return (v_ir / vr_ir) * 524288.0;
}

static double mlx90632_calc_object_medical(struct mlx90632_data *data)
{
	double t_amb = data->ambient_temp;
	double v_ir = mlx90632_preprocess_object(data);

	double kFa = (double)data->fa / 17592186044416.0;
	double kFb = (double)data->fb / 1048576.0;
	double kGa = (double)data->ga / 1048576.0;

	double t_amb_k = t_amb + 273.15;
	double t_amb_k2 = t_amb_k * t_amb_k;
	double t_amb_k4 = t_amb_k2 * t_amb_k2;

	double f_amb = (kFa * t_amb_k4) + (kFb * t_amb_k2 * t_amb_k) + (kGa * t_amb_k2);

	double t_obj_k = t_amb_k;

	for (int i = 0; i < 3; i++) {
		double t_obj_k2 = t_obj_k * t_obj_k;
		double t_obj_k3 = t_obj_k2 * t_obj_k;
		double t_obj_k4 = t_obj_k2 * t_obj_k2;

		double f_obj = (kFa * t_obj_k4) + (kFb * t_obj_k3) + (kGa * t_obj_k2);

		double df_obj =
			(4.0 * kFa * t_obj_k3) + (3.0 * kFb * t_obj_k2) + (2.0 * kGa * t_obj_k);

		t_obj_k = t_obj_k + (v_ir - (f_obj - f_amb)) / df_obj;
	}
	double t_obj_c = t_obj_k - 273.15;

	return t_obj_c * (1.0 + (double)data->hb / 16384.0);
}

static double mlx90632_calc_object_standard(struct mlx90632_data *data)
{
	double v_ir = mlx90632_preprocess_object(data);
	double t_amb = data->ambient_temp;

	double kFa = (double)data->fa / 17592186044416.0;
	double kGa = (double)data->ga / 1048576.0;

	double t_amb_k = t_amb + 273.15;

	double v_ir_comp = v_ir / (1.0 + kGa * (t_amb - 25.0));

	double t_obj_k = pow((v_ir_comp / kFa) + pow(t_amb_k, 4), 0.25);

	return t_obj_k - 273.15;
}

static int mlx90632_read_extended_channels(const struct device *dev)
{
	struct mlx90632_data *data = dev->data;
	int ret;

	ret = mlx90632_reg_read(dev, MLX90632_RAM_52, &data->ram_52);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_RAM_53, &data->ram_53);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_RAM_54, &data->ram_54);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_RAM_55, &data->ram_55);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_RAM_56, &data->ram_56);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_RAM_57, &data->ram_57);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_RAM_58, &data->ram_58);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_RAM_59, &data->ram_59);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_reg_read(dev, MLX90632_RAM_60, &data->ram_60);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static double mlx90632_calc_ambient_extended(struct mlx90632_data *data)
{

	double am_ta = (double)data->ram_54 / 12.0;
	double vr_ta = (double)data->ram_57 + ((double)data->gb / 1024.0) * am_ta;
	double tmp_ta = (am_ta / vr_ta) * 524288.0;

	double Asub = (double)data->p_t / 17592186044416.0;
	double Bsub = tmp_ta - ((double)data->p_r / 256.0);
	double Ablock = Asub * (Bsub * Bsub);
	double Bblock = (Bsub / (double)data->p_g) * 1048576.0;
	double Cblock = (double)data->p_o / 256.0;

	return Ablock + Bblock + Cblock;
}

static double mlx90632_calc_object_extended(struct mlx90632_data *data)
{
	double t_amb = data->ambient_temp;
	double kKa = (double)data->ka / 1024.0;

	double vr_to = (double)data->ram_57 + kKa * ((double)data->ram_54 / 12.0);

	double s = ((double)(data->ram_52 - data->ram_53 - data->ram_55 + data->ram_56) / 2.0) +
		   (double)(data->ram_58 + data->ram_59);
	double s_to = ((s / 12.0) / vr_to) * 524288.0;

	double kFa = ((double)data->fa / 17592186044416.0) / 2.0;
	double kFb = (double)data->fb / 68719476736.0;
	double kGa = (double)data->ga / 68719476736.0;
	double kHa = (double)data->ha / 16384.0;
	double kHb = (double)data->hb / 1024.0;

	double t_amb_k = data->ambient_temp + 273.15;
	double t_amb_k2 = t_amb_k * t_amb_k;
	double t_amb_k4 = t_amb_k2 * t_amb_k2;

	double f_amb = (kFa * t_amb_k4) + (kFb * t_amb_k2 * t_amb_k) + (kGa * t_amb_k2);

	double t_obj_k = t_amb_k;

	for (int i = 0; i < 3; i++) {
		double t_obj_k2 = t_obj_k * t_obj_k;
		double t_obj_k3 = t_obj_k2 * t_obj_k;
		double t_obj_k4 = t_obj_k2 * t_obj_k2;

		double f_obj = (kFa * t_obj_k4) + (kFb * t_obj_k3) + (kGa * t_obj_k2);
		double df_obj =
			(4.0 * kFa * t_obj_k3) + (3.0 * kFb * t_obj_k2) + (2.0 * kGa * t_obj_k);

		t_obj_k = t_obj_k + (s_to - kHa * (f_obj - f_amb)) / (kHa * df_obj);
	}

	return (t_obj_k - 273.15) - kHb;
}

static void mlx90632_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct mlx90632_data *data = CONTAINER_OF(dwork, struct mlx90632_data, data_work);
	const struct device *dev = data->dev;

	uint16_t reg_status;
	int ret;

	ret = mlx90632_reg_read(dev, MLX90632_REG_STATUS, &reg_status);
	if (ret < 0) {
		data->work_ret = ret;
		k_sem_give(&data->data_sem);
		return;
	}

	if (reg_status & MLX90632_STAT_DATA_RDY) {
		data->work_ret = 0;
		k_sem_give(&data->data_sem);
	} else {
		k_work_reschedule(&data->data_work, K_MSEC(10));
	}
}
static int mlx90632_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mlx90632_data *data = dev->data;
	uint16_t reg_status;
	int ret;

#ifndef CONFIG_MLX90632_MODE_CONTINUOUS
	ret = mlx90632_trigger_measurement(dev);
	if (ret < 0) {
		return ret;
	}
#endif

	k_sem_reset(&data->data_sem);
	k_work_schedule(&data->data_work, K_NO_WAIT);

	ret = k_sem_take(&data->data_sem, K_SECONDS(2));
	if (ret == -EAGAIN) {
		LOG_ERR("Timeout waiting for sensor data");
		return -ETIMEDOUT;
	}

	if (data->work_ret < 0) {
		return data->work_ret;
	}

	uint8_t cycle_pos = (reg_status >> 2) & 0x1F;

	data->last_cycle = cycle_pos;

	if (data->is_extended) {
		ret = mlx90632_read_extended_channels(dev);
		if (ret < 0) {
			return ret;
		}

		data->ambient_temp = mlx90632_calc_ambient_extended(data);

	} else {
		ret = mlx90632_reg_read(dev, MLX90632_RAM_6, &data->ram_6);
		if (ret < 0) {
			return ret;
		}
		ret = mlx90632_reg_read(dev, MLX90632_RAM_9, &data->ram_9);
		if (ret < 0) {
			return ret;
		}
		if (data->is_medical) {
			ret = mlx90632_reg_read(dev, MLX90632_RAM_4, &data->ram_4);
			if (ret < 0) {
				return ret;
			}
			ret = mlx90632_reg_read(dev, MLX90632_RAM_5, &data->ram_5);
			if (ret < 0) {
				return ret;
			}
			ret = mlx90632_reg_read(dev, MLX90632_RAM_7, &data->ram_7);
			if (ret < 0) {
				return ret;
			}
			ret = mlx90632_reg_read(dev, MLX90632_RAM_8, &data->ram_8);
			if (ret < 0) {
				return ret;
			}
		} else {
			if (cycle_pos == 1) {
				ret = mlx90632_reg_read(dev, MLX90632_RAM_4, &data->ram_4);
				if (ret < 0) {
					return ret;
				}
				ret = mlx90632_reg_read(dev, MLX90632_RAM_5, &data->ram_5);
				if (ret < 0) {
					return ret;
				}
			} else {
				ret = mlx90632_reg_read(dev, MLX90632_RAM_7, &data->ram_7);
				if (ret < 0) {
					return ret;
				}
				ret = mlx90632_reg_read(dev, MLX90632_RAM_8, &data->ram_8);
				if (ret < 0) {
					return ret;
				}
			}
		}
		data->ambient_temp = mlx90632_calc_amb(data);
	}

	ret = mlx90632_reg_write(dev, MLX90632_REG_STATUS, reg_status & ~MLX90632_STAT_DATA_RDY);
	if (ret < 0) {
		return ret;
	}

	if (data->is_extended) {
		data->object_temp = mlx90632_calc_object_extended(data);
	} else if (data->is_medical) {
		data->object_temp = mlx90632_calc_object_medical(data);
	} else {
		data->object_temp = mlx90632_calc_object_standard(data);
	}

	return 0;
}

static int mlx90632_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct mlx90632_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_DIE_TEMP:
		sensor_value_from_double(val, data->ambient_temp);
		break;

	case SENSOR_CHAN_AMBIENT_TEMP:
		sensor_value_from_double(val, data->object_temp);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static DEVICE_API(sensor, mlx90632_driver_api) = {
	.sample_fetch = mlx90632_sample_fetch,
	.channel_get = mlx90632_channel_get,
};

#define MLX90632_DEFINE(inst)                                                                      \
	static struct mlx90632_data mlx90632_data_##inst;                                          \
	static const struct mlx90632_config mlx90632_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mlx90632_init, NULL, &mlx90632_data_##inst,             \
				     &mlx90632_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &mlx90632_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(MLX90632_DEFINE)
