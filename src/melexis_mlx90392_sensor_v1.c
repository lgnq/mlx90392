/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include "melexis_mlx90392_sensor_v1.h"

#define DBG_TAG "melexis.mlx90392.sensor.v1"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define mlx_dev ((struct mlx90392_device *)sensor->parent.user_data)

static struct mlx90392_device *_mlx90392_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;

    return mlx90392_init(intf->dev_name, i2c_addr);
}

static rt_err_t _mlx90392_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    LOG_D("Setting range is not supported!");
    return RT_EOK;
}

static rt_err_t _mlx90392_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        LOG_D("set mode to POLLING");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", mode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _mlx90392_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    LOG_D("Setting power is not supported!");
    return RT_EOK;

}

static rt_err_t _mlx90392_nop(rt_sensor_t sensor)
{
    mlx90392_nop((struct mlx90392_device *)sensor->parent.user_data);
}

static rt_err_t _mlx90392_reset(rt_sensor_t sensor)
{
    mlx90392_reset((struct mlx90392_device *)sensor->parent.user_data);
}

static RT_SIZE_TYPE _mlx90392_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
   if (sensor->info.type == RT_SENSOR_CLASS_MAG)
   {
       struct mlx90392_xyz_flux xyz;
       if (mlx90392_single_measurement(mlx_dev, &xyz) != RT_EOK)
       {
           return 0;
       }

       data->type = RT_SENSOR_CLASS_MAG;
       data->data.mag.x = xyz.x;
       data->data.mag.y = xyz.y;
       data->data.mag.z = xyz.z;
       data->timestamp = rt_sensor_get_ts();
   }else
   if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
   {
       float temp;
       if (mlx90392_get_temperature(mlx_dev, &temp) != RT_EOK)
       {
           return 0;
       }
       data->type = RT_SENSOR_CLASS_TEMP;
       data->data.temp=temp;
       data->timestamp = rt_sensor_get_ts();
   }
    return 1;
}

static RT_SIZE_TYPE mlx90392_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _mlx90392_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t mlx90392_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t *)args = mlx_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _mlx90392_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _mlx90392_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _mlx90392_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    case MLX90392_CTRL_NOP:
        result = _mlx90392_nop(sensor);
        break;
    case MLX90392_CTRL_RESET:
        result = _mlx90392_reset(sensor);
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    mlx90392_fetch_data,
    mlx90392_control
};

int rt_hw_mlx90392_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    struct mlx90392_device *mlx_dev_temp;
    rt_sensor_t sensor_mps = RT_NULL;

    mlx_dev_temp = _mlx90392_init(&cfg->intf);
    if (mlx_dev_temp == RT_NULL)
    {
        LOG_E("_mlx90392 init err!");
        goto __exit;
    }

    /* MPS sensor register */
    {
        sensor_mps = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mps == RT_NULL)
            goto __exit;

        sensor_mps->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_mps->info.vendor     = RT_SENSOR_VENDOR_MELEXIS;
        sensor_mps->info.model      = "mlx90392";
        sensor_mps->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_mps->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_mps->info.range_max  = 16000;
        sensor_mps->info.range_min  = 2000;
        sensor_mps->info.period_min = 5;

        rt_memcpy(&sensor_mps->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mps->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_mps, name, RT_DEVICE_FLAG_RDWR, mlx_dev_temp);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (mlx_dev_temp)
        mlx90392_deinit(mlx_dev_temp);

    return -RT_ERROR;
}
