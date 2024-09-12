/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include <rtthread.h>
#include "mlx90392.h"

/* Default configuration, please change according to the actual situation, support i2c and spi device name */
#define MLX90392_DEVICE_NAME  "i2c2"

/* Test function */
static int mlx90392_test()
{
    struct mlx90392_device *dev;
    int i;

    /* Initialize mlx90392, The parameter is RT_NULL, means auto probing for i2c*/
    dev = mlx90392_init(MLX90392_DEVICE_NAME, RT_NULL);

    if (dev == RT_NULL)
    {
        rt_kprintf("mlx90392 init failed\n");
        return -1;
    }
    rt_kprintf("mlx90392 init succeed\n");
    struct mlx90392_xyz_flux xyz;
    for (i = 0; i < 5; i++)
    {
        mlx90392_single_measurement(dev,&xyz);
        rt_kprintf("xyz.x = %3d xyz.y = %3d, xyz.z = %3d\n", xyz.x, xyz.y, xyz.z);
        rt_thread_mdelay(100);
    }

    mlx90392_deinit(dev);

    return 0;
}
MSH_CMD_EXPORT(mlx90392_test, mlx90392 sensor test function);
