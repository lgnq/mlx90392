# MLX90392

## Introduction

This software package is a universal sensor driver package for Melexis's Magnetic Position sensors, compatible with mlx90392. 
And this software package has been connected to the Sensor framework, through the Sensor framework, developers can quickly drive this sensor.

## Support

| Contains equipment          | Magnetometer |
| --------------------------- | ------------ |
| **Communication Interface** |              |
| IIC                         | √            |
| **Work Mode**               |              |
| Polling                     | √            |
| Interruption                |              |
| FIFO                        |              |
| **Power Mode**              |              |
| Power down                  | √            |
| Low power consumption       |              |
| Normal                      | √            |
| High power consumption      |              |
| **Data output rate**        |              |
| **Measuring Range**         | √            |
| **Self-check**              |              |
| **Multi-instance**          |              |

## Instructions for use

### Dependence

- RT-Thread 4.0.0+
- Sensor component
- IIC/SPI driver: mlx90392 devices use IIC for data communication, and need system IIC driver support;

### Get the package

To use the MLX90392 software package, you need to select it in the RT-Thread package management. The specific path is as follows:

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
      mlx90392: Universal 3-axis sensor driver package,support: magnetometer.
              Version (latest)  --->
        [*]   Enable mlx90392 mag
```

**Enable MLX90392 mag**: Configure to turn on the Magnetometer function

**Version**: software package version selection

### Using packages

The initialization function of MLX90392 software package is as follows:

```
int rt_hw_mlx90392_init(const char *name, struct rt_sensor_config *cfg);
```

This function needs to be called by the user. The main functions of the function are:

- Device configuration and initialization (configure interface devices and interrupt pins according to the incoming configuration information);
- Register the corresponding sensor device and complete the registration of the MLX90392 device;

#### Initialization example

```
#include "sensor_melexis_mlx90392.h"

int rt_hw_mlx90392_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)MLX90392_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_hw_mlx90392_init("mlx", &cfg);
    return 0;
}
INIT_APP_EXPORT(rt_hw_mlx90392_port);
```

## Precautions

No

## contact information

Maintenance man:

- [lgnq](https://github.com/lgnq)

- Homepage: <https://github.com/lgnq/mlx90392>
