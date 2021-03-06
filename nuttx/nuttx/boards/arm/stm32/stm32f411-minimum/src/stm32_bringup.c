/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_bringup.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/fs/fs.h>

#include "stm32.h"

#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#include "stm32f411-minimum.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.  
 *                                add by herc  2021.10.26
 *                                            copy from omnibusf4 board file
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *                                add by herc  2021.10.26  
 *                                          copy from omnibusf4 board file
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
  stm32_i2c_register(1);
#if 0
  stm32_i2c_register(1);
  stm32_i2c_register(2);
#endif
}
#else
#  define stm32_i2ctool()
#endif
/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_STM32_I2C
  FAR struct i2c_master_s *i2cbus;

  /* Get the i2c bus instance. */
  i2cbus = stm32_i2cbus_initialize(I2C_BUS);
  if (!i2cbus)
  {
      ferr("ERROR: Failed to initialize I2CBUS%d\n", I2C_BUS);
      return -ENODEV;
  }
#endif


#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)   //add by herc 2021.10.26
  stm32_i2ctool();
#endif


#ifdef CONFIG_DEV_GPIO
  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MPU60X0
  /* Initialize the MPU60x0 device. */
  
  ret = stm32_mpu60x0_initialize(i2cbus);
  if (ret < 0)
  {
    syslog(LOG_ERR, "ERROR: stm32_mpu60x0_initialize() failed: %d\n", ret);
  }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

  return ret;
}
