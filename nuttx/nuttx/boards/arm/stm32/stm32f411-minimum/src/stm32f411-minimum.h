/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32f411-minimum.h
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

#ifndef __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H
#define __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <arch/chip/chip.h>
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LED.  User LED1: the blue LED is a user LED connected to board LED C13
 * corresponding to MCU I/O PC13.
 *
 * - When the I/O is LOW value, the LED is on.
 * - When the I/O is HIGH, the LED is off.
 */

#define GPIO_LED1 \
  (GPIO_PORTB | GPIO_PIN8 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PULLUP | \
   GPIO_SPEED_50MHz)


/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN7)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN6)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN5)



/* GPIO pins used by the GPIO Subsystem   ---add by  herc*/

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

//#define GPIO_IN1          (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN8)
//#define GPIO_OUT1         (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN8)
//#define GPIO_INT1         (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN8)

#define GPIO_IN1          (GPIO_PORTB | GPIO_PIN9 | GPIO_INPUT | GPIO_SPEED_50MHz)
#define GPIO_OUT1         (GPIO_PORTB | GPIO_PIN8 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#define GPIO_INT1         (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN12)


/* 
*   PWM   ----------herc
*/

/* PWM Configuration */

#define STM32F411MINIMUM_PWMTIMER   3
#define STM32F411MINIMUM_PWMCHANNEL 4

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);


/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   PWM initialization               herc
 *
 ****************************************************************************/
#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio        herc
 *
 ****************************************************************************/
#ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#endif


#endif /* __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H */
