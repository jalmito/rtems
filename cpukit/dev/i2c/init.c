/*
 * Copyright (c) 2014 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
  #include "config.h"
#endif

#include <dev/i2c/i2c.h>
#include <dev/i2c/eeprom.h>
#include <dev/i2c/gpio-nxp-pca9535.h>
#include <dev/i2c/switch-nxp-pca9548a.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
//
#include <rtems/libcsupport.h>
#include <rtems/libi2c.h>
#include <bsp/tms570-i2c.h>
#include "bmDriver.h"

//
////#include "tmacros.h"
//
//const char rtems_test_name[] = "I2C 1";
const char i2cdevname[] = "I2C1";
rtems_task Init (rtems_task_argument argument);

#define SPARE_ADDRESS_BITS 3
//
#define DEVICE_SIMPLE_READ_WRITE 0x21
//
//#define DEVICE_EEPROM (1UL << SPARE_ADDRESS_BITS)
//
//#define DEVICE_GPIO_NXP_PCA9535 (2UL << SPARE_ADDRESS_BITS)
//
//#define DEVICE_SWITCH_NXP_PCA9548A (3UL << SPARE_ADDRESS_BITS)

//#define EEPROM_SIZE 512

//typedef struct test_device test_device;

//struct test_device {
//  int (*transfer)(
//    i2c_bus *bus,
//    i2c_msg *msgs,
//    uint32_t msg_count,
//    test_device *dev
//  );
//};
//
//typedef struct {
//  test_device base;
//  char buf[3];
//} test_device_simple_read_write;
//
//typedef struct {
//  test_device base;
//  unsigned current_reg;
//  uint8_t regs[8];
//} test_device_gpio_nxp_pca9535;
//
//typedef struct {
//  test_device base;
//  bool eio;
//  unsigned current_address;
//  uint8_t data[EEPROM_SIZE];
//} test_device_eeprom;
//
//typedef struct {
//  test_device base;
//  uint8_t control;
//} test_device_switch_nxp_pca9548a;
//
//typedef struct {
//  i2c_bus base;
//  unsigned long clock;
//  test_device *devices[4];
//  test_device_simple_read_write simple_read_write;
//  test_device_gpio_nxp_pca9535 gpio_nxp_pca9535;
//  test_device_eeprom eeprom;
//  test_device_switch_nxp_pca9548a switch_nxp_pca9548a;
//} test_bus;

static const char bus_path[] = "/dev/i2c-0";

//static const char gpio_nxp_pca9535_path[] = "/dev/i2c-0.gpio-nxp-pc9535-0";
//
//static const char eeprom_path[] = "/dev/i2c-0.eeprom-0";
//
//static const char switch_nxp_pca9548a_path[] =
//  "/dev/i2c-0.switch-nxp-pca9548a-0";

//static void cyclic_inc(unsigned *val, unsigned cycle)
//{
//  unsigned v = *val;
//  unsigned m = cycle - 1U;
//
//  *val = (v & ~m) + ((v + 1U) & m);
//}
//
//static int test_simple_read_write_transfer(
//  i2c_bus *bus,
//  i2c_msg *msgs,
//  uint32_t msg_count,
//  test_device *base
//)
//{
//  test_device_simple_read_write *dev = (test_device_simple_read_write *) base;
//
//  if (msg_count == 1 && msgs[0].len == sizeof(dev->buf)) {
//    if ((msgs[0].flags & I2C_M_RD) != 0) {
//      memcpy(msgs[0].buf, &dev->buf[0], sizeof(dev->buf));
//    } else {
//      memcpy(&dev->buf[0], msgs[0].buf, sizeof(dev->buf));
//    }
//
//    return 0;
//  } else {
//    return -EIO;
//  }
//}

//static int test_gpio_nxp_pca9535_transfer(
//  i2c_bus *bus,
//  i2c_msg *msgs,
//  uint32_t msg_count,
//  test_device *base
//)
//{
//  test_device_gpio_nxp_pca9535 *dev = (test_device_gpio_nxp_pca9535 *) base;
//  i2c_msg *first = &msgs[0];
//  i2c_msg *second = &msgs[1];
//  int i;
//
//  /* Get command byte */
//  if (
//    msg_count < 1
//      || (first->flags & I2C_M_RD) != 0
//      || first->len < 1
//  ) {
//    return -EIO;
//  }
//
//  dev->current_reg = first->buf[0];
//
//  if (first->len > 1) {
//    /* Write */
//
//    if (msg_count != 1) {
//      return -EIO;
//    }
//
//    for (i = 1; i < first->len; ++i) {
//      dev->regs[dev->current_reg] = first->buf[i];
//
//      /* Output is input */
//      if (dev->current_reg == 2) {
//        dev->regs[0] = first->buf[i];
//      } else if (dev->current_reg == 3) {
//        dev->regs[1] = first->buf[i];
//      }
//
//      cyclic_inc(&dev->current_reg, 2);
//    }
//  } else {
//    /* Read */
//
//    if (msg_count != 2) {
//      return -EIO;
//    }
//
//    for (i = 0; i < second->len; ++i) {
//      second->buf[i] = dev->regs[dev->current_reg];
//      cyclic_inc(&dev->current_reg, 2);
//    }
//  }
//
//  return 0;
//}
//
//static int test_eeprom_transfer(
//  i2c_bus *bus,
//  i2c_msg *msgs,
//  uint32_t msg_count,
//  test_device *base
//)
//{
//  test_device_eeprom *dev = (test_device_eeprom *) base;
//  i2c_msg *msg = &msgs[0];
//  uint32_t i;
//
//  if (dev->eio) {
//    return -EIO;
//  }
//
//  if (msg_count > 0 && (msg->flags & I2C_M_RD) == 0) {
//    if (msg->len < 1) {
//      return -EIO;
//    }
//
//    dev->current_address = msg->buf[0] | ((msg->addr & 0x1) << 8);
//    --msg->len;
//    ++msg->buf;
//  }
//
//  for (i = 0; i < msg_count; ++i) {
//    int j;
//
//    msg = &msgs[i];
//
//    if ((msg->flags & I2C_M_RD) != 0) {
//      for (j = 0; j < msg->len; ++j) {
//        msg->buf[j] = dev->data[dev->current_address];
//        cyclic_inc(&dev->current_address, sizeof(dev->data));
//      }
//    } else {
//      for (j = 0; j < msg->len; ++j) {
//        dev->data[dev->current_address] = msg->buf[j];
//        cyclic_inc(&dev->current_address, 8);
//      }
//    }
//  }
//
//  return 0;
//}
//
//static int test_switch_nxp_pca9548a_transfer(
//  i2c_bus *bus,
//  i2c_msg *msgs,
//  uint32_t msg_count,
//  test_device *base
//)
//{
//  test_device_switch_nxp_pca9548a *dev = (test_device_switch_nxp_pca9548a *) base;
//  uint32_t i;
//
//  for (i = 0; i < msg_count; ++i) {
//    i2c_msg *msg = &msgs[i];
//    int j;
//
//    if ((msg->flags & I2C_M_RD) != 0) {
//      for (j = 0; j < msg->len; ++j) {
//        msg->buf[j] = dev->control;
//      }
//    } else {
//      for (j = 0; j < msg->len; ++j) {
//        dev->control = msg->buf[j];
//      }
//    }
//  }
//
//  return 0;
//}
//
static int test_transfer(i2c_bus *bus, i2c_msg *msgs, uint32_t msg_count)
{
  uint16_t addr;

  addr = msgs[0].addr >> SPARE_ADDRESS_BITS;


  return (*bus->transfer)(bus, msgs, msg_count);
}

static int test_set_clock(i2c_bus *bus, unsigned long clock)
{
//  test_bus *bus = (test_bus *) base;

//  bus->clock = clock;

  return 0;
}

static void test_destroy(i2c_bus *base)
{
  i2c_bus_destroy_and_free(base);
}



void Init(rtems_task_argument arg)
{
uint32_t delay;
  int busnr,rv,fd;
  char buf[3];
  ssize_t n;
  rtems_status_code sc;
  i2c_dev *dev;
  rtems_libi2c_bus_t * bus = tms570_i2c_1;
  struct bme680_field_data bmeFieldData;

 // rtems_libi2c_drv_t tms570_drv_tbl = {
 //     .ops = &tms570_i2c_ops,
 //     .size = sizeof(tms570_drv_tbl)
 // };


  rv = rtems_libi2c_initialize ();

  busnr = rtems_libi2c_register_bus (&bus_path[0],bus);


  fd = rtems_libi2c_register_drv (&i2cdevname[0], &my_drv_tbl ,busnr,BME680_I2C_ADDR_SECONDARY);
  
//  mknod("/dev/i2c-54", mode, MKDEV(rtems_libi2c_major, RTEMS_LIBI2C_MAKE_MINOR(0,0x54)));

  fd = open("/dev/i2c-0.I2C1",O_RDWR);
//  write(fd,&buf[0],3);
  read(fd,(char *)&bmeFieldData,sizeof(struct bme680_field_data));
  close(fd);
//  sc = rtems_i2c_ioctl(fd, I2C_SLAVE, DEVICE_SIMPLE_READ_WRITE);

//  sc = rtems_i2c_write_send_start(fd);
//  sc = rtems_i2c_write(fd,&bus_path[0],3);
//  sc = rtems_i2c_send_addr(fd,1);
//  sc = rtems_i2c_write_bytes(fd, &buf[0], 3);

//  sc = rtems_i2c_send_stop(fd);

  printf( "\n\n*** (͡° ͜ʖ ͡°) ***\n" );
  printf( "(づ｡◕‿‿◕｡)づ\n" );

  printf("%f\t%f", bmeFieldData.temperature, bmeFieldData.pressure);

            for(delay=0;delay<1000000;delay++);

	/**/
  printf( "*** (͡° ͜ʖ ͡°) ***\n" );
  exit( 0 );
}
#define CONFIGURE_MICROSECONDS_PER_TICK 20000

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 7

#define CONFIGURE_MAXIMUM_TASKS             rtems_resource_unlimited(30)
#define CONFIGURE_MAXIMUM_SEMAPHORES        rtems_resource_unlimited(500)
//#define CONFIGURE_MAXIMUM_TASKS 1
//
//#define CONFIGURE_MAXIMUM_SEMAPHORES 1

#define CONFIGURE_INIT_TASK_STACK_SIZE (16*1024)

//#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION
#define CONFIGURE_MAXIMUM_DRIVERS 32
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
