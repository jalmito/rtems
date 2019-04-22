
#ifndef LIBBSP_ARM_TMS570_I2C_H
#define LIBBSP_ARM_TMS570_I2C_H

#include <rtems.h>
#include <rtems/libi2c.h>

//#include <bsp/io.h>
#include <bsp/tms570.h>
#define I2C_READ_BROADCAST 0xF4

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct {
    uint32_t sda;
    uint32_t scl;
} tms570_pin_range;

typedef struct {
  rtems_libi2c_bus_t bus;
  volatile tms570_i2c_t *regs;
  size_t index;
  tms570_pin_range *pins;
  rtems_vector_number vector;
  rtems_id state_update;
  uint8_t *volatile data;
  uint8_t *volatile end;
} tms570_i2c_bus_entry;

extern const rtems_libi2c_bus_ops_t tms570_i2c_ops;

//extern const rtems_libi2c_drv_t tms570_drv_tbl = {
//    .ops = &tms570_i2c_ops,
//    .size = sizeof(rtems_libi2c_bus_ops_t)
//};

//extern rtems_libi2c_drv_t * const tms570_drv_1 = &tms570_drv_tbl;

extern rtems_libi2c_bus_t *tms570_i2c_0;

extern rtems_libi2c_bus_t * const tms570_i2c_1;



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_TMS570_I2C_H */
