#include <bspopts.h>
#include <bsp/tms570-i2c.h>
#include <bsp/irq.h>

#ifdef TMS570_LC43X
#ifdef TMS570_CONFIG_I2C_0
  static const tms570_pin_range tms570_i2c_pins_0  = {
    TMS570_BALL_B2_I2C1_SDA,
    TMS570_BALL_C3_I2C1_SCL
  };

  static tms570_i2c_bus_entry tms570_i2c_entry_0 = {
    .bus = {
      .ops = &tms570_i2c_ops,
      .size = sizeof(tms570_i2c_bus_entry)
    },
    .regs = (volatile tms570_i2c_t*) &TMS570_I2C1,
    .index = 0,
    .pins = &tms570_i2c_pins_0 [0],
    .vector = TMS570_IRQ_I2C_INTERRUPT
  };

  rtems_libi2c_bus_t * const tms570_i2c_0 =
    &tms570_i2c_entry_0.bus;
#endif

#ifdef TMS570_CONFIG_I2C_1
 static tms570_pin_range tms570_i2c_pins_1 = {
        .sda = 0x1 ,//TMS570_BALL_G17_I2C2_SDA,
        .scl = 0x2 //TMS570_BALL_G16_I2C2_SCL
     }; 


tms570_i2c_bus_entry tms570_i2c_entry_1 = {
    .bus = {
      .ops = &tms570_i2c_ops,
      .size = sizeof(tms570_i2c_bus_entry)
    },
    .regs = (volatile tms570_i2c_t*) &TMS570_I2C2,
    .index = 1,
    .pins = &tms570_i2c_pins_1 ,
    .vector = TMS570_IRQ_I2C2_INTERRUPT
  };

extern  rtems_libi2c_bus_t * const tms570_i2c_1 =
    &tms570_i2c_entry_1.bus;
#endif
#endif /* TMS570_LC43X */
