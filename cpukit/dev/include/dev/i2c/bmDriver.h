#include <rtems.h>
#include <time.h>
#include <rtems/libi2c.h>

#include <rtems/libio.h>
#include <dev/i2c/bme680.h>

static struct bme680_dev gas_sensor;
extern const rtems_libi2c_drv_t my_drv_tbl;
uint8_t bme_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t bme_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void bmi2_delay(uint32_t period);
//rtems_status_code bme_init (rtems_device_major_number major, rtems_device_minor_number minor,void *arg);
