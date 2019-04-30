#include <dev/i2c/bmDriver.h>
#define mg_sleep(x) usleep((x) * 1000)
//struct bme680_dev gas_sensor;
static rtems_device_minor_number minor_bme;

//uint32_t SwizzleData(uint32_t word) {
//		return
//			(((word << 24U) & 0xFF000000U) |
//	 		((word <<  8U) & 0x00FF0000U)  |
//			((word >>  8U) & 0x0000FF00U)  |
//			((word >> 24U) & 0x000000FFU));
//}

uint8_t bme_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  char commval[len + 1];
  int intlen=0;
  rtems_libio_rw_args_t rwargs;
  rwargs.buffer = (char *) data;
  rwargs.count = (uint32_t) len;
  unsigned char cmd = (unsigned char) reg_addr ;
  rtems_status_code rsc;
  intlen |= (int) (0xFF & len);
/*Clean buffer from broadcasted data*/
//     if(rtems_libi2c_ioctl (8311, I2C_READ_BROADCAST) > 0)
//         while(rtems_libi2c_ioctl (8311, I2C_READ_BROADCAST) > 0)
  rsc = rtems_libi2c_start_write_bytes (minor_bme,
			       (unsigned char *)&reg_addr,
                                1 );
  rsc = rtems_libi2c_start_read_bytes (minor_bme,
			       (unsigned char *)data,
                               intlen );
//  read(3,(unsigned char *)data,(int) len);
//
  if( rsc < 0)
    return rsc;
  else
      return RTEMS_SUCCESSFUL;
 //   rsc = read(fd,rwargs.buffer,rwargs.count);
//printf('stargting');


}

uint8_t bme_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  char commval[len + 1];
  rtems_libio_rw_args_t rwargs;
  rwargs.buffer = (char *) data;
  rwargs.count = (uint32_t) len;
  unsigned char cmd = (unsigned char) reg_addr ;
  rtems_status_code rsc;

//  data = ((uint8_t *) SwizzleData((uint32_t*) data)+6);
  commval[0] = reg_addr;
  memcpy(&commval[1],data,len);
/*Clean buffer from broadcasted data*/
//     if(rtems_libi2c_ioctl (8311, I2C_READ_BROADCAST) > 0)
//         while(rtems_libi2c_ioctl (8311, I2C_READ_BROADCAST) > 0)
    rsc = rtems_libi2c_start_write_bytes (minor_bme,
			       (unsigned char *)&commval[0],
                               (int) len + 1 );
//  rsc = write(3,(unsigned char *)data,(int) len);
  if( rsc < 0)
    return rsc;
  else
      return RTEMS_SUCCESSFUL;
 //   rsc = read(fd,rwargs.buffer,rwargs.count);


}

void bmi2_delay(uint32_t period)
{
 mg_sleep(period);   
}
rtems_status_code bme_init (rtems_device_major_number major, rtems_device_minor_number minor,
          void *arg)
{
  int8_t bme_result;
  uint8_t set_required_settings;
//  struct bme680_dev gas_sensor;// = (struct bme680_dev *) arg;
  minor_bme = minor;
  gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;//minor;
  gas_sensor.intf = BME680_I2C_INTF;
  gas_sensor.read = bme_read;
  gas_sensor.write = bme_write;
  gas_sensor.delay_ms = bmi2_delay;

  gas_sensor.amb_temp = 25;

  bme_result = bme680_init(&gas_sensor);



    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE; 

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
        | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    bme_result= bme680_set_sensor_settings(set_required_settings,&gas_sensor);

    /* Set the power mode */
    bme_result= bme680_set_sensor_mode(&gas_sensor);



  if (bme_result == BME680_OK)
      return RTEMS_SUCCESSFUL;
  else
      return -bme_result;
}
// rtems_status_code bme_open (void *arg)
//{
//  int8_t bme_result;
//  struct bme680_dev *gas_sensor = (bme680_dev *) arg;
//  
//  if (bme_result < 0)
//      return bme_result;
//  else
//      return RTEMS_SUCCESSFUL;
//}

rtems_status_code bme_close (void *arg)
{
  int8_t bme_result;

  return RTEMS_SUCCESSFUL;

}

rtems_status_code bme_read_data (rtems_device_major_number major, rtems_device_minor_number minor,
          void *arg)
{   
  int8_t result;
  struct bme680_field_data data;

  rtems_libio_rw_args_t *rwargs = arg;
  float *tpData = (float *)rwargs->buffer;

    result = bme680_get_sensor_data(&data, &gas_sensor);
    tpData[0] = data.temperature;
    tpData[1] = data.pressure;

    return result;
}
rtems_status_code bme_write_data (rtems_device_major_number major, rtems_device_minor_number minor,
          void *arg)
{
  int result;

  rtems_libio_rw_args_t *rwargs = arg;

  result = rtems_libi2c_start_write_bytes (minor, (unsigned char *)rwargs->buffer, rwargs->count); 

}


/* Our ops just dispatch to the registered drivers */
const rtems_driver_address_table bme_libi2c_io_ops = {
  .initialization_entry =  bme_init,
  .read_entry =            bme_read_data,
  .write_entry =           bme_write_data,
};

const rtems_libi2c_drv_t my_drv_tbl = {
  .ops =                  &bme_libi2c_io_ops,
  .size =                 sizeof (my_drv_tbl),
};
