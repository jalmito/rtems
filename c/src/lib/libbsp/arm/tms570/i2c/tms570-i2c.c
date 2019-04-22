//#include <ti_herc/reg_i2c.c>
#include <dev/i2c/i2c.h>
#include <bsp.h>
#include <rtems/error.h>
#include <bsp/tms570-i2c.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>
#include <bsp/system-clocks.h>

#define mg_sleep(x) usleep((x) * 1000)

static void tms570_i2c_handler(void *arg)
{
  tms570_i2c_bus_entry *e = arg;
  volatile tms570_i2c_t *regs = e->regs;
//  unsigned state = regs->IVR;
  uint16_t auxData;
  uint8_t *data = e->data;
  uint8_t *end = e->end;
  bool notify = true;
  int errno;

  switch (regs->IVR) {
    case 0x6:
//      regs->STR |= TMS570_I2C_STR_SCD * 1 ;
//      regs->MDR |=TMS570_I2C_MDR_MST * 1;
//        notify = false;
        break;
    case 0x5:
      /* Data has been transmitted successfully */
      if (data < end) {
        notify = false;
        regs->DXR = *(data);
        data ++;
        e->data = data;
      }
      else
         regs->DXR = *(data);// auxData = e->end;
      break;
    case 0x4:
      /* Data has been received */
      if (data < end) {
        notify = false;
        *data = ((uint8_t)regs->DRR);
         data++;
//         e->data = data;
//        data = data+1;
          e->data = data;
//        if (data < end) {
//            notify = false;
//          if (data + 1 != end) {
//            regs->conset = tms570_I2C_AA;
//          } else {
//            regs->conclr = tms570_I2C_AA;
//          }
//          regs->conclr = tms570_I2C_SI;
//          notify = false
//        }  
      }
      else
         auxData = regs->DRR;

      break;
    case 0x1:
      /* Arbitration has been lost */
    errno=1;
//    notify = false;
      break;
    case 0x3:
      /* ARDY */

      break;
    case 0x2:
      /* No ack */
    notify = false;
    if (regs->STR & TMS570_I2C_STR_NACKSNT != 0)
        regs->STR |= TMS570_I2C_STR_NACKSNT * 1;
      break;
    case 0x0:
      /* unknown */
    notify = false;
    errno=0;
      break;
    default:
      /* Do nothing */
      break;
  }

  /* Notify task if necessary */
  if (notify) {
    bsp_interrupt_vector_disable(e->vector);

    rtems_semaphore_release(e->state_update);
  }
}


static rtems_status_code tms570_i2c_wait(tms570_i2c_bus_entry *e)
{
  bsp_interrupt_vector_enable(e->vector);

  return rtems_semaphore_obtain(e->state_update, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
}

static rtems_status_code  tms570_i2c_init(rtems_libi2c_bus_t *bus)
{    
    uint32_t baud = 100;
    uint32_t prescale;
    uint32_t d;
    uint32_t ck;
    int64_t vclk = 75.000F * 1000000.0F;
    int64_t divider= 0.0F;
    uint32_t temp = 0U;
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
  if (bus != tms570_i2c_1)
      return 1;


    divider = vclk / 8000000.0F;
    prescale = (uint32_t)divider - 1U;

    if(prescale>=2U)
    {
        d = 5U;
    }
    else
    {
        d = (prescale != 0U) ? 6U : 7U;
    }

    temp = 2U * baud * 1000U * (prescale + 1U);
    divider = vclk / ((int64_t)temp);
    ck = (uint32_t)divider - d;

    regs->MDR = TMS570_I2C_MDR_nIRS * 0;//(uint32)((uint32)0U << 5U);
    regs->OAR = 0x21;// own address

    /** - set i2c mode */
    regs->MDR =  TMS570_I2C_MDR_NACKMOD * 0      /* nack mode  */
                   | TMS570_I2C_MDR_FREE * 0      /* free running */
                   | TMS570_I2C_MDR_STT * 0     /* start condition - master mode only */
                   | TMS570_I2C_MDR_STP * 0     /* stop condition   */
                   | TMS570_I2C_MDR_MST * 1     /* Master/Slave mode  */
                   | TMS570_I2C_MDR_TRX * 1     /* Transmitter/receiver */
                   | TMS570_I2C_MDR_XA * 0      /* Expanded address */
                   | TMS570_I2C_MDR_RM * 0        /* repeat mode */
                   | TMS570_I2C_MDR_DLB * 0      /* digital loopback */
                   | TMS570_I2C_MDR_STB * 0      /* start byte - master only */
                   | TMS570_I2C_MDR_FDF * 0      /* free data format */
                   | TMS570_I2C_MDR_BC(0);       /* bit count */
    /** - set i2c extended mode */
    regs->EMDR |= TMS570_I2C_EMDR_IGNACK * 0; /* Ignore Nack Enable/Disable */

    /** - set i2c Backward Compatibility mode */
    regs->EMDR |= TMS570_I2C_EMDR_BCM * 1;

    /** - Disable DMA */
    regs->DMACR = 0x00U;

    /** - set i2c data count */
    regs->CNT = 8U;

    /** - disable all interrupts */
    regs->IMR = 0x00U;

    /** - set prescale */
    regs->PSC = prescale;//8U;

    /** - set clock rate */
    regs->CKH = ck;//36U;
    regs->CKL = ck;//37U;

    /** - set i2c pins functional mode */
    regs->PFNC = (0U);

    /** - set i2c pins default output value */
    regs->DOUT = TMS570_I2C_DOUT_SDAOUT * 0     /* sda pin */
                  | TMS570_I2C_DOUT_SCLOUT * 0 ;     /* scl pin */

    /** - set i2c pins output direction */
    regs->DIR = TMS570_I2C_DIR_SDADIR * 0     /* sda pin */
                 | TMS570_I2C_DIR_SCLDIR * 0;     /* scl pin */

    /** - set i2c pins open drain enable */
    regs->PDR = TMS570_I2C_PDR_SDAPDR * 0      /* sda pin */    //0
                 | TMS570_I2C_PDR_SCLPDR * 0;     /* scl pin */                  //0

    /** - set i2c pins pullup/pulldown enable */
    regs->PDIS &= (0xFFFFFFFC | (TMS570_I2C_PDIS_SDAPDIS * 0     /* sda pin */
                | TMS570_I2C_PDIS_SCLPDIS *0));     /* scl pin */

    /** - set i2c pins pullup/pulldown select */
    regs->PSEL |= TMS570_I2C_PSEL_SDAPSEL * 1     /* sda pin */
                 | TMS570_I2C_PSEL_SCLPSEL * 1;     /* scl pin */

    /** - set interrupt enable */
    regs->IMR    &= 0;     /* Address as slave interrupt      */
    regs->IMR    = TMS570_I2C_IMR_AASEN * 1     /* Address as slave interrupt      */
                    | TMS570_I2C_IMR_SCDEN * 1     /* Stop Condition detect interrupt */
                    | TMS570_I2C_IMR_TXRDYEN * 1     /* Transmit data ready interrupt   */
                    | TMS570_I2C_IMR_RXRDYEN * 1     /* Receive data ready interrupt    */
                    | TMS570_I2C_IMR_ARDYEN * 1    /* Register Access ready interrupt */
                    | TMS570_I2C_IMR_NACKEN * 1     /* No Acknowledgment interrupt    */
                    | TMS570_I2C_IMR_ALEN * 1 ;     /* Arbitration Lost interrupt      */

    regs->MDR |= TMS570_I2C_MDR_nIRS * 1; /* i2c out of reset */
  sc = rtems_semaphore_create (
    rtems_build_name ('I','^' ,'2', 'C' + e->index),
    0,
    RTEMS_SIMPLE_BINARY_SEMAPHORE,
    0,
    &e->state_update
  );



  sc = rtems_interrupt_handler_install(
    e->vector,
    "I2C",
    RTEMS_INTERRUPT_UNIQUE,
    tms570_i2c_handler,
    e
  );
  bsp_interrupt_vector_disable(e->vector);

//    /** - initialize global transfer variables */
//    g_i2cTransfer_t[1U].mode   = (uint32)0U << 4U;
//    g_i2cTransfer_t[1U].length = 0U;     
    return RTEMS_SUCCESSFUL;
}

static rtems_status_code tms570_i2c_send_start(rtems_libi2c_bus_t *bus)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
  uint32_t debug_reg = regs->STR & TMS570_I2C_STR_BB;

//  if(debug_regregs->STR & TMS570_I2C_STR_ARDY == 0)
  if(debug_reg == 0)
  {
    regs->MDR |=TMS570_I2C_MDR_MST * 1;
  }
  else
        sc = tms570_i2c_wait(e);
    return sc;
}

static rtems_status_code tms570_i2c_send_stop(rtems_libi2c_bus_t *bus)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
//  regs->MDR |= TMS570_I2C_MDR_STP * 1; /* set stop condition */
  uint32_t debug_reg = regs->STR & TMS570_I2C_STR_ARDY;

//  if(debug_regregs->STR & TMS570_I2C_STR_ARDY == 0)
  if(debug_reg == 0)
  {
    sc = tms570_i2c_wait(e);
  }
  debug_reg = regs->STR & TMS570_I2C_STR_BB;
  if(debug_reg != 0)
  {
    sc = tms570_i2c_wait(e);
  }
    return sc;

}

static rtems_status_code tms570_i2c_send_addr(rtems_libi2c_bus_t *bus, uint32_t addr, int rw)
{
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;

  regs->SAR = addr ; /* set slave address */
  if (rw){
      regs->MDR &= ~(TMS570_I2C_MDR_TRX);
 // need to write the parameter first that is being read
  }
  else
      regs->MDR |= TMS570_I2C_MDR_TRX * 1;


  return RTEMS_SUCCESSFUL;
}

static int tms570_i2c_read(rtems_libi2c_bus_t *bus, unsigned char *in, int n)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
  uint8_t *data = (uint8_t *)in ;
  uint8_t *end = (uint8_t *)in + n;


  if (n <= 0) {
    return n;
  } else if ((regs->IMR & TMS570_I2C_IMR_RXRDYEN) == 0U) {
    return -RTEMS_IO_ERROR;
  }
  regs->MDR |=TMS570_I2C_MDR_MST * 1;
  
//  else if ((regs->STR & TMS570_I2C_STR_RXRDY) == 0U) {
//    return -RTEMS_IO_ERROR;
//  }

    /* Clear error flags*/
//  regs->STR = TMS570_I2C_STR_AL | TMS570_I2C_STR_NACK;
//  if(tms570_i2c_write(
//  bus,
//  unsigned char *out,
//  int n
//))
//  regs->IMR &= ~(TMS570_I2C_IMR_SCDEN);
  regs->CNT = (uint16_t)n;
  regs->MDR |= TMS570_I2C_MDR_STP * 1;
  regs->MDR |= TMS570_I2C_MDR_STT * 1; /* set start condition */

  e->data = in;
  e->end = in+n;

 // regs->DXR = *in;

  sc = tms570_i2c_wait(e);

 return n;
}
static int tms570_i2c_write(
  rtems_libi2c_bus_t *bus,
  unsigned char *out,
  int n
)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
  unsigned state = 0;

  if (n <= 0) {
    return n;
  }
//  regs->SAR = addr; /* set slave address */
  regs->CNT = (uint16_t)n;
  regs->MDR |= TMS570_I2C_MDR_STP * 1;
  regs->MDR |= TMS570_I2C_MDR_STT * 1; /* set start condition */
//  regs->MDR |= TMS570_I2C_MDR_TRX * 1U; /*Set Transmit/Receive Mode*/
  if((regs->STR & TMS570_I2C_STR_TXRDY) == 0)
    sc = tms570_i2c_wait(e);
    //return -RTEMS_IO_ERROR;

  /* Setup transmit buffer */
  e->data = out + 1;
  e->end = out + n;


  /* Transmit first byte */
  regs->DXR = *out;

//  regs->IMR |= TMS570_I2C_IMR_TXRDYEN;
  sc = tms570_i2c_wait(e);

  return n;
}

static int tms570_i2c_set_transfer_mode(
  rtems_libi2c_bus_t *bus,
  const rtems_libi2c_tfr_mode_t *mode
)
{
  return -RTEMS_NOT_IMPLEMENTED;
}

static int tms570_i2c_ioctl(rtems_libi2c_bus_t *bus, int cmd, void *arg)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
  uint32_t debug_reg; 
  int rv = -1;
  const rtems_libi2c_tfr_mode_t *tm = (const rtems_libi2c_tfr_mode_t *) arg;

  switch (cmd) {
    case I2C_TENBIT:
        if(arg != 0)
            regs->MDR |= TMS570_I2C_MDR_XA * 1;
        else
            regs->MDR &= (0xFFFFFFFF & (~TMS570_I2C_MDR_XA));
        rv=0;
        break;
    case I2C_READ_BROADCAST:
        debug_reg = (regs->STR  & TMS570_I2C_STR_AD0);
        if(debug_reg != 0){ 
           e->data = rv;
           e->end = rv+1;
           sc = tms570_i2c_wait(e);
           return (0x00000000 | rv);
        }
        else 
           return rv;
            break;
    case I2C_RDWR:
            regs->MDR |= TMS570_I2C_MDR_RM * 1;
            rv=0;
            break;
    case I2C_SLAVE:
            regs->SAR = arg;
            break;
    case RTEMS_LIBI2C_IOCTL_SET_TFRMODE:
      rv = tms570_i2c_set_transfer_mode(bus, tm);
      break;
    default:
      rv = -RTEMS_NOT_DEFINED;
      break;
  }

  return rv;
}

const rtems_libi2c_bus_ops_t tms570_i2c_ops = {
  .init         = tms570_i2c_init,
  .send_start   = tms570_i2c_send_start,
  .send_stop    = tms570_i2c_send_stop,
  .send_addr    = tms570_i2c_send_addr,
  .read_bytes   = tms570_i2c_read,
  .write_bytes  = tms570_i2c_write,
  .ioctl        = tms570_i2c_ioctl
};

