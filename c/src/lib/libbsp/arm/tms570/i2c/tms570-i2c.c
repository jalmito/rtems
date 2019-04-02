//#include <ti_herc/reg_i2c.c>
#include <dev/i2c/i2c.h>
#include <bsp.h>
#include <bsp/tms570-i2c.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>
#include <bsp/system-clocks.h>

static void tms570_i2c_handler(void *arg)
{
  tms570_i2c_bus_entry *e = arg;
  volatile tms570_i2c_t *regs = e->regs;
  unsigned state = regs->IVR;
  uint8_t *data = e->data;
  uint8_t *end = e->end;
  bool notify = true;

  switch (state) {
    case 0x05U:
      /* Data has been transmitted successfully */
      if (data != end) {
        regs->DXR = *data;
        regs->IMR |= TMS570_I2C_IMR_TXRDYEN;
        ++data;
        e->data = data;
      }
      break;
    case 0x04U:
      /* Data has been received */
      if (data != end) {
        *data = (uint8_t) regs->DRR;
        ++data;
        if (data != end) {
//          if (data + 1 != end) {
//            regs->conset = tms570_I2C_AA;
//          } else {
//            regs->conclr = tms570_I2C_AA;
//          }
//          regs->conclr = tms570_I2C_SI;
//          notify = false;
          e->data = data;
        } else {
          /* This is an error and should never happen */
        }
      }
      break;
    case 0x1U:
      /* Arbitration has been lost */
      break;
    case 0x2U:
      /* No Acknowledgement */
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
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
  if (bus != tms570_i2c_1)
      return 1;


    regs->MDR = TMS570_I2C_MDR_nIRS * 0;//(uint32)((uint32)0U << 5U);

    /** - set i2c mode */
    regs->MDR =  TMS570_I2C_MDR_NACKMOD * 0      /* nack mode  */
                   | TMS570_I2C_MDR_FREE * 0      /* free running */
                   | TMS570_I2C_MDR_STT * 0     /* start condition - master mode only */
                   | TMS570_I2C_MDR_STP * 1     /* stop condition   */
                   | TMS570_I2C_MDR_MST * 1     /* Master/Slave mode  */
                   | TMS570_I2C_MDR_TRX * 1     /* Transmitter/receiver */
                   | TMS570_I2C_MDR_XA * 0      /* Expanded address */
                   | TMS570_I2C_MDR_RM * 0        /* repeat mode */
                   | TMS570_I2C_MDR_DLB * 0      /* digital loopback */
                   | TMS570_I2C_MDR_STB * 0      /* start byte - master only */
                   | TMS570_I2C_MDR_FDF * 0      /* free data format */
                   | TMS570_I2C_MDR_BC(0);       /* bit count */
    /** - set i2c extended mode */
    regs->EMDR |= TMS570_I2C_EMDR_IGNACK * 1; /* Ignore Nack Enable/Disable */

    /** - set i2c Backward Compatibility mode */
    regs->EMDR |= TMS570_I2C_EMDR_BCM * 1;

    /** - Disable DMA */
    regs->DMACR = 0x00U;

    /** - set i2c data count */
    regs->CNT = 8U;

    /** - disable all interrupts */
    regs->IMR = 0x00U;

    /** - set prescale */
    regs->PSC = 8U;

    /** - set clock rate */
    regs->CKH = 36U;
    regs->CKL = 37U;

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
//    regs->PDIS &= (0xFFFFFFFC | (TMS570_I2C_PDIS_SDAPDIS * 0     /* sda pin */
//                | TMS570_I2C_PDIS_SCLPDIS *0));     /* scl pin */
//
//    /** - set i2c pins pullup/pulldown select */
//    regs->PSEL = TMS570_I2C_PSEL_SDAPSEL * 1     /* sda pin */
//                 | TMS570_I2C_PSEL_SCLPSEL * 1;     /* scl pin */

    /** - set interrupt enable */
    regs->IMR    = TMS570_I2C_IMR_AASEN * 0     /* Address as slave interrupt      */
                    | TMS570_I2C_IMR_SCDEN * 0     /* Stop Condition detect interrupt */
                    | TMS570_I2C_IMR_TXRDYEN * 1     /* Transmit data ready interrupt   */
                    | TMS570_I2C_IMR_RXRDYEN * 1     /* Receive data ready interrupt    */
                    | TMS570_I2C_IMR_ARDYEN * 0    /* Register Access ready interrupt */
                    | TMS570_I2C_IMR_NACKEN * 1     /* No Acknowledgment interrupt    */
                    | TMS570_I2C_IMR_ALEN *1 ;     /* Arbitration Lost interrupt      */

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
  regs->MDR |= TMS570_I2C_MDR_STT * 1; /* set start condition */

   return RTEMS_SUCCESSFUL;
}

static rtems_status_code tms570_i2c_send_stop(rtems_libi2c_bus_t *bus)
{
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
  regs->MDR |= TMS570_I2C_MDR_STP * 1; /* set stop condition */
  /* Wait */

   return RTEMS_SUCCESSFUL;
}

static rtems_status_code tms570_i2c_send_addr(rtems_libi2c_bus_t *bus, uint32_t addr, int rw)
{
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;


  regs->SAR = addr; /* set stop condition */
  regs->MDR &= ~TMS570_I2C_MDR_TRX;
  regs->MDR |= TMS570_I2C_MDR_TRX * ((rw != 0) ? 1U : 0U); /*Set Transmit/Receive Mode*/

  return RTEMS_SUCCESSFUL;
}

static int tms570_i2c_read(rtems_libi2c_bus_t *bus, unsigned char *in, int n)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
  uint8_t *data = in;
  uint8_t *end = in + n;


  if (n <= 0) {
    return n;
  } else if ((regs->IMR & TMS570_I2C_IMR_RXRDYEN) == 0U) {
    return -RTEMS_IO_ERROR;
  }else if ((regs->STR & TMS570_I2C_STR_RXRDY) == 0U) {
    return -RTEMS_IO_ERROR;
  }

    /* Clear error flags*/
  regs->STR = TMS570_I2C_STR_AL | TMS570_I2C_STR_NACK;

  /* Setup receive buffer */
  e->data = data;
  e->end = end;

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
  if((regs->STR & TMS570_I2C_STR_TXRDY) == 0)
    return -RTEMS_IO_ERROR;

  /* Setup transmit buffer */
  e->data = out + 1;
  e->end = out + n;

  /* Transmit first byte */
  regs->DXR = (uint32_t)*out;

  regs->IMR |= TMS570_I2C_IMR_TXRDYEN;
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
  tms570_i2c_bus_entry *e = (tms570_i2c_bus_entry *) bus;
  volatile tms570_i2c_t *regs = e->regs;
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

