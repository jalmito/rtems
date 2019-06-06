/**
 * @file
 *
 * @brief Can Bus Implementation
 *
 * @ingroup CanBus
 */

/*
 * Based on dev i2c bus
 *
 * 
 * 
 * 
 * 
 * 
 *
 * 
 * 
 * 
 */

#if HAVE_CONFIG_H
  #include "config.h"
#endif

#include <dev/can/can.h>

#include <rtems/imfs.h>

#include <stdlib.h>
#include <string.h>

void can_bus_obtain(can_bus *bus)
{
  rtems_status_code sc;

  sc = rtems_semaphore_obtain(bus->mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}

void can_bus_release(can_bus *bus)
{
  rtems_status_code sc;

  sc = rtems_semaphore_release(bus->mutex);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}

int can_bus_transfer(can_bus *bus, can_msg *msgs, uint32_t msg_count)
{
  int err;
  uint32_t i;
  uint32_t j;

  _Assert(msg_count > 0);

  for (i = 0, j = 0; i < msg_count; ++i) {
    if ((msgs[i].flags & CAN_M_NOSTART) != 0) {
      if ((msgs[i].flags & CAN_M_RD) != (msgs[j].flags & CAN_M_RD)) {
        return -EINVAL;
      }

      if (msgs[i].addr != msgs[j].addr) {
        return -EINVAL;
      }
    } else {
      j = i;
    }
  }

  can_bus_obtain(bus);
  err = (*bus->transfer)(bus, msgs, msg_count);
  can_bus_release(bus);

  return err;
}

static ssize_t can_bus_read(
  rtems_libio_t *iop,
  void *buffer,
  size_t count
)
{
  can_bus *bus = IMFS_generic_get_context_by_iop(iop);
  can_msg msg = {
    .addr = bus->default_address,
    .flags = CAN_M_RD,
    .len = (uint16_t) count,
    .buf = buffer
  };
  int err;

  if (bus->ten_bit_address) {
    msg.flags |= CAN_M_TEN;
  }

  err = can_bus_transfer(bus, &msg, 1);
  if (err == 0) {
    return msg.len;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static ssize_t can_bus_write(
  rtems_libio_t *iop,
  const void *buffer,
  size_t count
)
{
  can_bus *bus = IMFS_generic_get_context_by_iop(iop);
  can_msg msg = {
    .addr = bus->default_address,
    .flags = 0,
    .len = (uint16_t) count,
    .buf = RTEMS_DECONST(void *, buffer)
  };
  int err;

  if (bus->ten_bit_address) {
    msg.flags |= CAN_M_TEN;
  }

  err = can_bus_transfer(bus, &msg, 1);
  if (err == 0) {
    return msg.len;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static int can_bus_ioctl(
  rtems_libio_t *iop,
  ioctl_command_t command,
  void *arg
)
{
  can_bus *bus = IMFS_generic_get_context_by_iop(iop);
  can_rdwr_ioctl_data *rdwr;
  int err;

  switch (command) {
    case CAN_RDWR:
      rdwr = arg;
      if (rdwr->nmsgs > 0) {
        err = can_bus_transfer(bus, rdwr->msgs, rdwr->nmsgs);
      } else {
        err = 0;
      }
      break;
    case CAN_BUS_OBTAIN:
      can_bus_obtain(bus);
      err = 0;
      break;
    case CAN_BUS_RELEASE:
      can_bus_release(bus);
      err = 0;
      break;
    case CAN_BUS_GET_CONTROL:
      *(can_bus **) arg = bus;
      err = 0;
      break;
    case CAN_FUNCS:
      *(unsigned long *) arg = bus->functionality;
      err = 0;
      break;
    case CAN_RETRIES:
      bus->retries = (unsigned long) arg;
      err = 0;
      break;
    case CAN_TIMEOUT:
      bus->timeout = RTEMS_MILLISECONDS_TO_TICKS(10 * (unsigned long) arg);
      err = 0;
      break;
    case CAN_SLAVE:
    case CAN_SLAVE_FORCE:
      bus->default_address = (unsigned long) arg;
      err = 0;
      break;
    case CAN_TENBIT:
      bus->ten_bit_address = (unsigned long) arg != 0;
      err = 0;
      break;
    case CAN_PEC:
      bus->use_pec = (unsigned long) arg != 0;
      err = 0;
      break;
    case CAN_BUS_SET_CLOCK:
      can_bus_obtain(bus);
      err = (*bus->set_clock)(bus, (unsigned long) arg);
      can_bus_release(bus);
      break;
    default:
      err = -ENOTTY;
      break;
  }

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static const rtems_filesystem_file_handlers_r can_bus_handler = {
  .open_h = rtems_filesystem_default_open,
  .close_h = rtems_filesystem_default_close,
  .read_h = can_bus_read,
  .write_h = can_bus_write,
  .ioctl_h = can_bus_ioctl,
  .lseek_h = rtems_filesystem_default_lseek,
  .fstat_h = IMFS_stat,
  .ftruncate_h = rtems_filesystem_default_ftruncate,
  .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fcntl_h = rtems_filesystem_default_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev
};

static void can_bus_node_destroy(IMFS_jnode_t *node)
{
  can_bus *bus;

  bus = IMFS_generic_get_context_by_node(node);
  (*bus->destroy)(bus);

  IMFS_node_destroy_default(node);
}

static const IMFS_node_control can_bus_node_control = IMFS_GENERIC_INITIALIZER(
  &can_bus_handler,
  IMFS_node_initialize_generic,
  can_bus_node_destroy
);

int can_bus_register(
  can_bus *bus,
  const char *bus_path
)
{
  int rv;

  rv = IMFS_make_generic_node(
    bus_path,
    S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
    &can_bus_node_control,
    bus
  );
  if (rv != 0) {
    (*bus->destroy)(bus);
  }

  return rv;
}

static int can_bus_transfer_default(
  can_bus *bus,
  can_msg *msgs,
  uint32_t msg_count
)
{
  (void) bus;
  (void) msgs;
  (void) msg_count;

  return -EIO;
}

static int can_bus_set_clock_default(can_bus *bus, unsigned long clock)
{
  (void) bus;
  (void) clock;

  return -EIO;
}

static int can_bus_do_init(
  can_bus *bus,
  void (*destroy)(can_bus *bus)
)
{
  rtems_status_code sc;

  sc = rtems_semaphore_create(
    rtems_build_name('C', 'A', 'N', 'B'),
    1,
    RTEMS_BINARY_SEMAPHORE | RTEMS_INHERIT_PRIORITY | RTEMS_PRIORITY,
    0,
    &bus->mutex
  );
  if (sc != RTEMS_SUCCESSFUL) {
    (*destroy)(bus);

    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  bus->transfer = can_bus_transfer_default;
  bus->set_clock = can_bus_set_clock_default;
  bus->destroy = destroy;

  return 0;
}

void can_bus_destroy(can_bus *bus)
{
  rtems_status_code sc;

  sc = rtems_semaphore_delete(bus->mutex);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}

void can_bus_destroy_and_free(can_bus *bus)
{
  can_bus_destroy(bus);
  free(bus);
}

int can_bus_init(can_bus *bus)
{
  memset(bus, 0, sizeof(*bus));

  return can_bus_do_init(bus, can_bus_destroy);
}

can_bus *can_bus_alloc_and_init(size_t size)
{
  can_bus *bus = NULL;

  if (size >= sizeof(*bus)) {
    bus = calloc(1, size);
    if (bus != NULL) {
      int rv;

      rv = can_bus_do_init(bus, can_bus_destroy_and_free);
      if (rv != 0) {
        return NULL;
      }
    }
  }

  return bus;
}
