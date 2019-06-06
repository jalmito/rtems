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
#include <rtems/score/assert.h>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

static ssize_t can_dev_read(
  rtems_libio_t *iop,
  void *buffer,
  size_t count
)
{
  can_dev *dev = IMFS_generic_get_context_by_iop(iop);
  ssize_t n;

  n = (*dev->read)(dev, buffer, count, iop->offset);
  if (n >= 0) {
    iop->offset += n;

    return n;
  } else {
    rtems_set_errno_and_return_minus_one(-n);
  }
}

static ssize_t can_dev_write(
  rtems_libio_t *iop,
  const void *buffer,
  size_t count
)
{
  can_dev *dev = IMFS_generic_get_context_by_iop(iop);
  ssize_t n;

  n = (*dev->write)(dev, buffer, count, iop->offset);
  if (n >= 0) {
    iop->offset += n;

    return n;
  } else {
    rtems_set_errno_and_return_minus_one(-n);
  }
}

static int can_dev_ioctl(
  rtems_libio_t *iop,
  ioctl_command_t command,
  void *arg
)
{
  can_dev *dev = IMFS_generic_get_context_by_iop(iop);
  int err;

  err = (*dev->ioctl)(dev, command, arg);

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static int can_dev_fstat(
  const rtems_filesystem_location_info_t *loc,
  struct stat *buf
)
{
  can_dev *dev = IMFS_generic_get_context_by_location(loc);

  buf->st_size = (*dev->get_size)(dev);
  buf->st_blksize = (*dev->get_block_size)(dev);

  return IMFS_stat(loc, buf);
}

static const rtems_filesystem_file_handlers_r can_dev_handler = {
  .open_h = rtems_filesystem_default_open,
  .close_h = rtems_filesystem_default_close,
  .read_h = can_dev_read,
  .write_h = can_dev_write,
  .ioctl_h = can_dev_ioctl,
  .lseek_h = rtems_filesystem_default_lseek_file,
  .fstat_h = can_dev_fstat,
  .ftruncate_h = rtems_filesystem_default_ftruncate,
  .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fcntl_h = rtems_filesystem_default_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev
};

static void can_dev_node_destroy(IMFS_jnode_t *node)
{
  can_dev *dev;

  dev = IMFS_generic_get_context_by_node(node);
  (*dev->destroy)(dev);

  IMFS_node_destroy_default(node);
}

static const IMFS_node_control can_dev_node_control = IMFS_GENERIC_INITIALIZER(
  &can_dev_handler,
  IMFS_node_initialize_generic,
  can_dev_node_destroy
);

int can_dev_register(
  can_dev *dev,
  const char *dev_path
)
{
  int rv;

  rv = IMFS_make_generic_node(
    dev_path,
    S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
    &can_dev_node_control,
    dev
  );
  if (rv != 0) {
    (*dev->destroy)(dev);
  }

  return rv;
}

static ssize_t can_dev_read_default(
  can_dev *dev,
  void *buf,
  size_t n,
  off_t offset
)
{
  (void) dev;
  (void) buf;
  (void) n;
  (void) offset;

  return -EIO;
}

static ssize_t can_dev_write_default(
  can_dev *dev,
  const void *buf,
  size_t n,
  off_t offset
)
{
  (void) dev;
  (void) buf;
  (void) n;
  (void) offset;

  return -EIO;
}

static int can_dev_ioctl_default(
  can_dev *dev,
  ioctl_command_t command,
  void *arg
)
{
  (void) dev;
  (void) command;
  (void) arg;

  return -ENOTTY;
}

static off_t can_dev_get_size_default(can_dev *dev)
{
  (void) dev;

  return 0;
}

static blksize_t can_dev_get_block_size_default(can_dev *dev)
{
  (void) dev;

  return 1;
}

static int can_dev_do_init(
  can_dev *dev,
  const char *bus_path,
  uint16_t address,
  void (*destroy)(can_dev *dev)
)
{
  int rv;

  dev->bus_fd = open(bus_path, O_RDWR);
  if (dev->bus_fd < 0) {
    (*destroy)(dev);

    return -1;
  }

  rv = ioctl(dev->bus_fd, CAN_BUS_GET_CONTROL, &dev->bus);
  if (rv != 0) {
    (*destroy)(dev);

    return -1;
  }

  dev->read = can_dev_read_default;
  dev->write = can_dev_write_default;
  dev->ioctl = can_dev_ioctl_default;
  dev->get_size = can_dev_get_size_default;
  dev->get_block_size = can_dev_get_block_size_default;
  dev->destroy = destroy;
  dev->address = address;

  return 0;
}

void can_dev_destroy(can_dev *dev)
{
  int rv;

  rv = close(dev->bus_fd);
  _Assert(dev->bus_fd < 0 || rv == 0);
  (void) rv;
}

void can_dev_destroy_and_free(can_dev *dev)
{
  can_dev_destroy(dev);
  free(dev);
}

int can_dev_init(can_dev *dev, const char *bus_path, uint16_t address)
{
  return can_dev_do_init(dev, bus_path, address, can_dev_destroy);
}

can_dev *can_dev_alloc_and_init(
  size_t size,
  const char *bus_path,
  uint16_t address
)
{
  can_dev *dev = NULL;

  if (size >= sizeof(*dev)) {
    dev = calloc(1, size);
    if (dev != NULL) {
      int rv;

      rv = can_dev_do_init(dev, bus_path, address, can_dev_destroy_and_free);
      if (rv != 0) {
        return NULL;
      }
    }
  }

  return dev;
}
