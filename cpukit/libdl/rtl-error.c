/*
 *  COPYRIGHT (c) 2012-2018 Chris Johns <chrisj@rtems.org>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */
/**
 * @file
 *
 * @ingroup rtl
 *
 * @brief RTEMS Run-Time Linker Error
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <rtems/rtl/rtl.h>
#include "rtl-error.h"

void
rtems_rtl_set_error (int error, const char* format, ...)
{
  rtems_rtl_data* rtl = rtems_rtl_lock ();
  va_list         ap;
  va_start (ap, format);
  rtl->last_errno = error;
  vsnprintf (rtl->last_error, sizeof (rtl->last_error), format, ap);
  rtems_rtl_unlock ();
  va_end (ap);
}

int
rtems_rtl_get_error (char* message, size_t max_message)
{
<<<<<<< HEAD
  rtems_rtl_data_t* rtl = rtems_rtl_lock ();
  if (rtl != NULL)
  {
    int last_errno = rtl->last_errno;
    strncpy (message, rtl->last_error, sizeof (rtl->last_error));
    rtems_rtl_unlock ();
    return last_errno;
  }

  strncpy(message, "RTL init error", max_message);

  return EIO;
}

void
rtems_rtl_clear_error (void)
{
  rtems_rtl_data_t* rtl = rtems_rtl_lock ();
  if (rtl != NULL)
  {
    rtl->last_errno = 0;
    rtl->last_error[0] = '\0';
    rtems_rtl_unlock ();
  }
=======
  rtems_rtl_data* rtl = rtems_rtl_lock ();
  if (rtl != NULL)
  {
    int last_errno = rtl->last_errno;
    strlcpy (message, rtl->last_error, max_message);
    rtems_rtl_unlock ();
    return last_errno;
  }

  strncpy(message, "RTL init error", max_message);

  return EIO;
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
}
