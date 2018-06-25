/*
<<<<<<< HEAD
 *  COPYRIGHT (c) 2012-2016 Chris Johns <chrisj@rtems.org>
=======
 *  COPYRIGHT (c) 2012-2016, 2018 Chris Johns <chrisj@rtems.org>
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */
/**
 * @file
 *
 * @ingroup rtems_rtld
 *
 * @brief RTEMS Run-Time Link Editor
 *
 * This is the RTL implementation.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <stdio.h>

#include <rtems/rtl/rtl.h>
#include "rtl-elf.h"
#include "rtl-error.h"
#include "rtl-unwind.h"
#include "rtl-unwind-dw2.h"

/*
 * These interfaces are not exported outside the GCC source.
 */
void __register_frame (void *begin);
void __deregister_frame (void *begin);

bool
<<<<<<< HEAD
rtems_rtl_elf_unwind_dw2_parse (const rtems_rtl_obj_t* obj,
                                const char*            name,
                                uint32_t               flags)
=======
rtems_rtl_elf_unwind_dw2_parse (const rtems_rtl_obj* obj,
                                const char*          name,
                                uint32_t             flags)
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
{
  return
    ((flags & RTEMS_RTL_OBJ_SECT_CONST) != 0) &&
    ((strcmp(name, ".eh_frame") == 0) ||
     (strncmp(name, ".gcc_except_table.", sizeof (".gcc_except_table.") - 1) == 0));
}

bool
<<<<<<< HEAD
rtems_rtl_elf_unwind_dw2_register (const rtems_rtl_obj_t* obj)
{
  rtems_rtl_obj_sect_t* sect = rtems_rtl_obj_find_section (obj, ".eh_frame");
=======
rtems_rtl_elf_unwind_dw2_register (const rtems_rtl_obj* obj)
{
  rtems_rtl_obj_sect* sect = rtems_rtl_obj_find_section (obj, ".eh_frame");
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

  if (sect != NULL && sect->size > 0 && sect->base != NULL)
  {
    __register_frame (sect->base);
  }

  return true;
}

<<<<<<< HEAD
bool rtems_rtl_elf_unwind_dw2_deregister (const rtems_rtl_obj_t* obj)
{
  rtems_rtl_obj_sect_t* sect = rtems_rtl_obj_find_section (obj, ".eh_frame");
=======
bool rtems_rtl_elf_unwind_dw2_deregister (const rtems_rtl_obj* obj)
{
  rtems_rtl_obj_sect* sect = rtems_rtl_obj_find_section (obj, ".eh_frame");
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

  if (sect != NULL && sect->size > 0 && sect->base != NULL)
  {
    __deregister_frame (sect->base);
  }

  return true;
}
