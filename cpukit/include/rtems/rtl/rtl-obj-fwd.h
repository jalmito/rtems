/*
 *  COPYRIGHT (c) 2012 Chris Johns <chrisj@rtems.org>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */
/**
 * @file
 *
 * @ingroup rtems_rtl
 *
 * @brief RTEMS Run-Time Linker ELF Headers
 */

#if !defined (_RTEMS_RTL_OBJ_FWD_H_)
#define _RTEMS_RTL_OBJ_FWD_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * The forward declaration of the obj section structure.
 */
struct rtems_rtl_obj_sect;
typedef struct rtems_rtl_obj_sect rtems_rtl_obj_sect;

/**
 * The forward declaration of the obj depends structure.
 */
struct rtems_rtl_obj_depends;
typedef struct rtems_rtl_obj_depends rtems_rtl_obj_depends;

/**
 * The forward declaration of the obj structure.
 */
struct rtems_rtl_obj;
typedef struct rtems_rtl_obj rtems_rtl_obj;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
