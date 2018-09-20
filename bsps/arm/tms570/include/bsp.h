/**
 * @file bsp.h
 *
 * @ingroup tms570
 *
 * @brief Global BSP definitions.
 */

/*
 * Copyright (c) 2014 Premysl Houdek <kom541000@gmail.com>
 *
 * Google Summer of Code 2014 at
 * Czech Technical University in Prague
 * Zikova 1903/4
 * 166 36 Praha 6
 * Czech Republic
 *
 * Based on LPC24xx and LPC1768 BSP
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_TMS570_BSP_H
#define LIBBSP_ARM_TMS570_BSP_H

#ifndef TMS570_LC43X
#define TMS570_LC43X
#endif

#include <bspopts.h>

#define BSP_FEATURE_IRQ_EXTENSION

#ifndef ASM

#include <rtems.h>
#include <rtems/console.h>
#include <rtems/clockdrv.h>
#include <bsp/default-initial-extension.h>

#ifdef TMS570_LC43X
#define BSP_OSCILATOR_CLOCK 15000000
#define BSP_PLL_OUT_CLOCK 300000000
#else
#define BSP_OSCILATOR_CLOCK 8000000
#define BSP_PLL_OUT_CLOCK 160000000
#endif
/** Define operation count for Tests */
#define OPERATION_COUNT 4

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

   /*
    *RTEMS Network Driver configuration
    */ 
    
struct rtems_bsdnet_ifconfig;

extern int rtems_tms_driver_attach (struct rtems_bsdnet_ifconfig *config, int attaching);
#define RTEMS_BSP_NETWORK_DRIVER_NAME	"tms1"
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH	rtems_tms_driver_attach


/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ASM */

#endif /* LIBBSP_ARM_TMS570_BSP_H */
