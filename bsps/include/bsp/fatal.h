/*
 * Copyright (c) 2012, 2018 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_SHARED_BSP_FATAL_H
#define LIBBSP_SHARED_BSP_FATAL_H

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define BSP_FATAL_CODE_BLOCK(idx) ((unsigned long) (idx) * 256UL)

/**
 * @brief BSP fatal error codes.
 */
typedef enum {
  /* Generic BSP fatal codes */
  BSP_FATAL_INTERRUPT_INITIALIZATION = BSP_FATAL_CODE_BLOCK(0),
  BSP_FATAL_SPURIOUS_INTERRUPT,
  BSP_FATAL_CONSOLE_MULTI_INIT,
  BSP_FATAL_CONSOLE_NO_MEMORY_0,
  BSP_FATAL_CONSOLE_NO_MEMORY_1,
  BSP_FATAL_CONSOLE_NO_MEMORY_2,
  BSP_FATAL_CONSOLE_NO_MEMORY_3,
  BSP_FATAL_CONSOLE_REGISTER_DEV_0,
  BSP_FATAL_CONSOLE_REGISTER_DEV_1,
  BSP_FATAL_CONSOLE_NO_DEV,
  BSP_FATAL_CONSOLE_INSTALL_0,
  BSP_FATAL_CONSOLE_INSTALL_1,
  BSP_FATAL_CONSOLE_REGISTER_DEV_2,

  /* ARM fatal codes */
  BSP_ARM_A9MPCORE_FATAL_CLOCK_IRQ_INSTALL = BSP_FATAL_CODE_BLOCK(1),
  BSP_ARM_A9MPCORE_FATAL_CLOCK_IRQ_REMOVE,
  BSP_ARM_PL111_FATAL_REGISTER_DEV,
  BSP_ARM_PL111_FATAL_SEM_CREATE,
  BSP_ARM_PL111_FATAL_SEM_RELEASE,
  BSP_ARM_A9MPCORE_FATAL_CLOCK_SMP_INIT,
  BSP_ARM_ARMV7M_CPU_COUNTER_INIT,
  BSP_ARM_FATAL_GENERIC_TIMER_CLOCK_IRQ_INSTALL,

  /* LEON3 fatal codes */
  LEON3_FATAL_NO_IRQMP_CONTROLLER = BSP_FATAL_CODE_BLOCK(2),
  LEON3_FATAL_CONSOLE_REGISTER_DEV,
  LEON3_FATAL_CLOCK_INITIALIZATION,
  LEON3_FATAL_INVALID_CACHE_CONFIG_MAIN_PROCESSOR,
  LEON3_FATAL_INVALID_CACHE_CONFIG_SECONDARY_PROCESSOR,
  LEON3_FATAL_CLOCK_NO_IRQMP_TIMESTAMP_SUPPORT,

  /* LPC24XX fatal codes */
  LPC24XX_FATAL_PL111_SET_UP = BSP_FATAL_CODE_BLOCK(3),
  LPC24XX_FATAL_PL111_PINS_SET_UP,
  LPC24XX_FATAL_PL111_PINS_TEAR_DOWN,
  LPC24XX_FATAL_PL111_TEAR_DOWN,

  /* MPC5200 fatal codes */
  MPC5200_FATAL_PCF8563_INVALID_YEAR = BSP_FATAL_CODE_BLOCK(4),
  MPC5200_FATAL_SLICETIMER_0_IRQ_INSTALL,
  MPC5200_FATAL_SLICETIMER_1_IRQ_INSTALL,
  MPC5200_FATAL_TM27_IRQ_INSTALL,
  MPC5200_FATAL_MSCAN_A_INIT,
  MPC5200_FATAL_MSCAN_B_INIT,
  MPC5200_FATAL_MSCAN_A_SET_MODE,
  MPC5200_FATAL_MSCAN_B_SET_MODE,
  MPC5200_FATAL_ATA_DISK_IO_INIT,
  MPC5200_FATAL_ATA_DISK_CREATE,
  MPC5200_FATAL_ATA_DMA_SINGLE_IRQ_INSTALL,
  MPC5200_FATAL_ATA_LOCK_CREATE,
  MPC5200_FATAL_ATA_LOCK_DESTROY,

  /* MPC55XX fatal codes */
  MPC55XX_FATAL_FMPLL_LOCK = BSP_FATAL_CODE_BLOCK(5),
  MPC55XX_FATAL_CLOCK_EMIOS_IRQ_INSTALL,
  MPC55XX_FATAL_CLOCK_EMIOS_PRESCALER,
  MPC55XX_FATAL_CLOCK_EMIOS_INTERVAL,
  MPC55XX_FATAL_CLOCK_PIT_IRQ_INSTALL,
  MPC55XX_FATAL_CONSOLE_GENERIC_COUNT,
  MPC55XX_FATAL_CONSOLE_GENERIC_REGISTER,
  MPC55XX_FATAL_CONSOLE_GENERIC_REGISTER_CONSOLE,
  MPC55XX_FATAL_CONSOLE_ESCI_BAUD,
  MPC55XX_FATAL_CONSOLE_ESCI_ATTRIBUTES,
  MPC55XX_FATAL_CONSOLE_ESCI_IRQ_INSTALL,
  MPC55XX_FATAL_CONSOLE_LINFLEX_BAUD,
  MPC55XX_FATAL_CONSOLE_LINFLEX_ATTRIBUTES,
  MPC55XX_FATAL_CONSOLE_LINFLEX_RX_IRQ_INSTALL,
  MPC55XX_FATAL_CONSOLE_LINFLEX_TX_IRQ_INSTALL,
  MPC55XX_FATAL_CONSOLE_LINFLEX_ERR_IRQ_INSTALL,
  MPC55XX_FATAL_CONSOLE_LINFLEX_RX_IRQ_REMOVE,
  MPC55XX_FATAL_CONSOLE_LINFLEX_TX_IRQ_REMOVE,
  MPC55XX_FATAL_CONSOLE_LINFLEX_ERR_IRQ_REMOVE,
  MPC55XX_FATAL_EDMA_IRQ_INSTALL,
  MPC55XX_FATAL_EDMA_IRQ_REMOVE,

  /* MRM332 fatal codes */
  MRM332_FATAL_SPURIOUS_INTERRUPT = BSP_FATAL_CODE_BLOCK(6),

  /* PowerPC fatal codes */
  PPC_FATAL_EXCEPTION_INITIALIZATION = BSP_FATAL_CODE_BLOCK(7),

  /* Libchip fatal codes */
  DWMAC_FATAL_TOO_MANY_RBUFS_CONFIGURED = BSP_FATAL_CODE_BLOCK(8),

  /* ARM fatal codes */
  ARM_FATAL_L2C_310_UNEXPECTED_ID = BSP_FATAL_CODE_BLOCK(9),
  ARM_FATAL_L2C_310_UNEXPECTED_NUM_WAYS,
  ARM_FATAL_L2C_310_EXCLUSIVE_CONFIG,

  /* QorIQ fatal codes */
  QORIQ_FATAL_SMP_IPI_HANDLER_INSTALL = BSP_FATAL_CODE_BLOCK(10),
  QORIQ_FATAL_FDT_NO_BUS_FREQUENCY,
  QORIQ_FATAL_FDT_NO_CLOCK_FREQUENCY,
  QORIQ_FATAL_FDT_NO_TIMEBASE_FREQUENCY,
  QORIQ_FATAL_RESTART_FAILED,
  QORIQ_FATAL_RESTART_INSTALL_INTERRUPT,
  QORIQ_FATAL_RESTART_INTERRUPT_FAILED,

  /* ATSAM fatal codes */
  ATSAM_FATAL_XDMA_IRQ_INSTALL = BSP_FATAL_CODE_BLOCK(11),
  ATSAM_FATAL_PIO_IRQ_A,
  ATSAM_FATAL_PIO_IRQ_B,
  ATSAM_FATAL_PIO_IRQ_C,
  ATSAM_FATAL_PIO_IRQ_D,
  ATSAM_FATAL_PIO_IRQ_E,
  ATSAM_FATAL_PIO_CONFIGURE_IT,

  /* i.MX fatal codes */
  IMX_FATAL_GENERIC_TIMER_FREQUENCY = BSP_FATAL_CODE_BLOCK(12),

  /* RISC-V fatal codes */
  RISCV_FATAL_NO_TIMEBASE_FREQUENCY_IN_DEVICE_TREE = BSP_FATAL_CODE_BLOCK(13),
  RISCV_FATAL_NO_NS16550_REG_IN_DEVICE_TREE,
  RISCV_FATAL_NO_NS16550_CLOCK_FREQUENCY_IN_DEVICE_TREE,
  RISCV_FATAL_UNEXPECTED_INTERRUPT_EXCEPTION,
  RISCV_FATAL_CLOCK_IRQ_INSTALL,
  RISCV_FATAL_NO_CLINT_REG_IN_DEVICE_TREE,
  RISCV_FATAL_INVALID_HART_REG_IN_DEVICE_TREE,
  RISCV_FATAL_INVALID_CLINT_IRQS_EXTENDED_IN_DEVICE_TREE,
  RISCV_FATAL_NO_PLIC_REG_IN_DEVICE_TREE,
  RISCV_FATAL_INVALID_PLIC_NDEV_IN_DEVICE_TREE,
  RISCV_FATAL_TOO_LARGE_PLIC_NDEV_IN_DEVICE_TREE,
  RISCV_FATAL_INVALID_INTERRUPT_AFFINITY,
  RISCV_FATAL_NO_NS16550_INTERRUPTS_IN_DEVICE_TREE
} bsp_fatal_code;

RTEMS_NO_RETURN static inline void
bsp_fatal( bsp_fatal_code code )
{
  rtems_fatal( RTEMS_FATAL_SOURCE_BSP, (rtems_fatal_code) code );
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_SHARED_BSP_FATAL_H */
