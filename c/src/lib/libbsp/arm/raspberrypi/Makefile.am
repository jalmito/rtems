##
#
# @file
#
# @brief Makefile of LibBSP for the raspberrypi board.
#

ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am
include $(top_srcdir)/../../bsp.am

dist_project_lib_DATA = ../../../../../../bsps/arm/raspberrypi/start/bsp_specs

###############################################################################
# Header                                                     #
###############################################################################

<<<<<<< HEAD
include_HEADERS = include/bsp.h
include_HEADERS += ../../shared/include/tm27.h

nodist_include_HEADERS = ../../shared/include/coverhd.h \
    include/bspopts.h

nodist_include_bsp_HEADERS = ../../shared/include/bootcard.h

include_bsp_HEADERS =
include_bsp_HEADERS += ../../../libbsp/shared/include/mm.h
include_bsp_HEADERS += ../../shared/include/utility.h
include_bsp_HEADERS += ../../shared/include/irq-generic.h
include_bsp_HEADERS += ../../shared/include/irq-info.h
include_bsp_HEADERS += ../../shared/include/stackalloc.h
include_bsp_HEADERS += ../../shared/include/uart-output-char.h
include_bsp_HEADERS += ../shared/include/linker-symbols.h
include_bsp_HEADERS += ../shared/include/start.h
include_bsp_HEADERS += ../shared/include/arm-cp15-start.h
include_bsp_HEADERS += ../shared/include/arm-errata.h
include_bsp_HEADERS += ../shared/lpc/include/lpc-timer.h
include_bsp_HEADERS += ../shared/lpc/include/lpc-dma.h
include_bsp_HEADERS += ../shared/include/arm-release-id.h
include_bsp_HEADERS += include/irq.h
include_bsp_HEADERS += include/mmu.h
include_bsp_HEADERS += include/usart.h
include_bsp_HEADERS += include/raspberrypi.h

include_libcpu_HEADERS =  ../../../libcpu/arm/shared/include/arm-cp15.h
=======
noinst_HEADERS = ../../../../../../bsps/arm/raspberrypi/console/font_data.h
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

###############################################################################
# Data                                                       #
###############################################################################

start.$(OBJEXT): ../../../../../../bsps/arm/shared/start/start.S
	$(CPPASCOMPILE) -o $@ -c $<
project_lib_DATA = start.$(OBJEXT)

project_lib_DATA += linkcmds

###############################################################################
# LibBSP                                                     #
###############################################################################

project_lib_LIBRARIES = librtemsbsp.a

librtemsbsp_a_SOURCES =

# Shared
<<<<<<< HEAD
libbsp_a_SOURCES += ../../shared/bootcard.c
libbsp_a_SOURCES += ../../shared/bspclean.c
libbsp_a_SOURCES += ../../shared/bspgetworkarea.c
libbsp_a_SOURCES += ../../shared/bsplibc.c
libbsp_a_SOURCES += ../../shared/bsppost.c
libbsp_a_SOURCES += ../../shared/bsppredriverhook.c
libbsp_a_SOURCES += ../../shared/bsppretaskinghook.c
libbsp_a_SOURCES += ../../shared/cpucounterread.c
libbsp_a_SOURCES += ../../shared/cpucounterdiff.c
libbsp_a_SOURCES += ../../shared/gnatinstallhandler.c
libbsp_a_SOURCES += ../../shared/sbrk.c
libbsp_a_SOURCES += ../../shared/src/stackalloc.c
libbsp_a_SOURCES += ../shared/startup/bsp-start-memcpy.S
libbsp_a_SOURCES += ../shared/arm-cp15-set-ttb-entries.c
if BSP_START_IN_HYP_SUPPORT
libbsp_a_SOURCES += ../shared/startup/bsp-start-in-hyp-support.S
=======
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/getentropy/getentropy-cpucounter.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspfatal-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/cpucounter/cpucounterread.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/cpucounter/cpucounterdiff.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/sbrk.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/stackalloc.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/start/bsp-start-memcpy.S
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cp15/arm-cp15-set-ttb-entries.c
if BSP_START_IN_HYP_SUPPORT
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/start/bsp-start-in-hyp-support.S
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
endif

# Startup
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/bspstart.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/cmdline.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/bspgetworkarea.c
if HAS_SMP
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/bspsmp.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/bspsmp_init.c
endif

# Restart
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/bspreset.c

# IRQ
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cp15/arm-cp15-set-exception-handler.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/irq/irq.c

# Console
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/legacy-console.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/legacy-console-control.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/console/console-config.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/console/console_select.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/console/usart.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/console/fb.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/console/fbcons.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/console/outch.c

# Mailbox
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/mailbox.c

# VideoCore
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/vc.c

# clock
librtemsbsp_a_SOURCES +=../../../../../../bsps/arm/raspberrypi/clock/clockdrv.c

# Timer
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/timer.c

# GPIO
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/gpio/gpio-support.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/gpio/rpi-gpio.c

# RTC

# SSP

# I2C
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/i2c/i2c.c

# SPI
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/spi/spi.c

# Cache
<<<<<<< HEAD
libbsp_a_SOURCES += ../../../libcpu/shared/src/cache_manager.c
libbsp_a_SOURCES += ../shared/include/arm-cache-l1.h
libbsp_a_SOURCES += ../shared/armv467ar-basic-cache/cache_.h
libbsp_a_CPPFLAGS += -I$(srcdir)/../shared/armv467ar-basic-cache
=======
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cache/cache-cp15.c
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

# Start hooks
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/bspstarthooks.c

# LIBMM
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/raspberrypi/start/mm_config_table.c

###############################################################################
#                  Special Rules                                              #
###############################################################################


include $(top_srcdir)/../../../../automake/local.am
include $(srcdir)/../../../../../../bsps/shared/irq-sources.am
include $(srcdir)/../../../../../../bsps/shared/shared-sources.am
include $(srcdir)/../../../../../../bsps/arm/raspberrypi/headers.am
