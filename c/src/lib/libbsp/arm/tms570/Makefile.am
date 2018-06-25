##
#
# @file makefile.am
#
# @brief Makefile of LibBSP for the TMS570 boards.
#

ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am
include $(top_srcdir)/../../bsp.am

dist_project_lib_DATA = ../../../../../../bsps/arm/tms570/start/bsp_specs

# ----------------------------
# ------  Headers
# ----------------------------

<<<<<<< HEAD
include_HEADERS = include/bsp.h

nodist_include_HEADERS = ../../shared/include/coverhd.h
nodist_include_HEADERS += include/bspopts.h

nodist_include_bsp_HEADERS = ../../shared/include/bootcard.h

include_bsp_HEADERS =
include_bsp_HEADERS += ../../shared/include/utility.h
include_bsp_HEADERS += ../../shared/include/irq-generic.h
include_bsp_HEADERS += ../../shared/include/irq-info.h
include_bsp_HEADERS += ../../shared/include/stackalloc.h
include_bsp_HEADERS += ../../shared/include/uart-output-char.h
include_bsp_HEADERS += ../shared/include/start.h
include_bsp_HEADERS += include/tms570.h
include_bsp_HEADERS += include/tms570-sci.h
include_bsp_HEADERS += include/irq.h
include_bsp_HEADERS += include/tms570-rti.h
include_bsp_HEADERS += include/tms570-vim.h
include_bsp_HEADERS += include/tms570-pom.h
include_bsp_HEADERS += include/tms570-sci-driver.h
include_bsp_HEADERS += include/system-clocks.h
include_bsp_HEADERS += include/tms570-pinmux.h

include_bsp_HEADERS += include/tms570-pins.h
include_bsp_HEADERS += include/tms570ls3137zwt-pins.h

include_bsp_ti_hercules_HEADERS = include/ti_herc/reg_adc.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_ccmsr.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_crc.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_dcan.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_dcc.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_dma.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_dmm.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_efuse.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_emacc.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_emacm.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_emif.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_esm.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_flash.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_flex_ray.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_gio.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_htu.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_i2c.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_iomm.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_lin.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_mdio.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_n2het.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_pbist.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_pll.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_pmm.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_rti.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_rtp.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_sci.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_tcr.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_tcram.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_vim.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_pom.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_spi.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_stc.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_sys.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_sys2.h
include_bsp_ti_hercules_HEADERS += include/ti_herc/reg_pcr.h

include_HEADERS += ../../shared/include/tm27.h

=======
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
# ----------------------------
# ------  Data
# ----------------------------

start.$(OBJEXT): ../../../../../../bsps/arm/shared/start/start.S
	$(CPPASCOMPILE) -o $@ -c $<
project_lib_DATA = start.$(OBJEXT)

<<<<<<< HEAD
EXTRA_DIST =
EXTRA_DIST += startup/linkcmds.tms570ls3137_hdk
EXTRA_DIST += startup/linkcmds.tms570ls3137_hdk_with_loader
EXTRA_DIST += startup/linkcmds.tms570ls3137_hdk_sdram
EXTRA_DIST += startup/linkcmds.tms570ls3137_hdk_intram
=======
project_lib_DATA += linkcmds
dist_project_lib_DATA += ../../../../../../bsps/arm/tms570/start/linkcmds.tms570ls3137_hdk
dist_project_lib_DATA += ../../../../../../bsps/arm/tms570/start/linkcmds.tms570ls3137_hdk_intram
dist_project_lib_DATA += ../../../../../../bsps/arm/tms570/start/linkcmds.tms570ls3137_hdk_sdram
dist_project_lib_DATA += ../../../../../../bsps/arm/tms570/start/linkcmds.tms570ls3137_hdk_with_loader
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

# ----------------------------
# ------  LibBSP
# ----------------------------

project_lib_LIBRARIES = librtemsbsp.a

librtemsbsp_a_SOURCES =

# Shared
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/getentropy/getentropy-cpucounter.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspfatal-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspgetworkarea-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/sbrk.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/stackalloc.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/cpucounter/cpucounterdiff.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cp15/arm-cp15-set-exception-handler.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cp15/arm-cp15-set-ttb-entries.c

# Startup
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/start/bsp-start-memcpy.S
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/start/bsp-start-init-registers.S
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/bspreset.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/bspstart.c

# POM
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/tms570-pom.c

# IRQ
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/irq/irq-default-handler.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/irq/irq.c

# Console
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/console-termios.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/console/printk-support.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/console/tms570-sci.c

# Clock
librtemsbsp_a_SOURCES +=../../../../../../bsps/arm/tms570/clock/clock.c

# RTC

# GPIO
<<<<<<< HEAD
libbsp_a_SOURCES += pinmux/pinmux.c
# Timer
=======
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/pinmux.c
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

# Benchmark Timer
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/btimer/btimer-cpucounter.c

# Misc
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/cpucounter/cpucounterread.c

# Watchdog

# Cache
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/cache/nocache.c

if TMS570_USE_HWINIT_STARTUP
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/tms570_sys_core.S
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/fail_notification.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/init_emif_sdram.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/init_esm.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/init_pinmux.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/init_system.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/tms570_selftest.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/tms570_selftest_par_can.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/tms570_selftest_par_mibspi.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/tms570_selftest_par_std.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/tms570_selftest_parity.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/tms570_tcram_tests.c

if TMS570_USE_HWINIT_STARTUP
libbsp_a_SOURCES += hwinit/tms570_sys_core.S
libbsp_a_SOURCES += hwinit/fail_notification.c
libbsp_a_SOURCES += hwinit/init_emif_sdram.c
libbsp_a_SOURCES += hwinit/init_esm.c
libbsp_a_SOURCES += hwinit/init_pinmux.c
libbsp_a_SOURCES += hwinit/init_system.c
libbsp_a_SOURCES += hwinit/tms570_selftest.c
libbsp_a_SOURCES += hwinit/tms570_selftest_par_can.c
libbsp_a_SOURCES += hwinit/tms570_selftest_par_mibspi.c
libbsp_a_SOURCES += hwinit/tms570_selftest_par_std.c
libbsp_a_SOURCES += hwinit/tms570_selftest_parity.c
libbsp_a_SOURCES += hwinit/tms570_tcram_tests.c

include_bsp_HEADERS += hwinit/tms570_selftest_parity.h
include_bsp_HEADERS += hwinit/tms570_hwinit.h
include_bsp_HEADERS += hwinit/tms570_selftest.h

# Start hooks
libbsp_a_SOURCES += hwinit/bspstarthooks-hwinit.c
else
# Start hooks
<<<<<<< HEAD
libbsp_a_SOURCES += startup/bspstarthooks.c
=======
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/bspstarthooks-hwinit.c
else
# Start hooks
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/tms570/start/bspstarthooks.c
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
endif

# ----------------------------
# ------  Special Rules
# ----------------------------


include $(top_srcdir)/../../../../automake/local.am
include $(srcdir)/../../../../../../bsps/shared/irq-sources.am
include $(srcdir)/../../../../../../bsps/shared/shared-sources.am
include $(srcdir)/../../../../../../bsps/arm/tms570/headers.am
