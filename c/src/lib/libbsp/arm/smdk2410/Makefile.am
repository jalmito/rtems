ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am
include $(top_srcdir)/../../bsp.am

dist_project_lib_DATA = ../../../../../../bsps/arm/smdk2410/start/bsp_specs


<<<<<<< HEAD
include_HEADERS = ../gp32/include/bsp.h
include_HEADERS += ../gp32/smc/smc.h
include_HEADERS += ../../shared/include/tm27.h

include_bsp_HEADERS =
libbsp_a_CPPFLAGS =

nodist_include_HEADERS = include/bspopts.h
nodist_include_bsp_HEADERS = ../../shared/include/bootcard.h
DISTCLEANFILES = include/bspopts.h
nodist_include_HEADERS += ../../shared/include/coverhd.h

noinst_LIBRARIES = libbspstart.a
libbspstart_a_SOURCES = ../gp32/start/start.S
=======
start.$(OBJEXT): ../../../../../../bsps/arm/smdk2410/start/start.S
	$(CPPASCOMPILE) -o $@ -c $<
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
project_lib_DATA = start.$(OBJEXT)

project_lib_DATA += linkcmds

project_lib_LIBRARIES = librtemsbsp.a
librtemsbsp_a_SOURCES =

# startup
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/smdk2410/start/bspstart.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/smdk2410/start/bspidle.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspfatal-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/smdk2410/start/bspreset.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/smdk2410/start/memmap.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/getentropy/getentropy-cpucounter.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/sbrk.c

librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspgetworkarea-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/cpucounter/cpucounterread.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/cpucounter/cpucounterdiff.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/smdk2410/btimer/btimer.c
librtemsbsp_a_SOURCES +=../../../../../../bsps/arm/smdk2410/clock/clockdrv.c
librtemsbsp_a_SOURCES +=../../../../../../bsps/arm/smdk2410/clock/support.c
# console
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/legacy-console.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/legacy-console-control.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/legacy-console-select.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/smdk2410/console/uart.c
# IRQ
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/irq/irq-default-handler.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/smdk2410/irq/irq.c
# smc
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/smdk2410/smc/smc.c

# Cache
<<<<<<< HEAD
libbsp_a_SOURCES += ../../../libcpu/shared/src/cache_manager.c
libbsp_a_SOURCES += ../shared/include/arm-cache-l1.h
libbsp_a_SOURCES += ../shared/armv467ar-basic-cache/cache_.h
libbsp_a_CPPFLAGS += -I$(srcdir)/../shared/armv467ar-basic-cache
=======
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cache/cache-cp15.c
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cp15/arm920-mmu.c

include $(top_srcdir)/../../../../automake/local.am
include $(srcdir)/../../../../../../bsps/shared/irq-sources.am
include $(srcdir)/../../../../../../bsps/shared/shared-sources.am
include $(srcdir)/../../../../../../bsps/arm/smdk2410/headers.am
