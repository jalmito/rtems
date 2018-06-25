ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am
include $(top_srcdir)/../../bsp.am

dist_project_lib_DATA = ../../../../../../bsps/arm/csb337/start/bsp_specs

if ENABLE_LCD
<<<<<<< HEAD
include_HEADERS += include/sed1356.h
endif

include_bsp_HEADERS =
libbsp_a_CPPFLAGS =

if ENABLE_UMON
include_umondir = $(includedir)/umon
include_umon_HEADERS  = ../../shared/umon/cli.h
include_umon_HEADERS += ../../shared/umon/monlib.h
include_umon_HEADERS += ../../shared/umon/tfs.h

include_rtemsdir = $(includedir)/rtems
include_rtems_HEADERS  = ../../shared/umon/umon.h
=======
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
endif

noinst_PROGRAMS =


if ENABLE_LCD
endif

start.$(OBJEXT): ../../../../../../bsps/arm/csb337/start/start.S
	$(CPPASCOMPILE) -o $@ -c $<
project_lib_DATA = start.$(OBJEXT)

project_lib_DATA += linkcmds
dist_project_lib_DATA += ../../../../../../bsps/arm/csb337/start/linkcmds.csb337
dist_project_lib_DATA += ../../../../../../bsps/arm/csb337/start/linkcmds.csb637


project_lib_LIBRARIES = librtemsbsp.a
librtemsbsp_a_SOURCES =

# startup
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspgetworkarea-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/start/bspstart.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspfatal-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/start/bspreset.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/start/memmap.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/sbrk.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/getentropy/getentropy-cpucounter.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/cpucounter/cpucounterread.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/cpucounter/cpucounterdiff.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/start/pmc.c
librtemsbsp_a_SOURCES +=../../../../../../bsps/arm/csb337/clock/clock.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/btimer/btimer.c
# console
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/legacy-console.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/legacy-console-control.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/legacy-console-select.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/console/uarts.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/console/dbgu.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/console/usart.c
# IRQ
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/irq/irq-default-handler.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/irq/irq.c
if ENABLE_LCD
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/console/sed1356.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/console/fbcons.c
endif
# umon
if ENABLE_UMON
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/umon/monlib.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/umon/tfsDriver.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/umon/umonrtemsglue.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/start/umonsupp.c
endif
if ENABLE_UMON_CONSOLE
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/umon/umoncons.c
endif

# Cache
<<<<<<< HEAD
libbsp_a_SOURCES += ../../../libcpu/shared/src/cache_manager.c
libbsp_a_SOURCES += ../shared/include/arm-cache-l1.h
libbsp_a_SOURCES += ../shared/armv467ar-basic-cache/cache_.h
libbsp_a_CPPFLAGS += -I$(srcdir)/../shared/armv467ar-basic-cache
=======
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cache/cache-cp15.c
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

if HAS_NETWORKING
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/csb337/net/network.c
endif

librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cp15/arm920-mmu.c

include $(top_srcdir)/../../../../automake/local.am
include $(srcdir)/../../../../../../bsps/shared/irq-sources.am
include $(srcdir)/../../../../../../bsps/shared/shared-sources.am
include $(srcdir)/../../../../../../bsps/arm/csb337/headers.am
