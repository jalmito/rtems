ACLOCAL_AMFLAGS = -I aclocal

include $(top_srcdir)/automake/compile.am
include $(top_srcdir)/automake/multilib.am

# librtemscpu
_SUBDIRS = . score rtems sapi posix
_SUBDIRS += dev
_SUBDIRS += dtc/libfdt
_SUBDIRS += libcrypt
_SUBDIRS += libcsupport libblock libfs
_SUBDIRS += libdrvmgr
_SUBDIRS += libnetworking librpc
_SUBDIRS += libpci
_SUBDIRS += libi2c
_SUBDIRS += libmisc
_SUBDIRS += libmd
_SUBDIRS += libgnat
_SUBDIRS += libdl
_SUBDIRS += libstdthreads
_SUBDIRS += libdebugger
_SUBDIRS += wrapup

_SUBDIRS += zlib

# applications
_SUBDIRS += ftpd
_SUBDIRS += telnetd
_SUBDIRS += pppd
_SUBDIRS += mghttpd

if LIBNETWORKING
endif

if LIBDL
<<<<<<< HEAD
include_HEADERS += libdl/dlfcn.h
include_HEADERS += libdl/include/link.h
include_HEADERS += libdl/include/link_elf.h
include_sys_HEADERS += libdl/include/sys/cdefs_elf.h
include_sys_HEADERS += libdl/include/sys/exec_elf.h
include_arch_machinedir = $(includedir)/machine
include_arch_machine_HEADERS =
include_arch_machine_HEADERS += libdl/include/arch/@RTEMS_CPU@/machine/elf_machdep.h
include_rtems_rtldir = $(includedir)/rtems/rtl
include_rtems_rtl_HEADERS =
include_rtems_rtl_HEADERS += libdl/dlfcn-shell.h
include_rtems_rtl_HEADERS += libdl/rtl.h libdl/rtl-allocator.h libdl/rtl-obj-fwd.h
include_rtems_rtl_HEADERS += libdl/rtl-fwd.h libdl/rtl-obj.h libdl/rtl-obj-cache.h
include_rtems_rtl_HEADERS += libdl/rtl-obj-comp.h libdl/rtl-unresolved.h
include_rtems_rtl_HEADERS += libdl/rtl-indirect-ptr.h libdl/rtl-sym.h libdl/rtl-trace.h
include_rtems_rtl_HEADERS += libdl/rap.h libdl/rap-shell.h
=======
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
endif

if LIBDEBUGGER
endif

if !LIBPCI
endif
## should be conditional and only installed for PowerPC, x86, and ARM
## libfs
## POSIX FIFO/pipe
## devfs
if LIBDOSFS
endif

# RFS
# JFFS2
## libblock
## ftpd
if LIBNETWORKING
endif

## bspcmdline
## capture
# tracing headers
## cpuuse
## devnull
## dumpbuf
## rtemsfdt
## monitor
## fb
## mouse
## libqos
## redirector
## shell
if LIBSHELL
endif

## stringto
## testsupport
## i2c
## serdbg
if LIBSERDBG
endif

## stackchk
## libuntar
## fsmount
## Driver manager
if LIBDRVMGR
endif

if LIBGNAT
endif

## HACK: doxygen filter.

include $(top_srcdir)/automake/subdirs.am
include $(top_srcdir)/automake/local.am
include $(srcdir)/headers.am
