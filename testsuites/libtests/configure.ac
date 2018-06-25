## Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([rtems-c-src-tests-libtests],[_RTEMS_VERSION],[https://devel.rtems.org/newticket])
AC_CONFIG_SRCDIR([cpuuse])
AC_CONFIG_HEADER([config.h])
RTEMS_TOP([../..],[..])
RTEMS_SOURCE_TOP
RTEMS_BUILD_TOP

RTEMS_CANONICAL_TARGET_CPU

AM_INIT_AUTOMAKE([no-define foreign subdir-objects 1.12.2])
AM_MAINTAINER_MODE

RTEMS_ENABLE_CXX
RTEMS_ENV_RTEMSBSP
RTEMS_CHECK_RTEMS_TEST_NO_PAUSE

RTEMS_PROJECT_ROOT

RTEMS_PROG_CC_FOR_TARGET
RTEMS_PROG_CXX_FOR_TARGET

RTEMS_CANONICALIZE_TOOLS

RTEMS_CHECK_CUSTOM_BSP(RTEMS_BSP)
RTEMS_CHECK_CXX(RTEMS_BSP)
RTEMS_CHECK_CPUOPTS([RTEMS_NETWORKING])
RTEMS_CHECK_CPUOPTS([RTEMS_POSIX_API])

AC_PROG_CXX

AC_PROG_LN_S
AC_PATH_PROG([PAX],[pax],no)
<<<<<<< HEAD

AS_IF([test "x$PAX" = "xno"],[
	AC_MSG_ERROR([pax is missing.])
])
=======
AC_PATH_PROG([GZIP],[gzip],no)
AC_PATH_PROG([XZ],[xz],no)
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf

AS_IF([test "x$PAX" = "xno"],[
	AC_MSG_ERROR([pax is missing.])
])

AM_CONDITIONAL(TARTESTS,test "$as_ln_s" = "ln -s" && test -n "$PAX" && test -n "$GZIP")
AM_CONDITIONAL(TARTEST_XZ,test -n "$XZ")

AM_CONDITIONAL(HAS_CXX,test "$rtems_cv_HAS_CPLUSPLUS" = "yes")
AM_CONDITIONAL([HAS_COMPLEX],[test "$ac_cv_header_complex_h" = yes])
AM_CONDITIONAL(NETTESTS,test "$rtems_cv_RTEMS_NETWORKING" = "yes")
AM_CONDITIONAL(HAS_POSIX,test x"${rtems_cv_RTEMS_POSIX_API}" = x"yes")

# Must match the list in cpukit.
AC_MSG_CHECKING([whether CPU supports libdl])
case $RTEMS_CPU in
  arm | i386 | m68k | mips | moxie | powerpc | sparc)
   TEST_LIBDL=yes ;;
  # bfin has an issue to resolve with libdl. See ticket #2252
  bfin)
   HAVE_LIBDL=no ;;
  # lm32 has an issue to resolve with libdl. See ticket #2283
  lm32)
   HAVE_LIBDL=no ;;
  # v850 has an issue to resolve with libdl. See ticket #2260
  v850)
   HAVE_LIBDL=no ;;
  *)
   TEST_LIBDL=no ;;
esac
AC_MSG_RESULT([$TEST_LIBDL])

AS_IF([test x"$TEST_LIBDL" = x"yes"],[
  AC_CHECK_PROG(RTEMS_LD_CHECK,rtems-ld,yes)
  if test x"$RTEMS_LD_CHECK" != x"yes" ; then
    TEST_LIBDL=no
  fi
  AC_CHECK_PROG(RTEMS_SYMS_CHECK,rtems-syms,yes)
  if test x"$RTEMS_SYMS_CHECK" != x"yes" ; then
    TEST_LIBDL=no
  fi
])

AM_CONDITIONAL(DLTESTS,[test x"$TEST_LIBDL" = x"yes"])

<<<<<<< HEAD
# Explicitly list all Makefiles here
AC_CONFIG_FILES([Makefile
defaultconfig01/Makefile
pwdgrp02/Makefile
shell01/Makefile
pwdgrp01/Makefile
crypt01/Makefile
sha/Makefile
i2c01/Makefile
newlib01/Makefile
block17/Makefile
exit02/Makefile
exit01/Makefile
utf8proc01/Makefile
md501/Makefile
sparsedisk01/Makefile
block16/Makefile
mghttpd01/Makefile
block15/Makefile
block14/Makefile
block13/Makefile
rbheap01/Makefile
syscall01/Makefile
flashdisk01/Makefile
block01/Makefile
block02/Makefile
block03/Makefile
block04/Makefile
block05/Makefile
block06/Makefile
block07/Makefile
block08/Makefile
block09/Makefile
block10/Makefile
block11/Makefile
block12/Makefile
bspcmdline01/Makefile
cpuuse/Makefile
devfs01/Makefile
devfs02/Makefile
devfs03/Makefile
devfs04/Makefile
deviceio01/Makefile
devnullfatal01/Makefile
dl01/Makefile
dl02/Makefile
dl03/Makefile
dl04/Makefile
dl05/Makefile
dumpbuf01/Makefile
ftp01/Makefile
gxx01/Makefile
heapwalk/Makefile
malloctest/Makefile
malloc02/Makefile
malloc03/Makefile
malloc04/Makefile
monitor/Makefile
monitor02/Makefile
mouse01/Makefile
uid01/Makefile
putenvtest/Makefile
rtems++/Makefile
rtmonuse/Makefile
stackchk/Makefile
stackchk01/Makefile
stringto01/Makefile
tar01/Makefile
tar02/Makefile
tar03/Makefile
termios/Makefile
termios01/Makefile
termios02/Makefile
termios03/Makefile
termios04/Makefile
termios05/Makefile
termios06/Makefile
termios07/Makefile
termios08/Makefile
top/Makefile
tztest/Makefile
capture01/Makefile
POSIX/Makefile
math/Makefile
mathf/Makefile
mathl/Makefile
complex/Makefile
])
=======
# Must match the list in cpukit.
AC_MSG_CHECKING([whether CPU supports libdebugger])
case $RTEMS_CPU in
  arm | i386)
   TEST_LIBDEBUGGER=yes ;;
  *)
   TEST_LIBDEBUGGER=no ;;
esac
AC_MSG_RESULT([$TEST_LIBDEBUGGER])
AM_CONDITIONAL(DEBUGGERTESTS,[test x"$TEST_LIBDEBUGGER" = x"yes"])

# BSP Test configuration
RTEMS_TEST_CHECK([block01])
RTEMS_TEST_CHECK([block02])
RTEMS_TEST_CHECK([block03])
RTEMS_TEST_CHECK([block04])
RTEMS_TEST_CHECK([block05])
RTEMS_TEST_CHECK([block06])
RTEMS_TEST_CHECK([block07])
RTEMS_TEST_CHECK([block08])
RTEMS_TEST_CHECK([block09])
RTEMS_TEST_CHECK([block10])
RTEMS_TEST_CHECK([block11])
RTEMS_TEST_CHECK([block12])
RTEMS_TEST_CHECK([block13])
RTEMS_TEST_CHECK([block14])
RTEMS_TEST_CHECK([block15])
RTEMS_TEST_CHECK([block16])
RTEMS_TEST_CHECK([block17])
RTEMS_TEST_CHECK([bspcmdline01])
RTEMS_TEST_CHECK([calloc])
RTEMS_TEST_CHECK([capture01])
RTEMS_TEST_CHECK([clock_gettime])
RTEMS_TEST_CHECK([close])
RTEMS_TEST_CHECK([complex])
RTEMS_TEST_CHECK([cpuuse])
RTEMS_TEST_CHECK([crypt01])
RTEMS_TEST_CHECK([debugger01])
RTEMS_TEST_CHECK([defaultconfig01])
RTEMS_TEST_CHECK([devfs01])
RTEMS_TEST_CHECK([devfs02])
RTEMS_TEST_CHECK([devfs03])
RTEMS_TEST_CHECK([devfs04])
RTEMS_TEST_CHECK([deviceio01])
RTEMS_TEST_CHECK([devnullfatal01])
RTEMS_TEST_CHECK([dl01])
RTEMS_TEST_CHECK([dl02])
RTEMS_TEST_CHECK([dl03])
RTEMS_TEST_CHECK([dl04])
RTEMS_TEST_CHECK([dl05])
RTEMS_TEST_CHECK([dl06])
RTEMS_TEST_CHECK([dumpbuf01])
RTEMS_TEST_CHECK([dup2])
RTEMS_TEST_CHECK([exit01])
RTEMS_TEST_CHECK([exit02])
RTEMS_TEST_CHECK([fcntl])
RTEMS_TEST_CHECK([flashdisk01])
RTEMS_TEST_CHECK([flockfile])
RTEMS_TEST_CHECK([fork])
RTEMS_TEST_CHECK([free])
RTEMS_TEST_CHECK([fstat])
RTEMS_TEST_CHECK([ftp01])
RTEMS_TEST_CHECK([ftrylockfile])
RTEMS_TEST_CHECK([funlockfile])
RTEMS_TEST_CHECK([getentropy01])
RTEMS_TEST_CHECK([getlogin])
RTEMS_TEST_CHECK([getpwnam])
RTEMS_TEST_CHECK([getpwuid])
RTEMS_TEST_CHECK([gettimeofday])
RTEMS_TEST_CHECK([getuid])
RTEMS_TEST_CHECK([gxx01])
RTEMS_TEST_CHECK([heapwalk])
RTEMS_TEST_CHECK([htonl])
RTEMS_TEST_CHECK([i2c01])
RTEMS_TEST_CHECK([iconv])
RTEMS_TEST_CHECK([iconv_close])
RTEMS_TEST_CHECK([iconv_open])
RTEMS_TEST_CHECK([kill])
RTEMS_TEST_CHECK([libfdt01])
RTEMS_TEST_CHECK([longjmp])
RTEMS_TEST_CHECK([lseek])
RTEMS_TEST_CHECK([lstat])
RTEMS_TEST_CHECK([malloc])
RTEMS_TEST_CHECK([malloc02])
RTEMS_TEST_CHECK([malloc03])
RTEMS_TEST_CHECK([malloc04])
RTEMS_TEST_CHECK([malloctest])
RTEMS_TEST_CHECK([math])
RTEMS_TEST_CHECK([mathf])
RTEMS_TEST_CHECK([mathl])
RTEMS_TEST_CHECK([md501])
RTEMS_TEST_CHECK([mghttpd01])
RTEMS_TEST_CHECK([monitor])
RTEMS_TEST_CHECK([monitor02])
RTEMS_TEST_CHECK([mouse01])
RTEMS_TEST_CHECK([nanosleep])
RTEMS_TEST_CHECK([networking01])
RTEMS_TEST_CHECK([newlib01])
RTEMS_TEST_CHECK([open])
RTEMS_TEST_CHECK([pipe])
RTEMS_TEST_CHECK([posix_memalign])
RTEMS_TEST_CHECK([putenvtest])
RTEMS_TEST_CHECK([pwdgrp01])
RTEMS_TEST_CHECK([pwdgrp02])
RTEMS_TEST_CHECK([rbheap01])
RTEMS_TEST_CHECK([read])
RTEMS_TEST_CHECK([readv])
RTEMS_TEST_CHECK([realloc])
RTEMS_TEST_CHECK([rtmonuse])
RTEMS_TEST_CHECK([setjmp])
RTEMS_TEST_CHECK([sha])
RTEMS_TEST_CHECK([shell01])
RTEMS_TEST_CHECK([sigaddset])
RTEMS_TEST_CHECK([sigdelset])
RTEMS_TEST_CHECK([sigemptyset])
RTEMS_TEST_CHECK([sigfillset])
RTEMS_TEST_CHECK([sigismember])
RTEMS_TEST_CHECK([sigprocmask])
RTEMS_TEST_CHECK([sparsedisk01])
RTEMS_TEST_CHECK([spi01])
RTEMS_TEST_CHECK([stackchk])
RTEMS_TEST_CHECK([stackchk01])
RTEMS_TEST_CHECK([stat])
RTEMS_TEST_CHECK([stringto01])
RTEMS_TEST_CHECK([syscall01])
RTEMS_TEST_CHECK([tar01])
RTEMS_TEST_CHECK([tar02])
RTEMS_TEST_CHECK([tar03])
RTEMS_TEST_CHECK([termios])
RTEMS_TEST_CHECK([termios01])
RTEMS_TEST_CHECK([termios02])
RTEMS_TEST_CHECK([termios03])
RTEMS_TEST_CHECK([termios04])
RTEMS_TEST_CHECK([termios05])
RTEMS_TEST_CHECK([termios06])
RTEMS_TEST_CHECK([termios07])
RTEMS_TEST_CHECK([termios08])
RTEMS_TEST_CHECK([termios09])
RTEMS_TEST_CHECK([top])
RTEMS_TEST_CHECK([tztest])
RTEMS_TEST_CHECK([uid01])
RTEMS_TEST_CHECK([unlink])
RTEMS_TEST_CHECK([utf8proc01])
RTEMS_TEST_CHECK([vfork])
RTEMS_TEST_CHECK([wait])
RTEMS_TEST_CHECK([waitpid])
RTEMS_TEST_CHECK([write])
RTEMS_TEST_CHECK([writev])

AC_CONFIG_FILES([Makefile])
>>>>>>> e8b28ba0047c533b842f9704c95d0e76dcb16cbf
AC_OUTPUT
