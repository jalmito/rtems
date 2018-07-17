include $(top_srcdir)/automake/compile.am

#%dirutils.obj: %dirutils.o
#	$(LD) -r -o $@ $^
#project_lib_PROGRAMS = dirutils.rel
#dirutils_rel_SOURCES = dirutils.c
#dirutils_rel_CPPFLAGS = $(AM_CPPFLAGS)
#dirutils_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

if LIBNETWORKING
project_lib_LIBRARIES = libnfs.a

# if you have CEXP set this variable to 'YES'
# and some "help" info will be compiled in.
libnfs_a_SOURCES = rpc.c rpc.h
libnfs_a_SOURCES += portmap_xdr.c portmap_rpc.h
libnfs_a_SOURCES += mount_xdr.c xdr.c mount.h
libnfs_a_SOURCES += xdr_pbuf.c xdr_pbuf.h

# With CEXP
#libnfs_a_SOURCES += dirutils.c rpcio.modini.c nfs.modini.c cexphelp.c

#include_HEADERS = librtemsNfs.h

# PGMS=${ARCH}/nfs.obj ${ARCH}/rpcio.obj ${ARCH}/dirutils.obj

#libnfs_a_CPPFLAGS = $(AM_CPPFLAGS)
#libnfs_a_CPPFLAGS += -I$(srcdir)
#libnfs_a_CPPFLAGS += -UHAVE_CEXP

#%nfs.obj: %nfs.o %nfs.modini.o
#	$(LD) -r -o $@ $^ -L../$(ARCH) -lnfsprot
project_lib_PROGRAMS = nfs.rel
nfs_rel_SOURCES = nfs.c nfs.h 
nfs_rel_CPPFLAGS = $(AM_CPPFLAGS) -I$(srcdir)
nfs_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)
nfs_rel_LDADD = libnfs.a

#%rpcio.obj: %rpcio.o %sock_mbuf.o %xdr_mbuf.o %rpcio.modini.o
#	$(LD) -r -o $@ $^
#project_lib_PROGRAMS += rpcio.rel
#rpcio_rel_SOURCES = rpcio.c sock_mbuf.c xdr_pbuf.c
#rpcio_rel_CPPFLAGS = $(AM_CPPFLAGS)
#rpcio_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

l#ibnfs_a_SOURCES += mount_xdr.c xdr.c

libnfs_a_CPPFLAGS += -I$(srcdir)
libnfs_a_CPPFLAGS += -UHAVE_CEXP
endif


#if RPCTOOLS
#$(srcdir)mount_xdr.c: mount.x 
#	(cd $(srcdir) && $(RPCGEN) -c -C mount.x \
#	> mount_xdr.c)
#
#$(srcdir)nfs_xdr.c: nfs.x
#	(cd $(srcdir) && $(RPCGEN) -c -C nfs.x \
#	> nfs_xdr.c)
#$(srcdir)portmap_rpc.h: portmap.x
#	(cd $(srcdir) && $(RPCGEN) -h -C portmap.x > portmap_rpc.h)
#
#$(srcdir)portmap_xdr.c: portmap.x portmap_rpc.h
#	(cd $(srcdir) && $(RPCGEN) -c -C portmap.x \
#	> portmap_xdr.c)
#endif

include $(srcdir)/preinstall.am
include $(top_srcdir)/automake/local.am
