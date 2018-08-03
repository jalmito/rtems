/*
 * Copyright (c) 1989, 1991, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)mount.h	8.21 (Berkeley) 5/20/95
 * $FreeBSD: src/sys/sys/mount.h,v 1.198 2005/08/06 01:42:04 ssouhlal Exp $
 */

 
#ifndef _SYS_MOUNT_H_
#define _SYS_MOUNT_H_

#ifndef _KERNEL
#include <sys/ucred.h>
#endif
#include <sys/queue.h>

typedef struct fsid { int32_t val[2]; } fsid_t;	/* filesystem id type */

/*
 * File identifier.
 * These are unique per filesystem on a single machine.
 */
#define	MAXFIDSZ	16

struct fid {
	u_short		fid_len;		/* length of data in bytes */
	u_short		fid_reserved;		/* force longword alignment */
	char		fid_data[MAXFIDSZ];	/* data (variable length) */
};

/*
 * filesystem statistics
 */
#define	MFSNAMELEN	16		/* length of type name including null */
#define	MNAMELEN	88		/* size of on/from name bufs */
#define STATFS_VERSION  0x20030518      /* current version number */
struct statfs {
        uint32_t f_version;             /* structure version number */
        uint32_t f_type;                /* type of filesystem */
        uint64_t f_flags;               /* copy of mount exported flags */
        uint64_t f_bsize;               /* filesystem fragment size */
        uint64_t f_iosize;              /* optimal transfer block size */
        uint64_t f_blocks;              /* total data blocks in filesystem */
        uint64_t f_bfree;               /* free blocks in filesystem */
        int64_t  f_bavail;              /* free blocks avail to non-superuser */
        uint64_t f_files;               /* total file nodes in filesystem */
        int64_t  f_ffree;               /* free nodes avail to non-superuser */
        uint64_t f_syncwrites;          /* count of sync writes since mount */
        uint64_t f_asyncwrites;         /* count of async writes since mount */
        uint64_t f_syncreads;           /* count of sync reads since mount */
        uint64_t f_asyncreads;          /* count of async reads since mount */
        uint64_t f_spare[10];           /* unused spare */
        uint32_t f_namemax;             /* maximum filename length */
        uid_t     f_owner;              /* user that mounted the filesystem */
        fsid_t    f_fsid;               /* filesystem id */
        char      f_charspare[80];          /* spare string space */
        char      f_fstypename[MFSNAMELEN]; /* filesystem type name */
        char      f_mntfromname[MNAMELEN];  /* mounted filesystem */
        char      f_mntonname[MNAMELEN];    /* directory on which mounted */
};

/*
 * User specifiable flags, stored in mnt_flag.
 */
#define MNT_RDONLY      0x0000000000000001ULL /* read only filesystem */
#define MNT_SYNCHRONOUS 0x0000000000000002ULL /* fs written synchronously */
#define MNT_NOEXEC      0x0000000000000004ULL /* can't exec from filesystem */
#define MNT_NOSUID      0x0000000000000008ULL /* don't honor setuid fs bits */
#define MNT_NFS4ACLS    0x0000000000000010ULL /* enable NFS version 4 ACLs */
#define MNT_UNION       0x0000000000000020ULL /* union with underlying fs */
#define MNT_ASYNC       0x0000000000000040ULL /* fs written asynchronously */
#define MNT_SUIDDIR     0x0000000000100000ULL /* special SUID dir handling */
#define MNT_SOFTDEP     0x0000000000200000ULL /* using soft updates */
#define MNT_NOSYMFOLLOW 0x0000000000400000ULL /* do not follow symlinks */
#define MNT_GJOURNAL    0x0000000002000000ULL /* GEOM journal support enabled */
#define MNT_MULTILABEL  0x0000000004000000ULL /* MAC support for objects */
#define MNT_ACLS        0x0000000008000000ULL /* ACL support enabled */
#define MNT_NOATIME     0x0000000010000000ULL /* dont update file access time */
#define MNT_NOCLUSTERR  0x0000000040000000ULL /* disable cluster read */
#define MNT_NOCLUSTERW  0x0000000080000000ULL /* disable cluster write */
#define MNT_SUJ         0x0000000100000000ULL /* using journaled soft updates */
#define MNT_AUTOMOUNTED 0x0000000200000000ULL /* mounted by automountd(8) */

/*
 * NFS export related mount flags.
 */
#define MNT_EXRDONLY    0x0000000000000080ULL   /* exported read only */
#define MNT_EXPORTED    0x0000000000000100ULL   /* filesystem is exported */
#define MNT_DEFEXPORTED 0x0000000000000200ULL   /* exported to the world */
#define MNT_EXPORTANON  0x0000000000000400ULL   /* anon uid mapping for all */
#define MNT_EXKERB      0x0000000000000800ULL   /* exported with Kerberos */
#define MNT_EXPUBLIC    0x0000000020000000ULL   /* public export (WebNFS) */

/*
 * Flags set by internal operations,
 * but visible to the user.
 * XXX some of these are not quite right.. (I've never seen the root flag set)
 */
#define MNT_LOCAL       0x0000000000001000ULL /* filesystem is stored locally */
#define MNT_QUOTA       0x0000000000002000ULL /* quotas are enabled on fs */
#define MNT_ROOTFS      0x0000000000004000ULL /* identifies the root fs */
#define MNT_USER        0x0000000000008000ULL /* mounted by a user */
#define MNT_IGNORE      0x0000000000800000ULL /* do not show entry in df */

#if 0
/*
 * User specifiable flags.
 */
#define	MNT_RDONLY	0x00000001	/* read only filesystem */
#define	MNT_SYNCHRONOUS	0x00000002	/* filesystem written synchronously */
#define	MNT_NOEXEC	0x00000004	/* can't exec from filesystem */
#define	MNT_NOSUID	0x00000008	/* don't honor setuid bits on fs */
#define	MNT_NODEV	0x00000010	/* don't interpret special files */
#define	MNT_UNION	0x00000020	/* union with underlying filesystem */
#define	MNT_ASYNC	0x00000040	/* filesystem written asynchronously */
#define	MNT_NOATIME	0x10000000	/* disable update of file access time */

/*
 * NFS export related mount flags.
 */
#define	MNT_EXRDONLY	0x00000080	/* exported read only */
#define	MNT_EXPORTED	0x00000100	/* filesystem is exported */
#define	MNT_DEFEXPORTED	0x00000200	/* exported to the world */
#define	MNT_EXPORTANON	0x00000400	/* use anon uid mapping for everyone */
#define	MNT_EXKERB	0x00000800	/* exported with Kerberos uid mapping */
#define	MNT_EXPUBLIC	0x20000000	/* public export (WebNFS) */

/*
 * Flags set by internal operations,
 * but visible to the user.
 */
#define	MNT_LOCAL	0x00001000	/* filesystem is stored locally */
#define	MNT_QUOTA	0x00002000	/* quotas are enabled on filesystem */
#define	MNT_ROOTFS	0x00004000	/* identifies the root filesystem */
#define	MNT_USER	0x00008000	/* mounted by a user */
#define	MNT_IGNORE	0x00800000	/* do not show entry in df */

/*
 * External filesystem command modifier flags.
 * Unmount can use the MNT_FORCE flag.
 */
#define	MNT_UPDATE	0x00010000	/* not a real mount, just an update */
#define	MNT_DELEXPORT	0x00020000	/* delete export host lists */
#define	MNT_RELOAD	0x00040000	/* reload filesystem data */
#define	MNT_FORCE	0x00080000	/* force unmount or readonly change */
#endif
/*
 * Generic file handle
 */
struct fhandle {
	fsid_t	fh_fsid;	/* Filesystem id of mount point */
	struct	fid fh_fid;	/* Filesys specific id */
};
typedef struct fhandle	fhandle_t;

#ifdef _KERNEL

#else /* !_KERNEL */

#include <sys/cdefs.h>

#endif /* _KERNEL */

#endif /* !_SYS_MOUNT_H_ */
