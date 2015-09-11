pushdef([MY_PACKAGE_VERSION],[0.0.0-esyscmd(date +%Y%m%d|tr -d '\n')])
AC_INIT([[xmlfs]],[MY_PACKAGE_VERSION],[[amok@codestation.org]])
dnl It is expected that client applications have link to yaal's _aux.
dnl This macro has to be here due to primitivism of libtoolize.
AC_CONFIG_AUX_DIR([_aux])
m4_include([_aux/client-configure.m4])
YAAL_CONFIGURE_CLIENT_INIT(xmlfs,MY_PACKAGE_VERSION)
PKG_CHECK_MODULES(FUSE, [fuse >= 2.3])
AC_CHECK_HEADERS([xattr.h sys/xattr.h])
AC_CHECK_MEMBER(fuse_operations.write_buf, AC_DEFINE([HAVE_FUSE_OPERATIONS_WRITE_BUF],[],[fuse_operations has write_buf member.]))
AC_CHECK_MEMBER(fuse_operations.read_buf, AC_DEFINE([HAVE_FUSE_OPERATIONS_READ_BUF],[],[fuse_operations has read_buf member.]))
AC_CHECK_MEMBER(fuse_operations.flock, AC_DEFINE([HAVE_FUSE_OPERATIONS_FLOCK],[],[fuse_operations has flock member.]))
AC_CHECK_MEMBER(fuse_operations.fallocate, AC_DEFINE([HAVE_FUSE_OPERATIONS_FALLOCATE],[],[fuse_operations has fallocate member.]))
AC_CHECK_MEMBER(fuse_operations.ioctl, AC_DEFINE([HAVE_FUSE_OPERATIONS_IOCTL],[],[fuse_operations has ioctl member.]))
AC_CHECK_MEMBER(fuse_operations.poll, AC_DEFINE([HAVE_FUSE_OPERATIONS_POLL],[],[fuse_operations has poll member.]))
AC_CHECK_MEMBER(fuse_conn_info.want, AC_DEFINE([HAVE_FUSE_CONN_INFO_WANT],[],[fuse_conn_info has want member.]))
AC_CHECK_MEMBER(fuse_file_info.flock_release, AC_DEFINE([HAVE_FUSE_FILE_INFO_FLOCK_RELEASE],[],[fuse_file_info has flock_release member.]))
YAAL_CONFIGURE_CLIENT_FINALIZE
