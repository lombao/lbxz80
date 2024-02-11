#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

###
m4_define([v_maj], [2])
m4_define([v_min], [7])
m4_define([v_mic], [3])
m4_define([project_version], [v_maj.v_min.v_mic])


###
AC_PREREQ([2.69])
AC_INIT([lbxz80], [project_version], [cesar.lombao@gmail.com])
AC_CONFIG_SRCDIR([src/lbxz80.c])
AC_CONFIG_AUX_DIR([build-aux])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIRS([m4])

# Checks for programs.
AC_PROG_CC
AC_PROG_MAKE_SET
AM_PROG_AR

# Checks for header files.
AC_CHECK_HEADERS([stdint.h stdlib.h stdio.h time.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT8_T
AC_TYPE_UINT16_T
AC_TYPE_UINT8_T

# Checks for library functions.

AM_INIT_AUTOMAKE([1.11 -Wall -Werror foreign])
AC_CONFIG_FILES([src/Makefile Makefile])
AC_CONFIG_FILES([src/lbxz80.pc])
AC_CONFIG_FILES([src/lbxz80.h])
LT_INIT([shared disable-static])

AC_SUBST([libversion],[v_maj:v_min])

AC_OUTPUT

echo "
	LBXZ80 ${VERSION} has been configured
	PREFIX: ${prefix}
	You can know do
	make
	sudo make install
"