pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = lbxz80.pc

EXTRA_DIST = lbxz80.pc.in

AM_CFLAGS = -g  -Wall -fPIC -Wextra -O2 -DLBXZ80VERSION="$(VERSION)"

lib_LTLIBRARIES = liblbxz80.la
liblbxz80_la_SOURCES = lbxz80.c 
liblbxz80_la_LDFLAGS = -version-info @libversion@
include_HEADERS = lbxz80.h
