## Process this file with automake to produce Makefile.in

noinst_LTLIBRARIES = libsparse.la

libsparse_la_SOURCES =	\
	spalloc.c	\
	spbuild.c	\
	spconfig.h	\
	spdefs.h	\
	spextra.c	\
	spfactor.c	\
	spoutput.c	\
	spsolve.c	\
	sputils.c

if !KLU_WANTED
libsparse_la_SOURCES += spsmp.c
endif

AM_CPPFLAGS = @AM_CPPFLAGS@ -I$(top_srcdir)/src/include
AM_CFLAGS = $(STATIC)

MAINTAINERCLEANFILES = Makefile.in

