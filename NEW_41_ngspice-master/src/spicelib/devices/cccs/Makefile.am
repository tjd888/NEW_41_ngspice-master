## Process this file with automake to produce Makefile.in

noinst_LTLIBRARIES = libcccs.la

libcccs_la_SOURCES =	\
	cccs.c		\
	cccsask.c	\
	cccsdefs.h	\
	cccsext.h	\
	cccsinit.c	\
	cccsinit.h	\
	cccsitf.h	\
	cccsload.c	\
	cccspar.c	\
	cccspzld.c	\
	cccssacl.c	\
	cccsset.c	\
	cccssld.c	\
	cccssprt.c	\
	cccssset.c


if KLU_WANTED
libcccs_la_SOURCES += cccsbindCSC.c
endif

AM_CPPFLAGS = @AM_CPPFLAGS@ -I$(top_srcdir)/src/include
AM_CFLAGS = $(STATIC)

MAINTAINERCLEANFILES = Makefile.in
