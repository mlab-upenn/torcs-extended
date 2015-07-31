##############################################################################
#
#    file                 : Makefile
#    created              : Sat Nov 13 23:19:31 EST 2010
#    copyright            : (C) 2002 Utsav
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = autonet
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp serial.c timer.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml
SHIPSUBDIRS =

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-autonet_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-autonet_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
