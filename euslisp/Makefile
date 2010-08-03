include $(shell rospack find mk)/cmake.mk

EUSC_PATCH=eus.c_CUSTUM_EUSDIR.patch

all: Makefile.eus compile

Makefile.eus:
	wget http://jskeus.svn.sourceforge.net/viewvc/jskeus/trunk/Makefile -O Makefile.eus

export EUSDIR=$(shell pwd)/eus

compile:
	make -f Makefile.eus eus eus-update
	svn revert eus/lisp/c/eus.c
	## set default EUSDIR in eus.c
	cat eus.c_DEFAULT_ENV.patch | sed s@DEFAULT_EUSDIR@$(EUSDIR)@ | sed s@DEFAULT_ARCHDIR@$(ARCHDIR)@ > $(EUSC_PATCH)
	patch -p0 -d eus < $(EUSC_PATCH)
	make -f Makefile.eus eus eus-installed irteus-installed

clean:
	-rm $(EUS_PATCH)
	make -f Makefile.eus clean

wipe:
	make -f Makefile.eus wipe

