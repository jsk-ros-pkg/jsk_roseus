include Makefile.eus
include $(shell rospack find mk)/cmake.mk

EUSC_PATCH=eus.c_CUSTUM_EUSDIR.patch

all: Makefile.eus compile

Makefile.eus:
	wget http://jskeus.svn.sourceforge.net/viewvc/jskeus/trunk/Makefile -O Makefile.eus

compile:
	make -f Makefile.eus eus eus-update
	svn revert eus/lisp/c/eus.c
	## set default EUSDIR in eus.c
	cat eus.c_DEFAULT_ENV.patch | sed s@DEFAULT_EUSDIR@$(EUSDIR)@ | sed s@DEFAULT_ARCHDIR@$(ARCHDIR)@ > $(EUSC_PATCH)
	patch -p0 -d eus < $(EUSC_PATCH)
	make -f Makefile.eus eus-installed
	make -f Makefile.eus irteus irteus-update
	## force hard link for rosrun
	cd $(EUSDIR)/$(ARCHDIR)/bin; ln -f eus irteus; ln -f eusgl irteusgl;
	make -f Makefile.eus irteus-installed
	touch rospack_nosubdirs

clean:
	-rm $(EUSC_PATCH) rospack_nosubdirs
	make -f Makefile.eus clean

wipe:
	-rm -fr build bin
	-make -f Makefile.eus wipe
	-rm -f Makefile.eus

