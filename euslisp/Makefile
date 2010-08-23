include Makefile.eus
include $(shell rospack find mk)/cmake.mk

EUSC_PATCH=eus.c_CUSTUM_EUSDIR.patch

$(EUSC_PATCH): eus.c_DEFAULT_ENV.patch
	cat eus.c_DEFAULT_ENV.patch | sed s@DEFAULT_EUSDIR@$(EUSDIR)@ | sed s@DEFAULT_ARCHDIR@$(ARCHDIR)@ > $(EUSC_PATCH)
	if [ -e eus/lisp/c/eus.c ] ; then svn revert eus/lisp/c/eus.c; fi

all: $(EUSC_PATCH)
	## set default EUSDIR in eus.c
	-patch -r - --forward -p0 -d eus < $(EUSC_PATCH)
	## compile eus
	make -f Makefile.eus eus-installed
	## compile irteus
	make -f Makefile.eus irteus-installed
	## force hard link for rosrun
	cd $(EUSDIR)/$(ARCHDIR)/bin; ln -f eus irteus; ln -f eusgl irteusgl;
	# download manuals
	make -f Makefile.eus manuals
	touch rospack_nosubdirs




Makefile.eus:
	wget http://jskeus.svn.sourceforge.net/viewvc/jskeus/trunk/Makefile -O Makefile.eus

clean:
	-rm $(EUSC_PATCH) rospack_nosubdirs
	-rm eus-installed irteus-installed
	-rm -fr $(EUSDIR)/$(ARCHDIR)
	make -f Makefile.eus clean

wipe:
	-rm -fr build bin
	-make -f Makefile.eus wipe
	-rm -f Makefile.eus

