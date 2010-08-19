all-new: Makefile.eus compile

include Makefile.eus
include $(shell rospack find mk)/cmake.mk

EUSC_PATCH=eus.c_CUSTUM_EUSDIR.patch
ROS_PARALLEL_JOBS=-j1

compile:
	make -f Makefile.eus eus eus-update
	svn revert eus/lisp/c/eus.c
	## set default EUSDIR in eus.c
	cat eus.c_DEFAULT_ENV.patch | sed s@DEFAULT_EUSDIR@$(EUSDIR)@ | sed s@DEFAULT_ARCHDIR@$(ARCHDIR)@ > $(EUSC_PATCH)
	patch -p0 -d eus < $(EUSC_PATCH)
	# compile
	touch eus-update
	make -f Makefile.eus eus-installed
	rm eus-update
	## irteus
	make -f Makefile.eus irteus-installed
	## force hard link for rosrun
	cd $(EUSDIR)/$(ARCHDIR)/bin; ln -f eus irteus; ln -f eusgl irteusgl;
	touch rospack_nosubdirs




Makefile.eus:
	wget http://jskeus.svn.sourceforge.net/viewvc/jskeus/trunk/Makefile -O Makefile.eus

clean:
	-rm $(EUSC_PATCH) rospack_nosubdirs
	-rm eus-installed irteus-installed
	make -f Makefile.eus clean

wipe:
	-rm -fr build bin
	-make -f Makefile.eus wipe
	-rm -f Makefile.eus

