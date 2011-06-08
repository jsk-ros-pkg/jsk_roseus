all-target: ros-euslisp-all
include $(shell rospack find mk)/cmake.mk

EUSC_PATCH=eus.c_CUSTUM_EUSDIR.patch

include jskeus/Makefile
export EUSDIR=$(shell pwd)/jskeus/eus

ros-euslisp-all: wiped $(EUSC_PATCH)
	## set default environment variables to  eus.c
	## EUSDIR=$(EUSDIR)
	## ARCHDIR=$(ARCHDIR)
	-patch -r - --forward -p0 -d jskeus/eus < $(EUSC_PATCH)
	## compile jskeus
	(cd jskeus; make)
	## force hard link for rosrun
	cd $(EUSDIR)/$(ARCHDIR)/bin; ln -f eus irteus; ln -f eusgl irteusgl;
	touch installed

$(EUSC_PATCH): eus.c_DEFAULT_ENV.patch
	cat eus.c_DEFAULT_ENV.patch | sed s@DEFAULT_EUSDIR@$(EUSDIR)@ | sed s@DEFAULT_ARCHDIR@$(ARCHDIR)@ > $(EUSC_PATCH)
	if [ -e jskeus/eus/lisp/c/eus.c ] ; then svn revert jskeus/eus/lisp/c/eus.c; fi


clean:
	-rm $(EUSC_PATCH) rospack_nosubdirs
	-(cd jskeus;make clean)
	-rm -fr $(EUSDIR)/$(ARCHDIR)

wipe: clean
	-rm -fr build bin installed wiped

wiped: Makefile
	make wipe
	touch wiped
