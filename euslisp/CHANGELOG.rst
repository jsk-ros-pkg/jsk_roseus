^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package euslisp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.19 (2014-06-10)
-------------------
* c274553 (Euslisp : models/*-robot.l, models/*-object.l) : Update sensor access ; do not overwrite :cameras method in each robot file ;; remove unused :cameras method from object files discussed in euslisp/jskeus/pull/92
* 3378b05 (Euslisp : load-irt-all-objects.l) Add dewalt-drill and unknown-side-table to conversion list at euslib's r62547 commit
* 5e77f0e (Euslisp : models/hitachi-fiesta-refrigerator-object.l, models/patra-robot.l, models/room73b2-hitachi-fiesta-refrigerator-object.l) Update rbrain converted models
* 1564d0a (jskeus : irtrobot.l, robot-model-usage.l, sample-robot-model.l) Add sensor accessosr and test codes discussed in euslisp/jskeus/pull/72 and jsk-ros-pkg/jsk_model_tools/issues/18
* 9996bf0 (Euslisp : primt.l) Henry Baker's contribution of 2013 July
* aca5c68 (jskeus : README.md) Update README.md
* Contributors: Shunichi Nozawa

1.1.18 (2014-05-16)
-------------------
* omit euslisp test codes which are arleady tested in jskeus/irteus/test
* fix test-irtrobot.test ;; we do not need to set demo-function for irteus-demo.l after https://github.com/euslisp/jskeus/pull/87
* Contributors: nozawa

1.1.17 (2014-05-11)
-------------------

1.1.16 (2014-05-11)
-------------------

1.1.15 (2014-05-10)
-------------------

1.1.14 (2014-05-09)
-------------------
* Fix long first name of k-okada with traditional japanese person style
* remove euslisp codes which are arleady migrated to irteus/test and include them in test launch
* Contributors: Yuto Inagaki, Shunnichi Nozawa

1.1.13 (2014-05-06)
-------------------

1.1.12 (2014-05-06)
-------------------

1.1.11 (2014-05-04)
-------------------

1.1.10 (2014-05-03)
-------------------

1.1.9 (2014-05-03)
------------------

1.1.8 (2014-05-02)
------------------
* add test codes using irteus motion codes copied from euslib/demo/ik/ik-test.l
* Contributors: nozawa

1.1.7 (2014-04-28)
------------------

1.1.6 (2014-04-28)
------------------

1.1.5 (2014-04-27)
------------------

1.1.4 (2014-04-25)
------------------
* add test code for using robot-model class
* Contributors: Shunnichi Nozawa

1.1.3 (2014-04-14)
------------------

1.1.2 (2014-04-07)
------------------
* catkin.make : fix: use gcc dumpmachine to check archtecture
* Contributors: Kei Okada
* test : add test code to test launch, test codes are already included in irteus/demo
* Contributors: Shunnichi Nozawa

1.1.1 (2014-04-07)
------------------
* use gcc dumpmachine to check archtecture
* Contributors: Kei Okada

1.1.0 (2014-04-07)
------------------
* Merge pull request `#49 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/49>`_ from cottsay/master
  Fix permissions on installed libraries
* (`#41 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/41>`_) check if installed binary inlcude old rpath with file(STRING,
* check gcc -dumpmachine for deb build
* Contributors: Kei Okada
* Fix permissions on installed libraries
  All shared-object libraries should have execute permissions.
* Contributors: Scott K Logan

1.0.4 (2014-03-31)
------------------
* try to download jskeus for 10 times
* do not set INCLUDE_DIRS to jskeus/eus/include, which cause error in roseus as "Project 'euslisp' specifies 'include' as an include dir, which is not found"
* Contributors: Kei Okada

1.0.3 (2014-03-29)
------------------
* euslisp: unittest.l, uses numnber of test, not number of assert
* euslisp: unittest.l, force error if signal or error
* euslisp: add build_depend to libpq-dev, see issue `#8 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/8>`_
* Contributors: Kei Okada

1.0.2 (2014-03-28)
------------------
* euslisp: add git to depends
* test/unittest.l: check test/results/failures numbers, return -1 if it fail to execute, force remove test results
* Contributors: Kei Okada

1.0.1 (2014-03-27)
------------------
* euslisp/roseus: add version numeber to 1.0.0
* Contributors: Kei Okada, Manabu saito, Masaki Murooka, Shunnichi Nozawa, Youhei Kakiuchi
