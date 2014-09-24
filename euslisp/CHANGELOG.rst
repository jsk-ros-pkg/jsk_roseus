^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package euslisp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.23 (2014-09-24)
-------------------
* (euslisp) update issue tracker location.
* remove xfont-server, due to https://github.com/ros/rosdistro/issues/5163
* Contributors: Isaac IY Saito, Kei Okada, Ryohei Ueda

 * 2014-09-10 28f6367 (jskeus) (irtrobot, irtdyna, walk-motion) : Use :name instead of plist for footstep l/r
* 2014-09-09 06e14cb (jskeus) add documents for de617c2949ace302634611a3ccb3b1b4c7919534
* 2014-09-05 b62008f (EusLisp) change read-integer-array, please refer to https://github.com/euslisp/EusLisp/pull/51* 2014-09-09 be44a9f (jskeus) add documents for 2f94bc1d9723812bbe6adc1ca9afffb9e8daf68c
* 2014-09-05 b2bb3d6 (jskeus) add documents for 92f7cf1ea41941421e8f3cdf19a693e8ce4a8efe
* 2014-09-05 9fcd771 (jskeus) (irtmodel.tex) : Remove sample description for :solve-ik
* 2014-09-05 9037f92 (jskeus) (sample-arm-model, hanoi-arm) : Use :inverse-kinematics instead of :solve-ik (according to https://github.com/euslisp/jskeus/issues/125#issuecomment-54590070)
* 2014-09-05 7906365 (EusLisp) add documents for e245bcc12ff5a45dc6b736df1ddaf894d2398d69


1.1.22 (2014-09-04)
-------------------
* eus.c_DEFAULT_ENV.patch: remove debug print message
* Contributors: Kei Okada

* 2014-08-31 e23c4e5 (EusLisp.git) fix: escape double quote and less/greater
* 2014-08-31 d62e6ca (jskeus.git) fix order of author
* 2014-08-31 a63fd79 (EusLisp.git) escape double quate
* 2014-08-31 79cd5e6 (jskeus.git) update document strings
* 2014-08-31 17583a3 (jskeus.git) update readme for new documentation
* 2014-08-31 017ee01 (EusLisp.git) escape asterisk
* 2014-08-30 cecad8a (EusLisp.git) add tool to generate tex documentation
* 2014-08-30 c973174 (EusLisp.git) make latex sometimes silent for 10 min
* 2014-08-30 c26a5ba (jskeus.git) update documents
* 2014-08-30 b3946fa (EusLisp.git) fix optional
* 2014-08-30 6cf0908 (jskeus.git) use travis to generate pdf and htmls, add texlive-latex-base ptex-bin latex2html nkf ebb to install
* 2014-08-30 5c7a3e1 (jskeus.git) add jmanual.pdf jmanual.dvi
* 2014-08-30 03ccb7e (jskeus.git) add documentation tools
* 2014-08-29 cbae8b6 (EusLisp.git) change argument name of sys:poke, fix #46
* 2014-08-29 b5238a6 (EusLisp.git) write loaded modules to error-output
* 2014-08-29 69aedd9 (EusLisp.git) use utf8 for default language
* 2014-08-29 68942ee (EusLisp.git) enable set-dispatch-macro-character for integer
* 2014-08-29 5ded1ab (jskeus.git) add deftest for reader
* 2014-08-29 2d1477c (EusLisp.git) commit pdf/dvi
* 2014-08-29 2d0da02 (EusLisp.git) fix manuals for :newcoords, fixes #44
* 2014-08-29 2c6e11a (EusLisp.git) change document code to euc for latex
* 2014-08-28 ffa169b (EusLisp.git) publish documents to gh-pages
* 2014-08-28 f0d39d4 (EusLisp.git) add tex, nkf, latex
* 2014-08-28 d97642f (EusLisp.git) Update README.md
* 2014-08-28 9096c61 (jskeus.git) (sample-*-model) : Define sample-arm-robot and sample-hand-robot as robot-model subclass
* 2014-08-28 471c750 (EusLisp.git) fix: publish documents in gh-pages
* 2014-08-28 12975e5 (jskeus.git) (sample-multidof-arm-model) : Enable to set joint class for sample-multidof-arm-robot
* 2014-08-27 87dd484 (EusLisp.git) use UTF-8 for documents
* 2014-08-23 c0e1f29 (EusLisp.git) (.travis.yml) : Add apt-get update because error occurs in recent travis test reported in https://github.com/euslisp/EusLisp/pull/40
* 2014-08-21 6c948ca (EusLisp.git) (pgsql.l) : Fix libpq.so path for x86_64 and i386 environment discussed in https://github.com/euslisp/jskeus/issues/111* 2014-08-31 f98b265 (jskeus.git) add documents for 193a19527c7b9da228488f4831102de16fced155
* 2014-08-18 be55374 (jskeus.git) (null-space-ik) : Add additional-check for null-space example to wait for being enough distance
* 2014-08-18 a3815f2 (jskeus.git) (irtmodel.l) : Add comments for addtional-check argument
* 2014-07-23 2a79e43 (jskeus.git) (test-irt-motion.l) : Add test codes for calc-torque with external force and moment
* 2014-07-16 daae0e2 (jskeus.git) (irtmodel.l, test-irt-motion.l) : Fix link access in ik fail log and add test code for it. This bug is reported in https://github.com/jsk-ros-pkg/jsk_roseus/issues/139
* 2014-07-16 d740c10 (jskeus.git) (irtmodel.l, test-irt-motion.l) : Update dump ik fail log to escape all links and list to fix https://github.com/jsk-ros-pkg/jsk_roseus/issues/138 and fix bug in move-target or target-coords
* 2014-07-16 baf77d6 (jskeus.git) (irtmodel.l, test-irt-motion.l) : Fix link dump for move-target and search link included in :links and add test code
* 2014-07-16 b5811c1 (jskeus.git) (test-irt-motion.l) : Set *sample-robot* instead of *robot* to avoid conflict
* 2014-07-16 7fe0769 (jskeus.git) (test-irt-motion) : Clear ik fail log file for one test
* 2014-07-16 21ef7c9 (jskeus.git) (irtmodel.l, test-irt-motion.l) : Escape string link name and add test for string-name case
* 2014-07-16 14fff7f (jskeus.git) (.travis.ymml) : Add apt-get update to fix apt-get error reported in https://github.com/euslisp/jskeus/pull/101

1.1.21 (2014-06-30)
-------------------

1.1.20 (2014-06-29)
-------------------
* 2da6078 (lisp/geo/primp.l, lisp/l/common.l) revert Henry Baker's contribution of 2013 July, this breaks test code  https://github.com/euslisp/jskeus/pull/100
* c9a76d5 (-objects.l) : Assoc handles and attentions with adequate parent link ;; This bug is reported in https://github.com/euslisp/EusLisp/pull/31
* b21eda6 (.travis.yaml) : Update travis test to use irteus-demo.l and add test for eus/models
* 7755cb0 (models/drcbox-*.l) :refrain drcbox model. fix positions of objects in drcbox and color, and add casters
* 7ee3263 (drcbox-valve*.l) Import handle coordinates from rbrain models
* 5a89f25 (irt-all-scene.l, load-irt-all-scene.l) Add test codes for all irt scene models like irt-all-robots and irt-all-objects
* 0397569 (drcbox*.l) Add new models and  scene for drcbox ;; This originally derived from  https://github.com/euslisp/EusLisp/pull/27
* c9d6c82 (models/darwin.l) revert codes for collision model making according to https://github.com/euslisp/jskeus/pull/93 and https://github.com/jsk-ros-pkg/jsk_model_tools/pull/46
* 23e85ee (irteus/test/geo.l) owverwrite face-normal-vector, see https://github.com/euslisp/EusLisp/pull/21
* 454bde8 (irteus/test/geo.l): add test code for geometry functions (https://github.com/euslisp/EusLisp/pull/21)
* be1ecc0 (irtdyna.l, test-irt-motion.l) Fix bug of :cog-convergence-check and add test codes
* 99486d7 (irteus/test/joint.l) Execute test even if  display is not found
* 9e5ff99 (irteus/test/joint.l) Add min-max violation test ;; Update joint.l to replace magic number by min-angle or max-angle
* 413c575 (irteus/test/all-robots-objects.l) Add unittest for scene models corresponding to  https://github.com/euslisp/EusLisp/pull/29
* 425c9d1 (irteus/irtrobot.l) revert codes for collision model making according to https://github.com/euslisp/jskeus/pull/93 and https://github.com/jsk-ros-pkg/jsk_model_tools/pull/46
* Contributors: Shunichi Nozawa, Kei Okada, Eisoku Kuroiwa


1.1.19 (2014-06-11)
-------------------
* c274553 (Euslisp : models/*-robot.l, models/*-object.l) : Update  sensor access ; do not overwrite :cameras method in each robot file,  remove unused :cameras method from object files discussed in euslisp/jskeus/pull/92
* 3378b05 (Euslisp : load-irt-all-objects.l) Add dewalt-drill and unknown-side-table to conversion list at euslib's r62547 commit
* 5e77f0e (Euslisp : models/hitachi-fiesta-refrigerator-object.l, models/patra-robot.l, models/room73b2-hitachi-fiesta-refrigerator-object.l) Update rbrain converted models
* 1564d0a (jskeus : irtrobot.l, robot-model-usage.l, sample-robot-model.l) Add sensor accessosr and test codes discussed in euslisp/jskeus/pull/72 and jsk-ros-pkg/jsk_model_tools/issues/18
* 9996bf0 (Euslisp : primt.l) Henry Baker's contribution of 2013 July
* aca5c68 (jskeus : README.md) Update README.md
* Contributors: Henry Baker, Shunichi Nozawa, Kei Okada

1.1.18 (2014-05-16)
-------------------
* omit euslisp test codes which are arleady tested in jskeus/irteus/test
* fix test-irtrobot.test ;; we do not need to set demo-function for irteus-demo.l after https://github.com/euslisp/jskeus/pull/87
* Contributors: Shunichi Nozawa

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
* Contributors: Yuto Inagaki, Shunichi Nozawa

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
