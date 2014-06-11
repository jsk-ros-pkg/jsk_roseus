^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roseus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.19 (2014-06-11)
-------------------
* (#112,#113) fix service persist without keyward
  ros::service-call (name value &optional (persist nil))
* Contributors: Kei Okada, Ryohei Ueda

1.1.18 (2014-05-16)
-------------------

1.1.17 (2014-05-11)
-------------------

1.1.16 (2014-05-11)
-------------------

1.1.15 (2014-05-10)
-------------------

1.1.14 (2014-05-09)
-------------------
* add hasHeader for roscpp >= 1.11.1
* fix typo in install roseus
* Contributors: Kei Okada

1.1.13 (2014-05-06)
-------------------
* add more message when install roseus
* Contributors: Kei Okada

1.1.12 (2014-05-06)
-------------------

1.1.11 (2014-05-04)
-------------------

1.1.10 (2014-05-03)
-------------------

1.1.9 (2014-05-03)
------------------
* add debug message when install roseus
* Contributors: Kei Okada

1.1.8 (2014-05-02)
------------------
* create symlink in global/bin/roseus
* Contributors: Kei Okada

1.1.7 (2014-04-28)
------------------

1.1.6 (2014-04-28)
------------------

1.1.5 (2014-04-27)
------------------

1.1.4 (2014-04-25)
------------------
* check msg file udder CMAKE_PREFIX_PATH (#68)
* (#31) use 120 as wait-for-transform
* Contributors: Kei Okada

1.1.3 (2014-04-14)
------------------
* add rosdnoe to depends(#64)
* Contributors: Kei Okada

1.1.0 (2014-04-07)
------------------
* add geneus package that generate ros message for euslisp
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) copy jsk_roseus for one workspace and remove build on rosbuild
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) add rich test for euslisp message generation, remove scripts and generate them from one shell script.
  * one workspace/separated workspace
  * add several dependency
  * action messages generation
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) add scripts to test geneus more
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) check if test the message has created or not by simple roseus program, add euslisp test rather than cpp test code
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) add test-genmsg, test message generation on catkin and rosbuild
* add check delay of lookuptransform
* add checking delay of tf return
* Contributors: Kei Okada, Ryohei Ueda, YoheiKakiuchi

1.0.4 (2014-03-31)
------------------
* fix for catkin environment
* set euslisp_PACKAGE_PATH for both devel and installed
* switch from svnversion to git rev-parse --short HEAD
* removed debug messages
* Contributors: Kei Okada, Ryohei Ueda

1.0.3 (2014-03-29)
------------------
* catkin.cmake add rostest to find_package
* `#14 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/14>`_: depend roseus message generation on python message generation.
  in roseus.cmake, do not take into account the dependencies between messages
  and packages and just depends roseus message generation on python message generation.
  The 1st reason is the difference between hydro and groovy. On groovy, genmsg
  does not craete the targets of foo_generate_messages_py
  which are already compiled, I mean the packages installed by apt.
  The 2nd reason is that roseus message generation utilizes rospy and it requires
  for rospy messages to be available. So this dependencies are required.
  Namely, the dependency will be like this:
  parent_pkg
  +-child_pkg
    +-grandchild_pkg
      +-grandchild_pkg_generate_messages_py
        +-euslip targets for grandchild_pkg
* Contributors: Ryohei Ueda
* roseus/test/test-tf.test: tf2_buffer_server output to screen

1.0.2 (2014-03-28)
------------------
* roseus.cmake: remove debug code
* roseus/test/test-add-two-ints.l: reduce test time
* Contributors: Kei Okada

1.0.1 (2014-03-27)
------------------
* roseus: add version numeber to 1.0.0
* Contributors: Kei Okada, Ryohei Ueda, Yohei Kakiuchi, Haseru Chen, Yuki Furuta, Yuto Inagaki, kazuto Murase, Eisoku Kuroiwa, Manabu Saito, Hiroyuki Mikita, Shunnich Nozawa
