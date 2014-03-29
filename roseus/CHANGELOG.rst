^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roseus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
