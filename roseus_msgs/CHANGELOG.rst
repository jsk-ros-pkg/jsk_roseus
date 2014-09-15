^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roseus_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.22 (2014-09-04)
-------------------

1.1.21 (2014-06-30)
-------------------

1.1.20 (2014-06-29)
-------------------

1.1.19 (2014-06-11)
-------------------

1.1.18 (2014-05-16)
-------------------

1.1.17 (2014-05-11)
-------------------
* Merge pull request #104 from k-okada/add_more_packages
  add more package to generate messages
* add more package to generate messages

1.1.16 (2014-05-11)
-------------------
* use find_package() to set environment variables

1.1.15 (2014-05-10)
-------------------
* compile message even if not catkinized
* Contributors: Kei Okada

1.1.14 (2014-05-09)
-------------------
* geneus: add rospack_depends to find dependencies
* Contributors: Kei Okada

1.1.13 (2014-05-06)
-------------------

1.1.12 (2014-05-06)
-------------------
* add mode packages to depends
* use sudo for rosdep init
* Contributors: Kei Okada

1.1.11 (2014-05-04)
-------------------

1.1.10 (2014-05-03)
-------------------
* roseus_msgs depends on roseus, since pr2eus depends on roseus_msgs
* Contributors: Kei Okada

1.1.9 (2014-05-03)
------------------

1.1.8 (2014-05-02)
------------------
* run rosdep init and rosdep update before create messages
* add more package to depends
* Contributors: Kei Okada

1.1.7 (2014-04-28)
------------------
* use catkin to install genrated messages
* Contributors: Kei Okada

1.1.6 (2014-04-28)
------------------
* build process does not know CATKIN_GLOBAL_SHARE_DESTINATION
* Contributors: Kei Okada

1.1.5 (2014-04-27)
------------------
* comment out build_depend, to speed up travis, these build_depends only needs at ros build firm
* if CATKIN_DEVEL_PREFIX is not set, use CMAKE_INSTALL_PREFIX
* compile all installed package useing CMAKE_PREFIX_PATH
* Contributors: Kei Okada

1.1.4 (2014-04-25)
------------------
* add roseus_msgs that generates files under share/roseus/ros (#68)
* Contributors: Kei Okada

1.1.3 (2014-04-14)
------------------

1.1.2 (2014-04-07 23:17)
------------------------

1.1.1 (2014-04-07 09:02)
------------------------

1.1.0 (2014-04-07 00:52)
------------------------

1.0.4 (2014-03-31)
------------------

1.0.3 (2014-03-30)
------------------

1.0.2 (2014-03-28)
------------------

1.0.1 (2014-03-27)
------------------
