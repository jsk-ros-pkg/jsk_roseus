^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geneus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.18 (2014-05-16)
-------------------
* previous commit does not work with multiple CMAKE_PREFIX_PATH
* do not compile if users already installed with roseus-msgs package
* Contributors: Kei Okada

1.1.17 (2014-05-11)
-------------------

1.1.16 (2014-05-11)
-------------------

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
* check if rosbuild_init was called
* Contributors: Kei Okada

1.1.11 (2014-05-04)
-------------------

1.1.10 (2014-05-03)
-------------------

1.1.9 (2014-05-03)
------------------

1.1.8 (2014-05-02)
------------------

1.1.7 (2014-04-28)
------------------

1.1.6 (2014-04-28)
------------------

1.1.5 (2014-04-27)
------------------
* compile all dependent packages
* add target ALL to invoke compile
* Contributors: Kei Okada

1.1.4 (2014-04-25)
------------------
* use roseus_INSTALL_DIR variables so that we can put message file in different locate #68
* #63 seems introduce new bugs, reporeted on https://github.com/jsk-ros-pkg/jsk_visualization/pull/19
* Contributors: Kei Okada

1.1.3 (2014-04-14)
------------------
* fix for roseus message generation (`#51 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/51>`_)
 * set _ROSBUILD_GENERATED_MSG_FILES null before rosbuild_get_msgs
 * check eus2 has executable permission in msg/srv generation on rosbuild
 * fix depend tag of geneus manifest.xml
 * check SOURCE_DIR before set generate_messages_py
 * add depends to generate_messages_to_py
* Contributors: Kei Okada, Ryohei Ueda

1.1.0 (2014-04-07)
------------------
* roseus.cmake: add depend to message_generation_py, use same code for both msg/srv generation
* generated_eus: do not write generated file if manifest.l is not exists
* add geneus package that generate ros message for euslisp
* Contributors: Kei Okada

