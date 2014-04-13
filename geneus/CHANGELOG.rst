^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geneus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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

