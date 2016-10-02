^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roseus_smach
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2016-10-02)
------------------
* [roseus/src/state-machine-utils.l] add document string for exec-smach-with-spin
* [roseus_smach/src/state-machine-utils.l] support y-or-n-p when iterate mode
* Contributors: Yuki Furuta

1.5.3 (2016-05-28)
------------------

1.5.2 (2016-05-28)
------------------
* [roseus_smach/src/state-machine-utils.l] fix: return after goal reached on exec-smach-with-spin `#460 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/460>`_
* Remove no need euslisp from build_depend and find_package  `#456 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/456>`_
* Contributors: Kentaro Wada, Yuki Furuta

1.5.1 (2016-04-22)
------------------
* [roseus_smach/README.md] update to use github official image link
* Contributors: Furushchev

1.5.0 (2016-03-20)
------------------
* {roseus_smach, roseus_mongo}/README.md: fix section/subsection
* [roseus_smach/src/state-machine-utils.l] fix: smach connection from/to nil state
* Contributors: Kei Okada, Yuki Furuta

1.4.1 (2015-11-25)
------------------
* [roseus_smach/src/state-machine.l] another impl for `#383 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/383>`_
* Contributors: Yuki Furuta

1.4.0 (2015-11-03)
------------------
* [roseus_smach] warning message for :goal-state
* [roseus_smach] add failure nodes addition feature for state-machine generation utils
* [roseus_smach/sample/parallel-state-machine-sample.l] fix: https://github.com/jsk-ros-pkg/jsk_roseus/issues/324
* [roseus_smach] fix test launch file extension .launch -> .test; test only required
* Contributors: Kamada Hitoshi, Yuki Furuta

1.3.9 (2015-09-14)
------------------

1.3.8 (2015-09-12)
------------------

1.3.7 (2015-08-18)
------------------
* [README.md] describe how to run smach viewer
* [sample/state-machine-ros-sample.l, sample/state-machine-sample.l] add shbang
* [roseus_smach/CMakeLists.txt] disable test/test_parallel_state_machine_sample.launch for now
* [package.xml] add actionlib_tutorials depends to roseus_smach
* Contributors: Kei Okada

1.3.6 (2015-06-11)
------------------

1.3.5 (2015-05-15)
------------------

1.3.4 (2015-05-03)
------------------
* [roseus_smach] add test for parallel state machine
* [roseus_smach] add parallel state machine sample test to CMakeLists
* [roseus_smach] add sample test to CMakeLists
* [roseus_smach] split sample test in order to inspect failure detail
* [roseus_smach] change order of roseus in find_package
* [roseus_smach] move smach-exec function from sample to utils
* [roseus_smach] fix wrong file/module name
* [roseus_smach] miscellaneous fixes
* [roseus_smach] add test launch
* [roseus_smach] add feature async join
* [roseus_smach] fix transition fail when parallel state
* [roseus_smach] add async join state to  state-machine
* [roseus_smach] fix tmp -> next
* [roseus_smach] modify state-machine :execute-impl
* Contributors: Yuki Furuta, Kamada Hitoshi

1.3.3 (2015-04-29)
------------------

1.3.2 (2015-04-28)
------------------

1.3.1 (2015-04-26)
------------------
* [roseus_smach/src/state-machine-actionlib.l] support spin action client group, see `#274 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/274>`_
* [roseus_smach/src/state-machine-utils.l] modify make-state-machine
* [roseus_smach/src/state-machine-utils.l] add iterative execute state machine util
* Contributors: Yuki Furuta, Hitoshi Kamada, Kei Okada

1.3.0 (2015-04-24)
------------------
* [roseus_smach] add docstring for 'make-state-machine' function; add key option to custom exec-result to transit states
* [roseus_smach] fix sample parallel task transition; fix typo
* [roseus_smach] fix typo; change image link
* [roseus_smach] Create README.md add sample image
* [roseus_smach] add syntax suggar of creating state machine with parallel execution, and its sample code
* [roseus_smach] add feature: parallel executive state machine, and its visualization stuff
* [roseus_smach] use soft tab
* Contributors: Yuki Furuta

1.2.6 (2015-02-21)
------------------

1.2.5 (2015-02-13)
------------------

1.2.4 (2015-02-12)
------------------

1.2.3 (2015-02-02)
------------------

1.2.2 (2015-01-27)
------------------

1.2.1 (2015-01-27)
------------------

1.2.0 (2015-01-26)
------------------

1.1.33 (2015-01-26)
-------------------

1.1.32 (2015-01-26)
-------------------

1.1.31 (2015-01-23)
-------------------
* remove old manifest.xml, fully catkinize
* Contributors: Kei Okada

1.1.30 (2015-01-14)
-------------------

1.1.29 (2014-12-27)
-------------------

1.1.28 (2014-12-26)
-------------------

1.1.27 (2014-12-20)
-------------------
* remove cmake file for rosbuild
* not use executive_smach as deps directly; remove manifest.xml

1.1.26 (2014-11-10)
-------------------

1.1.25 (2014-10-10)
-------------------

1.1.23 (2014-09-24)
-------------------

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

1.1.16 (2014-05-11)
-------------------

1.1.15 (2014-05-10)
-------------------

1.1.14 (2014-05-09)
-------------------

1.1.13 (2014-05-06)
-------------------

1.1.12 (2014-05-06)
-------------------

1.1.11 (2014-05-04)
-------------------
* catkinize roseus_smach
* Contributors: Kei Okada

1.1.10 (2014-05-03 10:35)
-------------------------

1.1.9 (2014-05-03 09:30)
------------------------

1.1.8 (2014-05-02)
------------------

1.1.7 (2014-04-28 14:29)
------------------------

1.1.6 (2014-04-28 03:12)
------------------------

1.1.5 (2014-04-27)
------------------

1.1.4 (2014-04-25)
------------------

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
* roseus_smach: disable packages for groovy
* publish-all-status to state-machine-inspector
* use-sub-machine to pddl2smach.l
* modify :reset-state for setting typical state
* add keywords to pddl-graph-to-smach
* add smach utility functions
* fix default option
* add utility methods to state-machine-inspector
* add keyword for using userdata in pddl2smach
* add keyword for changing return value
* add :readable keyword for pddl2smach
* fix for working sample
* remove load command for irtgraph.l
* update internal data structure for new graph.l
* publish smach structure once, and latch it
* add test for roseus_smach samples, fixed the initial state setter method
* update roseus_smach for set initial state callback
* add actionlib_tutorials for sample scripts
* changed to use unreviewed version of irtgraph.l
* fix smach_structure publish properly timing, add user input action to task_compiler
* remove old method in roseus_smach
* move convert script from pddl to smach
* chenge test function to compare execution results
* commit for current scripts for demonstration
* use package:// for loading graph.l
* change test function for transition, eq -> equal
* set initial-state = send :start-state
* add initial-state-cb to roseus_smach
* add message name to constant in msg definition
* add ** to msg constant type
* add function to create state-instance which execute action-client
* commit current source tree
* add code for smach_viewer
* change name smach_roseus -> roseus_smach
* Contributors: Kei Okada, youhei, Manabu Saito, Xiangyu Chen
