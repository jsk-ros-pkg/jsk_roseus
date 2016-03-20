^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roseus_mongo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.0 (2016-03-20)
------------------
* {roseus_smach, roseus_mongo}/README.md: fix section/subsection
* Contributors: Kei Okada

1.4.1 (2015-11-25)
------------------
* [roseus_mongo/test/test_mongo_client.test] add missing machine tag for localhost
* [roseus_mongo] test with new mongodb_store; more loose condition to enable limit option
* [roseus_mongo/test/temp_mongodb_store.xml] update launch file path with fix https://github.com/strands-project/mongodb_store/pull/151
  [roseus_mongo/test/test_mongo_client_hydro.test] moved deprecated test launch file for hydro
  [roseus_mongo/CMakeLists.txt] updated to test only CATKIN_ENABLE_TESTING; check mongodb_store version and switch test launch file
* [roseus_mongo/test/test-mongo-client.l] limit test for indigo later only
  [roseus_mongo/euslisp/mongo-client.l] limit ':limit' option only for indigo and later
* Contributors: Kei Okada, Yuki Furuta

1.4.0 (2015-11-03)
------------------
* [roseus_mongo/README.md] update readme for setting params
* [roseus_mongo/euslisp/mongo-client.l] fix param name for *mongo-database*
* [roseus_mongo] support timezone
* Contributors: Yuki Furuta

1.3.9 (2015-09-14)
------------------

1.3.8 (2015-09-12)
------------------
* [roseus_mongo] fix replicate function (send-goal did not send-goal
* [roseus_mongo/euslisp/mongo-client-sample.l] fix: update sample
* Contributors: Yuki Furuta

1.3.7 (2015-08-18)
------------------
* [roseus_mongo/euslisp/json/json-decode.l] use keyword for key as default
* [roseus_mongo] support immediate replicatation function
* [roseus_mongo] avoid reserved function name delete -> delete-by-id
* [roseus_mongo] use "mongodb/database" and "robot/name" as default db/collection name
* [roseus_mongo] bugfix: support convert  to calendar-date object
* [roseus_mongo] add mongodb client for roseus
* Contributors: Yuki Furuta

1.3.6 (2015-06-11)
------------------

1.3.5 (2015-05-15)
------------------

1.3.4 (2015-05-03)
------------------

1.3.3 (2015-04-29)
------------------

1.3.2 (2015-04-28)
------------------

1.3.1 (2015-04-26)
------------------

1.3.0 (2015-04-24)
------------------

1.2.7 (2015-02-22)
------------------

1.2.6 (2015-02-21)
------------------

1.2.5 (2015-02-13)
------------------

1.2.4 (2015-02-12)
------------------

1.2.3 (2015-02-02)
------------------

1.2.2 (2015-01-27 18:38)
------------------------

1.2.1 (2015-01-27 00:34)
------------------------

1.2.0 (2015-01-26 23:20)
------------------------

1.1.33 (2015-01-26 14:56)
-------------------------

1.1.32 (2015-01-26 02:27)
-------------------------

1.1.31 (2015-01-23)
-------------------

1.1.30 (2015-01-14)
-------------------

1.1.29 (2014-12-27)
-------------------

1.1.28 (2014-12-26)
-------------------

1.1.27 (2014-12-20)
-------------------

1.1.26 (2014-11-10)
-------------------

1.1.25 (2014-10-10)
-------------------

1.1.24 (2014-09-24 11:56:16)
----------------------------

1.1.23 (2014-09-24 11:56:02)
----------------------------

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

1.1.17 (2014-05-11 13:27)
-------------------------

1.1.16 (2014-05-11 03:23)
-------------------------

1.1.15 (2014-05-10)
-------------------

1.1.14 (2014-05-09)
-------------------

1.1.13 (2014-05-06 15:36)
-------------------------

1.1.12 (2014-05-06 03:54)
-------------------------

1.1.11 (2014-05-04)
-------------------

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
