#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")

(load "package://eusgazebo/samples/play-domino-simulation.l")

(init-unit-test)

(deftest test-play-domino-simulation
  (init-domino-simulation)
  (play-domino-simulation)
  (unix:system "pkill gzserver"))

(run-all-tests)
(exit)
