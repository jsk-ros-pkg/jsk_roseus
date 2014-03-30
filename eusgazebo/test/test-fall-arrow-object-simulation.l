#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")

(load "package://eusgazebo/samples/fall-arrow-object-simulation.l")

(init-unit-test)

(deftest test-fall-arrow-object-simulation
  (fall-arrow-object-simulation)
  (unix:system "pkill gzserver"))

(run-all-tests)
(exit)
