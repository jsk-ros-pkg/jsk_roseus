#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")

(load "package://eusgazebo/euslisp/samples/fall-arrow-object-simulation.l")

(init-unit-test)

(deftest test-fall-arrow-object-simulation
  (fall-arrow-object-simulation))

(run-all-tests)
(exit)
