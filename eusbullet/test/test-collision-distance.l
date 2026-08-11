#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "../euslisp/sample/sample-collision-distance.l")


(init-unit-test)

(deftest test-sample-collision-distance-2d-analytical
  (sample-collision-distance-2d-analytical :max-cnt 500 :visualize? nil)
  )


(run-all-tests)
(exit)
