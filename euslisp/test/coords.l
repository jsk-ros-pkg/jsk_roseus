(require :unittest "lib/llib/unittest.l")

;;(setq sys::*gc-hook* #'(lambda (a b) (format t "GC! free:~A  total:~A~%" a b)))

(init-unit-test)

(setq i-max 1000000)

(deftest test-makecoords-rpy
  (dotimes (j 20)
    (dotimes (i i-max)
      (setq ret #f(0 0 0))
      (make-cascoords :pos (float-vector 0 0 0)
                      :rpy (float-vector 0 0 0)))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "make-coords pos/rpy")))

(deftest test-makecoords-rot-1
  (dotimes (j 20)
    (dotimes (i i-max)
      (setq ret #f(0 0 0))
      (make-cascoords :pos (float-vector 0 0 0)
                      :rot (unit-matrix 3)))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "make-coords pos/rot (unit-matrix)")))

(deftest test-makecoords-rot-2
  (dotimes (j 20)
    (dotimes (i i-max)
      (setq ret #f(0 0 0))
      (make-cascoords :pos (float-vector 0 0 0)
                      :rot (rpy-matrix 0 0 0)))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "make-coords pos/rot (rpy-matrix)")))

(deftest test-makecoords-quaternion
  (dotimes (j 20)
    (dotimes (i i-max)
      (setq ret #f(0 0 0 1 0 0 0))
      (make-cascoords :pos (float-vector 0 0 0)
                      :rot
                      (user::quaternion2matrix (float-vector 1 0 0 0))))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "make-coords quaternion2matrix")))

(run-all-tests)
(exit)
