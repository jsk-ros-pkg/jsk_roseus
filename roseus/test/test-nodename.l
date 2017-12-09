#!/usr/bin/env roseus
;;

(require :unittest "lib/llib/unittest.l")
;; remove __name:= to use (ros::roseus node-name) to set node name, otherwise node name is defined form cargs/cargs in ROSEUS() 
(print lisp::*eustop-argument*)
(setq node-name (subseq (car (member "__name:=" lisp::*eustop-argument* :test #'substringp)) (length "__name:=")))
(setq node-name (substitute #\_ #\- node-name)) ;; accept '-' in node name
(setf (elt lisp::*eustop-argument* (position "__name:=" lisp::*eustop-argument* :test #'substringp))
      (format nil "noname:=~A" node-name))

;;
(ros::roseus node-name :anonymous nil)


;; start test
(init-unit-test)

(deftest test-master ()

  (ros::ros-info "get-name ~A" (ros::get-name))

  (assert (string= (ros::get-name) (format nil "/~A" node-name))
          (format nil "~A == ~A" (ros::get-name) node-name))
  )

(run-all-tests)

(exit)
