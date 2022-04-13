(require :unittest "lib/llib/unittest.l")

(load "package://roseus_smach/sample/state-machine-ros-sample.l")
(load "package://roseus_smach/sample/parallel-state-machine-sample.l")

(ros::roseus "test_roseus_smach_samples")


(setq *container-active-status* nil)
(ros::subscribe "/server_name/smach/container_status" smach_msgs::SmachContainerStatus
                #'(lambda (msg) (push (send msg :active_states) *container-active-status*)))

(defmacro run-test-smach (func outcome active-states)
  (let ((test-func-name (read-from-string (format nil "test-~A" func))))
    `(deftest ,test-func-name
       (let (start-tm result)
         (setq *container-active-status* nil)
         (setq start-tm (ros::time-now))
         (setq result (funcall ',func))
         (format *error-output* "executed ~A and returns ~A~%" ',func result)
         (assert (eq (send result :name) ,outcome) "expected result is ~A, but returns ~A" ,outcome result)
         (setq elapsed-tm (ros::time- (ros::time-now) start-tm))
         (format *error-output* "received container-active-status is ~A, and takes ~A sec~%" *container-active-status* (send elapsed-tm :to-sec))
         (assert (= (length (remove nil *container-active-status*)) (length (remove nil ,active-states))) (format nil "length of container active status is ~A, but ~A~%" (length (remove nil ,active-states)) (length (remove nil *container-active-status*))))
         (assert (eps= (send elapsed-tm :to-sec) (float (length ,active-states)) 5.0) (format nil "duration of container active status is equal to length of container active status (~A), but ~A~%" (length ,active-states) (send elapsed-tm :to-sec)))
         ))
    ))

(init-unit-test)

(defun exec-sample-parallel-state-machine ()
  (make-sample-parallel-state-machine)
  (exec-state-machine *sm*))

(defun exec-smach-simple2 () (setq count 0) (exec-state-machine (smach-simple2)))
(defun exec-smach-simple3 () (setq count 0) (exec-state-machine (smach-simple3)))
(defun exec-smach-simple-nested () (setq count 0) (exec-state-machine (smach-simple-nested)))
(run-test-smach exec-smach-simple :outcome4 '((FOO) (BAR) (FOO) (BAR) (FOO) (BAR) (FOO)))
(run-test-smach exec-smach-simple2 :outcome4 '((FOO) (BAR) (FOO) (BAR) (FOO) (BAR) (FOO)))
(run-test-smach exec-smach-simple3 :outcome4 '((FOO) (BAR) (FOO) (BAR) (FOO) (BAR) (FOO)))

(run-test-smach exec-smach-nested :outcome5 '(nil (FOO) (BAR) (FOO) (BAR) (FOO) (BAR) nil (BAS) nil))
(run-test-smach exec-smach-simple-nested :outcome5 '(nil (FOO) (BAR) (FOO) (BAR) (FOO) (BAR) nil (BAS) nil))

(run-test-smach exec-smach-userdata :outcome4 '((FOO) (BAR) (FOO) (BAR) (FOO)))

(deftest test-smach-sample-userdata ()
;  (assert (eq (send (exec-smach-userdata) :name) :outcome4)
;	  "sample of smach with userdata")
  (assert (eq (send (exec-state-machine (smach-userdata)) :name) :outcome4)
          "exec (smach-userdata) without initial userdata"))

(run-test-smach exec-sample-parallel-state-machine :success '((PRESS-BUTTON) (CLOSE-DOOR) (PUT-SOAP PUT-CLOTH) (OPEN-DOOR)))

(deftest test-smach-action-client-state ()
  (setq userdata '(nil))
  (assert (eq (send (exec-state-machine (smach-action-client-state) userdata) :name) :SUCCEED-STATE)
          "exec (smach-action-server) is succeeded")
  (assert (cdr (assoc :result userdata))
          "action-client-state sets action result to userdata for key :result"))

(run-all-tests)

(exit)
