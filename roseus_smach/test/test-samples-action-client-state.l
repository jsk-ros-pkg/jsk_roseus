(require :unittest "lib/llib/unittest.l")

;; removed from test-samples.l since it is always failling on only installed test https://api.travis-ci.org/v3/job/406576370/log.txt
;; not sure why...
;;   start testing [test-smach-action-client-state]
;;   m;p=pointer?(0x6252338)
;;   ;; Segmentation Fault.
;;   terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
;;   what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
;;
(load "package://roseus_smach/sample/state-machine-ros-sample.l")

(ros::roseus "test_roseus_smach_samples_action_client_state")

(init-unit-test)

(deftest test-smach-action-client-state ()
  (setq userdata '(nil))
  (assert (eq (send (exec-state-machine (smach-action-client-state) userdata) :name) :SUCCEED-STATE)
          "exec (smach-action-server) is succeeded")
  (assert (cdr (assoc :result userdata))
          "action-client-state sets action result to userdata for key :result"))


(run-all-tests)

(exit)
