;;-*- mode: lisp -*-

(defun split-string (str)
  (let ((chars (coerce str cons)))
    (labels ((rec (rest-chars processing result)
               (cond ((null rest-chars)
                      (append result (list processing)))
                     ((= (car rest-chars) #\ )
                      (rec (cdr rest-chars)
                           nil
                           (append result (list processing))))
                     (t
                      (rec (cdr rest-chars)
                           (append processing (list (car rest-chars)))
                           result)))))
      (mapcar #'(lambda (x)
                  (coerce x string))
              (rec chars nil nil)))))

(make-package "ROS")
(defun genmanifest (target-file depend-packages)
  (print (list target-file depend-packages))
  (with-open-file (f target-file :direction :output)
    (pprint '(defun load-ros-package (pkg)
	       (unless (find-package (string-upcase pkg))
		 (make-package (string-upcase pkg)))
	       (when (probe-file (format nil "~A/msg/eus"
					 (ros::rospack-find pkg)))
		 (roseus-add-msgs pkg))
	       (when (probe-file (format nil "~A/msg/srv"
					 (ros::rospack-find pkg)))
		 (roseus-add-srvs pkg))
	       (when (probe-file (format nil "~A/src/euslisp/init.l"
					 (ros::rospack-find pkg)))
		 (load (format nil "~A/src/euslisp/init.l"
			       (ros::rospack-find pkg))))
	       )
	    f)
    (dolist (pkg (split-string depend-packages))
      (pprint `(load-ros-package ,pkg) f))
    ))
