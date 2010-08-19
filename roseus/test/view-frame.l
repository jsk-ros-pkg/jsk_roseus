#!/usr/bin/env roseus

(if (not (boundp '*pickview*)) (pickview))
(defun main ()
  (let (tl frame-strings frame-coords root-frame objs)
    (roseus "eus_view")
    (ros::rate 10)
    (setq tl (instance ros::transform-listener :init))
    (ros::sleep)
    (setq frame-strings (send tl :get-frame-strings))
    ;; find root
    (setq root-frame (find nil frame-strings :key #'(lambda (x) (send tl :get-parent x (ros::time)))))
    (dolist (frame frame-strings)
      (let ((coords (send tl :lookup-transform root-frame frame (ros::time))))
        (push coords frame-coords)))
    (dolist (coords frame-coords)
      (let* ((parent-name (send tl :get-parent (send coords :name) (ros::time)))
             (parent-coords (find parent-name frame-coords
                                  :test #'string=
                                  :key #'(lambda (x) (send x :name)))))
        ;;(when parent-name (send parent-coords :assoc coords))
        ))
    (setq objs
          (mapcar #'(lambda (c)
                      (send (make-arrow 50) :newcoords c)) frame-coords))
    (objects objs)
    frame-coords))
;;
(setq frame-coords (main))



