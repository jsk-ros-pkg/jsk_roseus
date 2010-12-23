#!/usr/bin/env roseus

(load "models/arrow-object.l")

(ros::roseus "eus_view_frame")
(setq *tl* (instance ros::transform-listener :init))

(defun tf->cascoords ()
  (let (frame-strings frame-coords root-frame objs)
    (ros::rate 10)
    (ros::ros-info "wait for tf")
    (while (null frame-strings)
      (ros::sleep)
      (setq frame-strings (send *tl* :get-frame-strings)))
    (ros::ros-info "frames ~A~%" frame-strings)
    ;; find root
    (setq root-frame (find nil frame-strings :key #'(lambda (x) (send *tl* :get-parent x (ros::time)))))

    (setq frame-coords
          (mapcan
           #'(lambda (frame)
               (let ((c (send *tl* :lookup-transform root-frame frame (ros::time))))
                 (if c (list c))))
           frame-strings))
    (ros::ros-info "coords ~A~%" frame-coords)

    (dolist (coords frame-coords)
      (let* ((parent-name (send *tl* :get-parent (send coords :name) (ros::time)))
             (parent-coords (find parent-name frame-coords
                                  :test #'string=
                                  :key #'(lambda (x) (send x :name)))))
        ;;(when parent-name (send parent-coords :assoc coords))
        ))
    (setq objs
          (mapcar #'(lambda (c)
                      (send (arrow) :newcoords c)) frame-coords))
    (objects objs)
    frame-coords))
;;
(do-until-key
 (setq frame-coords (tf->cascoords)))



