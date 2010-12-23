#!/usr/bin/env roseus

(load "models/arrow-object.l")

(ros::roseus "eus_view_frame")
(if (not (boundp '*tl*))
    (setq *tl* (instance ros::transform-listener :init)))
(setq *arrows* nil)

(defun tf->cascoords ()
  (let (frame-strings frame-coords root-frame objs)
    (ros::rate 100)
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
                 (when c
                   (send c :name frame)
                   (list c))))
           frame-strings))
    ;;(send *irtviewer* :viewer :clear)
    ;;(send-all frame-coords :draw-on :flush t)
    ;; (ros::ros-info "coords ~A~%" frame-coords)

    (dolist (coords frame-coords)
      (let* ((parent-name (send *tl* :get-parent (send coords :name) (ros::time)))
             (parent-coords (find parent-name frame-coords
                                  :test #'string=
                                  :key #'(lambda (x) (send x :name)))))
        ;;(when parent-name (send parent-coords :assoc coords))
        ))
    (setq objs
          (mapcar
           #'(lambda (c)
               (let (b)
                 (setq b (find (send c :name) *arrows* :key #'(lambda (x) (send x :name)) :test #'string=))
                 (unless b
                   (setq b (arrow :name (send c :name)))
                   (push b *arrows*))
                 (send b :move-to c :world)
                 (send b :update-descendants)
                 b)) frame-coords))
    (objects objs)
    (setq ooo objs)
    (x::window-main-one)
    frame-coords))
;;
(do-until-key
 (setq frame-coords (tf->cascoords)))



