#!/usr/bin/env roseus
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id: $
;;;
;;; Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
;;;
;;; This software is a collection of EusLisp code for robot applications,
;;; which has been developed by the JSK Laboratory for the IRT project.
;;; For more information on EusLisp and its application to the robotics,
;;; please refer to the following papers.
;;;
;;; Toshihiro Matsui
;;; Multithread object-oriented language euslisp for parallel and
;;;  asynchronous programming in robotics
;;; Workshop on Concurrent Object-based Systems,
;;;  IEEE 6th Symposium on Parallel and Distributed Processing, 1994
;;;
;;; Permission to use this software for educational, research
;;; and non-profit purposes, without fee, and without a written
;;; agreement is hereby granted to all researchers working on
;;; the IRT project at the University of Tokyo, provided that the
;;; above copyright notice remains intact.  
;;;

(defclass kinect-bvh-robot-model
  :super bvh-robot-model
  :slots ())
(defmethod kinect-bvh-robot-model
  (:init (&rest args)
   (send-super* :init args)
   (setq rarm (list
	       (find '/left_shoulder  links :key #'(lambda (x) (send x :name)))
	       (find '/left_elbow     links :key #'(lambda (x) (send x :name))))
	 larm (list
	       (find '/right_shoulder links :key #'(lambda (x) (send x :name)))
	       (find '/right_elbow    links :key #'(lambda (x) (send x :name))))
	 rleg (list
	       (find '/left_hip       links :key #'(lambda (x) (send x :name)))
	       (find '/left_knee      links :key #'(lambda (x) (send x :name))))
	 lleg (list
	       (find '/right_hip      links :key #'(lambda (x) (send x :name)))
	       (find '/right_knee     links :key #'(lambda (x) (send x :name))))
	 torso (list
		(find '/torso         links :key #'(lambda (x) (send x :name))))
	 head  (list
		(find '/neck          links :key #'(lambda (x) (send x :name)))))
  ;;
   (mapcar #'(lambda (limb name)
	       (eval `(let ((l (find-if #'(lambda (x) (eq (send x :name) 'site)) (send (car (last ,limb)) :descendants))))
                        (when l
                          (send l :name ',name)
			  (setq ,limb (append ,limb (list l)))))))
	   (list 'rarm 'larm 'rleg 'lleg 'head)
	   (list '/right_hand '/left_hand '/right_foot '/left_foot '/head))
   ;;
   (send self :init-end-coords)
   (send self :init-root-link)
   ;;
   ;;;
   self) ;; init
  (:init-end-coords
   ()
   (let (parent-link)
     ;;
     (setq parent-link (find '/left_hand links :key #'(lambda (x) (send x :name))))
     (setq rarm-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send rarm-end-coords :rotate -pi/2 :x)
     (send rarm-end-coords :rotate  pi   :z)
     (send parent-link :assoc rarm-end-coords)
     ;;
     (setq parent-link (find '/right_hand links :key #'(lambda (x) (send x :name))))
     (setq larm-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send larm-end-coords :rotate  pi   :x)
     (send parent-link :assoc larm-end-coords)
     ;;
     (setq parent-link (find '/left_foot links :key #'(lambda (x) (send x :name))))
     (setq rleg-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send rleg-end-coords :rotate -pi/2 :x)
     (send rleg-end-coords :rotate  pi/2 :z)
     (send parent-link :assoc rleg-end-coords)
     ;;
     (setq parent-link (find '/right_foot links :key #'(lambda (x) (send x :name))))
     (setq lleg-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send lleg-end-coords :rotate -pi/2 :x)
     (send lleg-end-coords :rotate  pi/2 :z)
     (send parent-link :assoc lleg-end-coords)
     ;;
     (setq parent-link (find '/torso links :key #'(lambda (x) (send x :name))))
     (setq torso-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send torso-end-coords :rotate  pi/2 :y)
     (send torso-end-coords :rotate -pi/2 :x)
     (send parent-link :assoc torso-end-coords)
     ;;
     (setq parent-link (find '/head links :key #'(lambda (x) (send x :name))))
     (setq head-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send head-end-coords :rotate  pi :y)
     (send parent-link :assoc head-end-coords)
     ))
  ) ;; kinect-bvh-robot-model

(defun make-kinect-bvh-robot-model ()
  (instance kinect-bvh-robot-model :init :tree
            '((root /torso)
              (offset #f(0.0 0.0 0.0))
              (channels 6 xposition yposition zposition zrotation xrotation yrotation)
              ((joint /left_hip)
               (offset #f(90 -200 0))
               (channels 3 zrotation xrotation yrotation)
               ((joint /left_knee)
                (offset #f(0 -400 0))
                (channels 3 zrotation xrotation yrotation)
                ((end /left_foot)
                 (offset #f(0 -380 0)))))
              ((joint /right_hip)
               (offset #f(-90 -200 0))
               (channels 3 zrotation xrotation yrotation)
               ((joint /right_knee)
                (offset #f(0 -400 0))
                (channels 3 zrotation xrotation yrotation)
                ((end /right_foot)
                 (offset #f(0 -380 0)))))
              ((joint /left_shoulder)
               (offset #f(140 190 0))
               (channels 3 zrotation xrotation yrotation)
               ((joint /left_elbow)
                (offset #f(280 0 0))
                (channels 3 zrotation xrotation yrotation)
                ((end /left_hand)
                 (offset #f(260 0 0)))))
              ((joint /right_shoulder)
               (offset #f(-140 190 0))
               (channels 3 zrotation xrotation yrotation)
               ((joint /right_elbow)
                (offset #f(-280 0 0))
                (channels 3 zrotation xrotation yrotation)
                ((end /right_hand)
                 (offset #f(-260 0 0)))))
              ((joint /neck)
               (offset #f(0 200 0))
               (channels 3 zrotation xrotation yrotation)
               ((end /head)
                (offset #f(0 200 0)))))
            )) ;; make-kinect-bvh-robot-model

;;
;; Kinect
;;
;; reading data from kinect

(defmethod kinect-bvh-robot-model
  (:copy-state-to
   (robot &rest args)
   (send-super* :copy-state-to robot
		:conds '(((:larm :rarm) . (nil shoulder elbow nil))
			 ((:lleg :rleg) . (hip knee nil))
			 ((:head) . (neck)))
		args)
   (send robot :rotate -pi/2 :x)
   (send robot :rotate  pi/2 :z)
   )
  (:bvh-offset-rotate
   (name)
   (let ((r (unit-matrix 3)))
     (case name
       ((/left_shoulder)
	(setq r (rotate-matrix r (deg2rad -90) :z)))
       ((/right_shoulder)
	(setq r (rotate-matrix r (deg2rad 90) :z)))
       )
     r))
  (:fix-joint-order (jo limb)
   (case limb
     ((:larm :rarm)
      (mapc #'(lambda (x) (replace-elm x :y :x)) jo)
      (mapc #'(lambda (x) (replace-elm x :p :y)) jo)
      (mapc #'(lambda (x) (replace-elm x :r :z)) jo))
     (t
      (mapc #'(lambda (x) (replace-elm x :y :y)) jo)
      (mapc #'(lambda (x) (replace-elm x :p :x)) jo) ;; cub
      (mapc #'(lambda (x) (replace-elm x :r :z)) jo))) ;; cyl
   jo)
  (:fix-joint-angle
   (i limb joint-name joint-order a)
   (when (and (memq limb '(:rarm))
	      (memq (elt joint-order i) '(:y)))
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:larm))
	      (memq (elt joint-order i) '(:z)))
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:rleg))
	      (memq (elt joint-order i) '(:x :y)))
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:lleg))
	      (memq (elt joint-order i) '(:z :x)))
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:head))
	      (memq (elt joint-order i) '(:x :y)))
     (setf (elt a i) (- (elt a i)))
     )

   a)
  ) ;; kinect-bvh-robot-model

(defmethod kinect-bvh-robot-model
  (:copy-state-from-tf
   (tf-listener)
   (let (from to m (tm (ros::time)) (ret t) (last-time))
     (dotimes (i 4)
       (setq last-time (send tf-listener :get-latest-common-time "map" (format nil "/torso_~d" i)))
       (if last-time (setq last-time (send (ros::time- (ros::time-now) last-time) :to-sec)))
       (when (and last-time (< last-time 1))
	 (dolist (j (cdr (send self :joint-list)))
	   (setq from (format nil "~a_~d" (string-downcase (send (send j :parent-link) :name)) i)
		 to   (format nil "~a_~d" (string-downcase (send j :name)) i))
	   (setq m (send tf-listener :lookup-transform from to tm))
	   (if m
	       (send j :joint-angle (map cons #'rad2deg (matrix-log (send m :worldrot))))
	     (setq ret nil)))
	 (setq m (send tf-listener :lookup-transform "/map" (format nil "/torso_~d" i) tm)) ;;; change from /map to /base_footprint for pr2
	 (if m
	     (send (car (send self :joint-list))
		   :joint-angle (coerce (append (coerce (send m :worldpos) cons)
						(map cons #'rad2deg (matrix-log (send m :worldrot)))) float-vector))
	   (setq ret nil))
	 (return-from :copy-state-from-tf ret)))
     nil))
  )

;;
;; Kinect Body Tracker Example
;;
#-:ros
(unless (find-package "ROS") (make-package "ROS"))
#+:ros
(defun test-kinect (&key robot fname loop-hook) ;; :fname "test-kinect.bvh", loop-hook is a function to call inside do-until-key loop
  (let ((floor (make-cube 2000 1000 10 :pos #f(1000 0 0))) f)
    (if fname (setq f (open fname :direction :output)))
    (ros::roseus "kinect_bvh")
    (if (not (boundp '*tl*))
        (setq *tl* (instance ros::transform-listener :init)))
    (setq b (make-kinect-bvh-robot-model)) ;; Kinect Human Model
    (objects (list floor b))
    (if robot (objects robot))
    ;;(defun find-node (name) (find name *arrows* :key #'(lambda (x) (send x :name)) :test #'string=))
    (if f (send b :dump-hierarchy f))
    ;;;
    ;;; Main Loop
    ;;;
    (do-until-key
     (let ()
       (when (send b :copy-state-from-tf *tl*) ;; copy kinect human tracker information to human body
         ;;
         ;; track human body
         ;;
	 (format t "kinect robot : larm pos=~A, rarm pos=~A (world)~%" ;; display arm tool coordinates in world coordinates
		 (send b :larm :end-coords :worldpos)
		 (send b :rarm :end-coords :worldpos))
	 (format t "             : larm pos=~A, rarm pos=~A (local)~%" ;; display arm tool cooridnates with reference to human torso coordinates
		 (send (send (send b :torso :end-coords) :transformation (send b :larm :end-coords)) :worldpos)
		 (send (send (send b :torso :end-coords) :transformation (send b :rarm :end-coords)) :worldpos)))
       (when robot
	 (send b :copy-state-to robot))
       (if (functionp loop-hook) (funcall loop-hook))
       (send (get *viewer* :pickviewer) :look-all)
       (x::window-main-one)
       (if f (send b :dump-motion f))
       )) ;; do-until-key
    ))

;; (test-kinect)

