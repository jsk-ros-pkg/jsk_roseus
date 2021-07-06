(load "sample-robot-utils.l")
(load "sample-room.l")


;; Initialize *room* and *robot*
(defun init (&optional (broom t) (start-viewer t))
  (defparameter *room* (instance sample-room-scene :init :broom broom))
  (defparameter *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :update-descendants)
  (if start-viewer (objects (list *room* *robot*))))


;; Utility
(defun draw ()
  (when (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects)
    (unix:usleep 500000)))

(defun reset-pose ()
  (unless (v= (send *robot* :angle-vector) (send *robot* :reset-pose))
    (draw)))

(defun dual-arm-grasp-obj (lobj robj)
  (send *robot* :dual-arm-ik
        (list 
         (send lobj :handle-handle0)
         (send robj :handle-handle0))
        :debug-view t)
  (send (send *robot* :larm :end-coords) :assoc lobj)
  (send (send *robot* :rarm :end-coords) :assoc robj))


;; Go to spot
(defun go-to-spot (name)
  (send *robot* :go-to-coords (send *room* :spot name) :debug-view t))


;; Pour task
(defun pick-sushi-bottle ()
  (let ((petbottle-obj (send *room* :object "petbottle"))
        (sushi-cup-obj (send *room* :object "sushi-cup")))
    (dual-arm-grasp-obj petbottle-obj sushi-cup-obj)
    (send *robot* :dual-arm-ik
          (list
           (make-coords :pos #f(1736.53 754.746 835.385)
                        :rpy #f(-0.160248 -0.502549 0.31589)
                        :name "petbottle-pos")
           (make-coords :pos #f(1736.82 265.63 805.839)
                        :rpy #f(0.50965 0.109477 -1.08614)
                        :name "sushi-cup-pos"))
          :debug-view t)))

(defun pour-sushi-bottle ()
  (send *robot* :larm :move-end-pos #f(0 -250 50) :world :debug-view t)
  (draw)
  (send *robot* :larm-wrist-r :joint-angle -30)
  (draw)
  (send *robot* :larm-wrist-r :joint-angle 30)
  (draw))

(defun place-sushi-bottle ()
  (let ((petbottle-obj (send *room* :object "petbottle"))
        (sushi-cup-obj (send *room* :object "sushi-cup")))
    (send *robot* :dual-arm-ik
          (list
           (make-coords :pos #f(1850.0 400.0 797.5) :name "petbottle-pos")
           (make-coords :pos #f(1850.0 100.0 746.0) :name "sushi-cup-pos"))
          :debug-view t)
    (send (send *robot* :larm :end-coords) :dissoc petbottle-obj)
    (send (send *robot* :rarm :end-coords) :dissoc sushi-cup-obj)))


;; Sweep task
(defun sweep-floor ()
  (let ((broom-obj (send *room* :object "broom")))
    (send *robot* :dual-arm-ik (send broom-obj :handle) :debug-view t)

    (dotimes (i 125)
      (send broom-obj :rpy -pi/2 0 (* 0.2 (sin (/ i 10.0))))
      (send broom-obj :locate (float-vector (+ 400 (* 250 (sin (/ (incf i) 10.0)))) -1500 0) :world)
      (send *robot* :dual-arm-ik (send broom-obj :handle))
      (send *irtviewer* :draw-objects))))
