(load "models/karimoku-1200-desk-object.l")
(load "models/petbottle-object.l")
(load "models/sushi-cup-object.l")
(load "models/broom-object.l")

(defclass sample-room-scene
  :super scene-model
  :slots ())

(defmethod sample-room-scene
  (:init (&rest args &key broom (name "sample-room"))
     (let ((obj-lst 
            (list
             (send (karimoku-1200-desk) :transform
                   (make-coords :pos (float-vector 2000 500 0)))
             (send (sushi-cup) :transform
                   (make-coords :pos (float-vector 1850 100 700)))
             (send (petbottle) :transform
                   (make-coords :pos (float-vector 1850 400 700)))))
           (spot-lst
            (list
             (make-cascoords :name "table-front" :pos (float-vector 1350 500 0)))))

       (when broom
         (push (send (broom) :transform (make-coords :pos (float-vector 400 -1500 0)
                                                     :rpy (float-vector -pi/2 0 0)))
               obj-lst)
         (push (make-cascoords :name "broom-front" :pos (float-vector 400 -1250 0)
                               :rpy (float-vector -pi/2 0 0))
               spot-lst))

       (send-super :init :name name :objects (append obj-lst spot-lst)))))
