(load "irteus/demo/sample-robot-model.l")
(load "models/broom-object.l")


(defmethod sample-robot
  (:get-legcoords ()
    (midcoords 0.5
              (send self :lleg :end-coords :copy-worldcoords)
              (send self :rleg :end-coords :copy-worldcoords)))

  (:go-to (x y theta &key debug-view)
    (send self :calc-walk-pattern-from-footstep-list
          (send self :go-pos-params->footstep-list x y theta)
          :debug-view debug-view)
    (send self :get-legcoords))

  (:go-to-coords (c &key debug-view)
    (let* ((dest (send (send self :get-legcoords) :transformation c))
           (x (elt (send dest :worldpos) 0))
           (y (elt (send dest :worldpos) 1))
           (theta (rad2deg (caar (send dest :rpy-angle)))))
      (print (list x y theta))
      (send self :go-to x y theta :debug-view debug-view)))

  (:dual-arm-ik (target-lst &key debug-view (view-target :midpoint))
    (let* ((move-target (send self :arms :end-coords))
           (link-list (mapcar #'(lambda (mt) (send self :link-list mt))
                              (send-all move-target :parent))))

      (send self :head :look-at
            (apply #'midpoint 0.5 (send-all target-lst :worldpos)))
      (send self :inverse-kinematics target-lst
            :link-list link-list :move-target move-target
            :stop 500 :thre '(10 10)
            :rotation-axis '(nil nil) :debug-view debug-view :dump-command nil)))
)
