(load "package://eusgazebo/euslisp/eusgazebo.l")


(defun fall-arrow-object-simulation
  ()

  ;; make and view eus model
  (load "models/arrow-object.l")
  (setq *arrow* (arrow))
  (objects (list *arrow*))

  ;; generate eusgazebo instance
  (setq *eusgazebo-server* (instance eusgazebo :init))
  (send *eusgazebo-server* :add-model *arrow*)
  (unix::usleep (* 1000 1000))

  ;; start simulation
  (progn
    ;; pause simulation
    (send *eusgazebo-server* :pause-sim)
    ;; set eus model pose
    (send *arrow* :newcoords (make-coords :pos #f(0 0 1000) :rpy (list 0 1 0)))
    ;; apply eus pose to gazebo
    (send *eusgazebo-server* :eus2gzb)
    ;; unpause simulation
    (send *eusgazebo-server* :unpause-sim)
    ;; view
    (dotimes (i 500)
      (send *eusgazebo-server* :gzb2eus)
      (send *irtviewer* :draw-objects)
      (x::window-main-one)
      (unix::usleep (* 10 1000)))
    ;;(send *eusgazebo-server* :gzb2eus-loop)
    ))
