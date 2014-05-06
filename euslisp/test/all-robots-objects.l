;; euslisp/test/all-robots-objects.l is wrappper to dump image file in euslisp/build directory
(setq *image-build-dir* (format nil "~A/build" (read-from-string (read-line (piped-fork "rospack" "find" "euslisp")))))
(load "irteus/test/all-robots-objects.l")
