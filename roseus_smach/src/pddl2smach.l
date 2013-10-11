;;
;; Convert from PDDL result (include failure recovery) to State Machine
;;
;; Author: Manabu Saito <saito@jsk.t.u-tokyo.ac.jp>
;;
(require :state-machine "package://roseus_smach/src/state-machine.l")

;; TODO
;; start, goal state
;; for many arc
;; return value

(defun rmnl (str) ;; remove new line
  (map string #'(lambda(c)(if (= c #\\) #\_ c)) str))
;  str)

(defun convert-smach (gr &key (readable t)
                         (use-userdata nil)
                         (return-success t)
                         (return-fail nil)
                         (add-self-return-failure-path nil)
                         (add-start-node)
                         (use-sub-machine nil))
  (let ((sm (instance state-machine :init))
        goals (nametbl (make-hash-table :test #'string=))
        (ns (send gr :name)))

    ;; add nodes
    (dolist (n (send gr :nodes))
      (let* ((act (send n :neighbor-action-alist))
             (name (rmnl (send n :name)))
             (func-name (caar (last act))))

        (when (and readable func-name)  ;; check uniq func-name
          (while (gethash func-name nametbl)
            (setq func-name (format nil "~A " func-name)))
          (setf (gethash func-name nametbl) n)
          (setf (get n :func-name) func-name))

        (cond ((null act) ;; no more action
               ;; goal
               (when readable
                 (setq name (format nil "goal~A" (length goals)))
                 (while (gethash name nametbl)
                   (setq name (format nil "~A " name)))
                 (setf (gethash name nametbl) n)
                 (setf (get n :func-name) name)
                 )
               (push name goals))
              (t ;; last arc is correct action, (first action is failure action)
               (let* ((sym (read-from-string func-name))
                      func)
                 (when (find-package (string-upcase ns))
                   (setf (elt sym 0)
                         (intern (string-upcase (car sym)) (find-package ns))))
                 (if use-userdata
                     (setq func
                           `(lambda-closure nil 0 0 (userdata)
                                            (apply ',(car sym) userdata ',(cdr sym))))
                   (setq func
                         `(lambda-closure nil 0 0 (x) (apply ',(car sym) ',(cdr sym)))))
                 (cond
                  (readable
                   (if use-sub-machine
                       (let ((sub-sm (instance state-machine :init)))
                         (send sm :add-node (instance state :init func-name sub-sm)))
                     (send sm :add-node (instance state :init func-name func))))
                  (t (if use-sub-machine
                         (let ((sub-sm (instance state-machine :init)))
                           (send sm :add-node (instance state :init name sub-sm)))
                       (send sm :add-node (instance state :init name func)))))
                 ))
              )))
    (send sm :goal-state goals)

    ;; add arcs
    (dolist (n (send gr :nodes))
      (let ((act (send n :neighbor-action-alist))
            (name (rmnl (send n :name))))
        (cond
         (readable
          (cond ((null act))
                (t
                 (send sm :add-transition (get n :func-name) (get (cdar (last act)) :func-name)
                       return-success)
                 (when (< 1 (length act))
                   (send sm :add-transition (get n :func-name) (get (cdar act) :func-name)
                         return-fail)))))
         (t
          (cond ((null act))
                (t
                 (send sm :add-transition name (rmnl (send (cdar (last act)) :name)) return-success)
                 (when (< 1 (length act))
                   (send sm :add-transition name (rmnl (send (cdar act) :name)) return-fail))))))
        ))

    ;; add self-return fail for nodes which have no failure path.
    (when add-self-return-failure-path
      (let ((nodes (send sm :nodes)))
        (setq nodes (set-difference nodes (send sm :goal-state)))
        (setq nodes
              (remove-if #'(lambda (n) (find-if #'(lambda (a) (equal return-fail (send a :name)))
                                                (send n :arc-list))) nodes))
        (dolist (n nodes)
          (send sm :add-transition (send n :name) (send n :name) return-fail))
        ))

    (when use-sub-machine
      (let ((nds (send sm :nodes)))
        (dolist (nd nds)
          (let ((subm (send nd :submachine)))
            (when subm
              ;;
              (send subm :add-node (instance state :init "start" 'func-start))
              (send subm :goal-state (list return-success return-fail))
              (send subm :start-state "start")
              (send subm :add-transition "start" return-success return-success)
              (send subm :add-transition "start" return-fail return-fail)
              ;;
              ))
          )))
    ;; select a node as start-node
    ;; now this is not always true
    ;; (send sm :start-state (send (send gr :start-state) :name))
    (send sm :start-state
          (if readable
              (get (car (last (send gr :nodes))) :func-name)
            (rmnl (send (car (last (send gr :nodes))) :name))))
    (send sm :reset-state)

    sm ))

;;
;; convert pddl-domain -> euslisp template of action definition
;;
(defun output-domain-template (filename domain)
  (with-open-file
   (fd filename :direction :output)
   (format fd ";;~%;; Please Fill in the Actions~%;; DOMAIN : ~a~%;;"
	   (send domain :name))
   (format fd "~%~%")
   ;;
   (pprint
    `(let ((ns ,(string-upcase (send *domain* :name))))
       (when (and ns (not (find-package ns)))
	 (make-package ns)))
    fd)
   (format fd "~%~%")
   ;;
  (dolist (act (send domain :action))
    (format fd "(defun ~a::~a~%  ~a~%t )~%"
	    (send domain :name) (send act :name)
	    (mapcan
	     #'(lambda(x)(if (= 1 (length x)) x (subseq x 0 (1- (length x)))))
	     (send act :parameters))
	    ))
  ;;
  (format fd "~%;; END OF THE FILE~%")
    ))

(defun pddl-graph-to-smach (agraph &rest args &key (return-success :success)
                                   (return-fail :fail) (add-self-return-failure-path t)
                                   &allow-other-keys)
  (let* ((sm
          (apply #'convert-smach agraph
                         :return-success return-success :return-fail return-fail
                         :add-self-return-failure-path add-self-return-failure-path
                         args
                         ))
         (sm-inspect (instance state-machine-inspector :init sm)))
    sm-inspect))

;;; debug functions
(defun defun-debug-functions (agraph &key (sleep 0.5) (return-success-value :success))
  (let (func-sym)
    (dolist (arc (flatten (send-all (send agraph :nodes) :arc-list)))
      (when arc
        (let ((sym (car (read-from-string (send arc :name)))))
          (push sym func-sym))))
    (setq func-sym (remove-duplicates func-sym))
    (dolist (fs func-sym)
      (eval `(defun ,fs (&rest args)
               (pprint (list ,(intern (symbol-string fs) *keyword-package*) args))
               (unix::usleep ,(round (* sleep 1000 1000)))
               ,return-success-value)))
    ))
