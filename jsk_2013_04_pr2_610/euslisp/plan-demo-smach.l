;;
;;(load "package://jsk_2013_04_pr2_610/euslisp/plan-demo.l")
(load "package://jsk_2013_04_pr2_610/euslisp/setup.l")
;;(load "package://jsk_2013_04_pr2_610/euslisp/setup-for-pddl.l")
(load "package://pddl_planner/src/eus-pddl-client.l")
(load "package://jsk_2013_04_pr2_610/pddl/description.l")

;; smach
(load "package://roseus_smach/src/pddl2smach.l")
(load "package://roseus_smach/src/state-machine-ros.l")

(defun plan-to-graph (goal-condition &optional debug)
  (let ((problem (copy-object *problem*)))
    (send problem :goal-condition goal-condition)

    (setq *result* (solve-pddl-planning *domain* problem))
    (setq *graph* (make-graph-from-pddl-results (list *result*) :node-name :pprint))
    (make-readable-graph *graph* :copy nil)
    (if debug (send *graph* :write-to-pdf "debug.pdf"))
    *graph*))

(defun graph-to-smach (agraph)
  (let* ((sm
          (convert-smach agraph :return-success :success :return-fail :fail
                         ;; :use-userdata t
                         :add-self-return-failure-path nil))
         (sm-inspect (instance state-machine-inspector :init sm)))
    sm-inspect))

(defun exec-ros-smach (sm-insp &key (user-data))
  (send sm-insp :exec-state-machine :reset t))

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

(defun plan-all (&key (debug) (exec))
  (plan-to-graph
   '((clean room)
     (clean-under table-front)
     (on tray sink)
     (under chair table-front)
     (on broom laundry)
     (in cloth laundry)
     (working laundry)))

  (setq *sm-insp* (graph-to-smach *graph*))

  (when debug (defun-debug-functions *graph*))
  (when exec (exec-ros-smach *sm-insp*))

  *sm-insp*
  )

(defun move-to (&rest args)
  (let ((pl (car args)))
    (funcall (read-from-string (format nil "move-to-~A" (symbol-string pl))))
    :success))

(defun pick (&rest args)
  (let ((obj (car args))
        (pl (cadr args)))
    (funcall (read-from-string (format nil "pick-~A-~A" (symbol-string obj) (symbol-string pl))))
    :success))

(defun place (&rest args)
  (let ((obj (car args))
        (pl (cadr args)))
    (funcall (read-from-string (format nil "place-~A-~A" (symbol-string obj) (symbol-string pl))))
    :success))

(defun put (&rest args)
  (let ((obj (car args))
        (pl (cadr  args)))
    (funcall (read-from-string (format nil "put-~A-~A" (symbol-string obj) (symbol-string pl))))
    :success))

(defun open-door (&rest args)
  (let ((obj (car args)))
    (funcall (read-from-string (format nil "open-~A" (symbol-string obj))))
    :success))

(defun close-door (&rest args)
  (let ((obj (car args)))
    (funcall (read-from-string (format nil "close-~A" (symbol-string obj))))
    :success))

(defun push-button (&rest args)
  (let ((obj (car args)))
    (funcall (read-from-string (format nil "push-button-~A" (symbol-string obj))))
    :success))

(defun sweep (&rest args)
  (let ((obj (car args)))
    (funcall (read-from-string (format nil "sweep-~A" (symbol-string obj))))
    :success))

(defun sweep-under (&rest args)
  (let ((obj (car args)))
    (funcall (read-from-string (format nil "sweep-under-~A" (symbol-string obj))))
    :success))

(defun pull-chair (&rest args)
  (pull-chair-table)
  :success)

(defun push-chair (&rest args)
  (move-back-chair)
  :success)
