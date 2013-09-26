(load "package://pr2eus/pr2-interface.l")

(defvar *attention-flag* "/attention_observation/flag")
(defvar *attention-id* "/attention_observation/id")

(ros::set-param *attention-flag* nil)

#|
(defun call-retry-all ()
  (ros::set-param *attention-flag* "retry-all"))

(defun call-retry-plus ()
  (ros::set-param *attention-flag* "retry-plus"))

(defun call-retry ()
  (ros::set-param *attention-flag* "retry"))
|#

;;
(defclass pr2-attention-interface
  :super pr2-interface
  :slots ())
(defmethod pr2-attention-interface
  (:wait-interpolation (&rest args)
   (let ((ret (send-super* :wait-interpolation args)))
     (unless (or (not (memq :attention-barrier (sys::list-all-catchers)))
                 (some #'identity ret))
       (throw :attention-barrier (ros::get-param *attention-flag*)))
     ;;(send self :try-check-cancel-flag)
     ret))
  (:move-to (&rest args)
   (let ((ret (send-super* :move-to args)))
     (unless (or (not (memq :attention-barrier (sys::list-all-catchers)))
                 ret)
       (throw :attention-barrier (ros::get-param *attention-flag*)))
     ;;(send self :try-check-cancel-flag)
     ret))
  (:go-pos (&rest args)
   (let ((ret (send-super* :go-pos args)))
     (unless (or (not (memq :attention-barrier (sys::list-all-catchers)))
                 ret)
       (throw :attention-barrier (ros::get-param *attention-flag*)))
     ;;(send self :try-check-cancel-flag)
     ret))
  (:go-pos-unsafe (&rest args)
   (let ((ret (send-super* :go-pos-unsafe args)))
     (unless (or (not (memq :attention-barrier (sys::list-all-catchers)))
                 ret)
       (throw :attention-barrier (ros::get-param *attention-flag*)))
     ;;(send self :try-check-cancel-flag)
     ret))
  (:move-gripper
   (arm pos &rest args)
   (let ((ret (send-super* :move-gripper arm pos args)))
     (send self :try-check-cancel-flag)
     ret))
  ;;
  (:wait-interpolation-org (&rest args)
   (send-super* :wait-interpolation args))
  (:move-to-org (&rest args)
   (send-super* :move-to args))
  (:go-pos-org (&rest args)
   (send-super* :go-pos args))
  (:go-pos-unsafe-org (&rest args)
   (send-super* :go-pos-unsafe args))
  (:move-gripper-org (&rest args)
   (send-super* :move-gripper args))
  ;;
  (:try-check-cancel-flag (&optional (func-before-throw))
   (when (memq :attention-barrier (sys::list-all-catchers))
     (let ((ret (ros::get-param *attention-flag*)))
       (when ret
         (if func-before-throw (funcall func-before-throw self))
         (throw :attention-barrier ret)))))
  (:get-cancel-flag ()
   (ros::get-param *attention-flag*))
  (:clear-cancel-flag ()
   (ros::set-param *attention-flag* nil))
  (:cancel-all-motion (&optional (flag t))
   (ros::set-param *attention-flag* flag)
   (send r-gripper-action :cancel-all-goals)
   (send l-gripper-action :cancel-all-goals)
   (send self :cancel-angle-vector)
   (send self :go-stop nil))
  (:ros-wait
   (tm &key (spin) (spin-self) (finish-check) (func-before-throw) &allow-other-keys)
   (send-super :ros-wait tm :spin spin :spin-self spin-self
               :finish-check
               (if finish-check
                   #'(lambda (sl) (prog1 (funcall finish-check sl) (send sl :try-check-cancel-flag func-before-throw)))
               #'(lambda (sl) (send sl :try-check-cancel-flag func-before-throw) nil)))
   )
  ) ;;

(unless (fboundp 'pr2-init-org)
  (setf (symbol-function 'pr2-init-org) (symbol-function 'pr2-init)))

(defun pr2-init (&rest args)
  (unless (boundp '*ri*)
    (setq *ri* (instance pr2-attention-interface :init)))
  (apply #'pr2-init-org args)
  )

(defun do-command-sequence (seq &rest args)
  (let ((ptr 0) com abr)
    (while (< ptr (length seq))
      (setq abr
            (catch :attention-barrier
              (setq com (elt seq ptr))
              (let (ret)
                (setq ret (apply com args))
                (unless ret (throw :attention-barrier :fail))
                :success
                )))
      (cond
       ((stringp abr)
        (let ((str (send *ri* :get-cancel-flag)))
          (setq abr (intern (string-upcase abr) *keyword-package*))
          ))
       ((null abr)
        (warn "abr = nil~%");;
        (setq abr :fail)) ;; just fail action
       ((eq t abr) ;; normal cancel
        (setq abr :fail))
       )
      (send *ri* :clear-cancel-flag)
      (case abr
        ((:retry :fail :tablet) )
        (:retry-plus
         (decf ptr 1)
         (if (< ptr 0) (setq ptr 0)))
        (:retry-all
         (setq ptr 0))
        (t (incf ptr))
       ))
    ))

(defun do-command-sequence-with-recover
  (seq &rest args)
  (let ((ptr 0) com com-recover abr)
    (while (< ptr (length seq))

      ;; try command
      (setq abr
            (catch :attention-barrier
              (setq com (car (elt seq ptr))
                    com-recover (cdr (elt seq ptr)))
              (let (ret)
                (setq ret (apply com args))
                (unless ret (throw :attention-barrier :fail))
                :success
                )))

      ;; parse cancel flag
      (cond
       ((stringp abr)
        (let ((str (send *ri* :get-cancel-flag)))
          (send *ri* :clear-cancel-flag)
          (setq abr (intern (string-upcase abr) *keyword-package*))
          ))
       ((null abr)
        (warn "abr = nil~%");;
        (setq abr :fail)) ;; just fail action
       ((eq t abr) ;; normal cancel
        (setq abr :fail))
       )

      ;;
      (case abr
        ((:retry :fail :tablet)
         (when com-recover
           (apply com-recover args))
         ;;
         )
        (:retry-plus
         (when com-recover
           (apply com-recover args))
         (decf ptr 1)
         (if (< ptr 0) (setq ptr 0)))
        (:retry-all
         (when com-recover
           (apply com-recover args))
         (setq ptr 0))
        (t (incf ptr)) ;; do-next-command
       ))
    ))

(defun wait-android-query ()
  (ros::ros-info "Begin Waiting")
  (let (type from (cntr 0))
    (ros::subscribe "/Tablet/StartDemo" roseus::StringStamped
                    #'(lambda(m)
                        ;;(print (list :subscribe (send m :data)))
                        (cond
                         ((string= (send m :data)
                                   "/Tablet/other/GetGeorgia")
                          (setq from :tablet)
                          (setq type "georgia"))
                         ((string= (send m :data)
                                   "/Tablet/other/DoNothing")
                          (setq from :tablet)
                          (setq type "nothing"))
                         (t ))))
    (ros::rate 100)
    (do-until-key
     (incf cntr)
     (when (> cntr 200)
       (ros::ros-info " Waiting ....")
       (setq cntr 0))
     (ros::spin-once)
     (send *ri* :spin-once)
     (when type
       (case from
         (:tablet
          (cond
           ((string= type "nothing")
            (ros::ros-info "Cancel all motion")
            (send *ri* :cancel-all-motion "tablet")
            )
           (t )
           )
          ))
       (setq type nil from nil)
       )
     (ros::sleep))
    (ros::unsubscribe "/Tablet/StartDemo")
    ))

;;
;; using smach
;;
(defun loop-check (node-list) nil)

(defun exec-smach-with-interrupt (sm-inspector &key mydata (hz 1))
  (let (ret state-history)
    (send sm-inspector :reset-state :user-data mydata) ;; set start-state
    (unix::sleep 0.5)
    (send sm-inspector :publish-structure) ;; publish once and latch
    (send sm-inspector :publish-status mydata)
    (unix::sleep 0.5)

    ;;
    (if (boundp '*ri*) (send *ri* :ros-wait 0.0 :spin t :spin-self t))

    (setq ret
          (catch :all-barrier
            (push (send sm-inspector :state-machine :active-state) state-history)
            (while (not (send sm-inspector :state-machine :goal-reached))
              (if (boundp '*ri*) (send *ri* :ros-wait 0.0 :spin t :spin-self t))
              (send sm-inspector :publish-status mydata)
              (send sm-inspector :state-machine :execute mydata :step -1)
              (push (send sm-inspector :state-machine :active-state) state-history)
              ;; loop check
              (when (loop-check state-history)
                (throw :all-barrier :loop))
              )
            :finish
            ))
    state-history
    ))

;;; debug functions
(defun defun-debug-functions-interrupt
  (agraph &key (sleep 0.5) (return-success-value :success) (return-fail-value :fail))
  (let (func-sym)
    (dolist (arc (flatten (send-all (send agraph :nodes) :arc-list)))
      (when arc
        (let ((sym (car (read-from-string (send arc :name)))))
          (push sym func-sym))))
    (setq func-sym (remove-duplicates func-sym))
    (dolist (fs func-sym)
      (eval `(defun ,fs (&rest args)
               (pprint (list ,(intern (symbol-string fs) *keyword-package*) args))
               (let ((ret
                      (catch :attention-barrier
                        (dotimes (i 50)
                          (unix::usleep ,(round (* (/ sleep 50.0) 1000 1000)))
                          (if (boundp '*ri*) (send *ri* :try-check-cancel-flag)))
                        :nothing)))
                 (cond
                  ((eq ret :nothing) ,return-success-value)
                  (t (if (boundp '*ri*) (send *ri* :clear-cancel-flag))
                     (warn ";; interrupt with ~A~%" ret)
                     ;; (if return-all (throw :motion-barrier ret))
                     ,return-fail-value))))
            ))
    ))

;;(defmacro attention-action-template ())
