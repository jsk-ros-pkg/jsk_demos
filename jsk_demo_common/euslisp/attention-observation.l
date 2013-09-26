(require :pr2-interface "package://pr2eus/pr2-interface.l")

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
(defun loop-check (node-lst &key (continuous-same-node 3)
                             (same-route 2))
  (let ((cur (car node-lst))
        (lst (cdr node-lst)))
    ;; continuous same node
    (let ((cntr 0))
      (while (and (eq cur (pop lst)) lst)
        (incf cntr))
      (if (>= cntr (1- continuous-same-node))
          (return-from loop-check cur)))
    ;; same route
    (let ((cntr 0)
          (ln 2))
      (while (<= ln (/ (length node-lst) 2))
        (let ((seq (subseq node-lst 0 ln))
              (lstseq (subseq node-lst ln)))
          (dotimes (i (1+ (- (length lstseq) ln)))
            (if (equal seq
                       (subseq lstseq i (+ ln i)))
                (incf cntr)))
          (if (>= cntr (1- same-route))
              (return-from loop-check seq))
          (incf ln)
          ))
      )
    ))

(defun exec-smach-with-interrupt (sm-inspector &key mydata (hz 1))
  (let (ret state-history)
    (send sm-inspector :reset-state :user-data mydata) ;; set start-state
    (unix::usleep (* 500 1000))
    (send sm-inspector :publish-structure) ;; publish once and latch
    (send sm-inspector :publish-status mydata)
    (unix::usleep (* 500 1000))

    (if (boundp '*ri*) (send *ri* :ros-wait 0.0 :spin t :spin-self t))
    (let ((ret (send *ri* :try-check-cancel-flag)))
      (when ret
        (unless (y-or-n ";; there is cancel flag (~A), do you continue motion (y/n)" ret)
          (return-from exec-smach-with-interrupt))
        (send *ri* :clear-cancel-flag)
        (unix::usleep (* 500 1000))
        ))
    (setq ret
          (catch :all-barrier
            (push (send sm-inspector :state-machine :active-state) state-history)
            (while (not (send sm-inspector :state-machine :goal-reached))
              (if (boundp '*ri*) (send *ri* :ros-wait 0.0 :spin t :spin-self t))
              (send sm-inspector :publish-status mydata)
              (send sm-inspector :state-machine :execute mydata :step -1)
              (push (send sm-inspector :state-machine :active-state) state-history)
              ;; loop check
              (let (ret)
                (when (setq ret (loop-check state-history))
                  (warn ";; loop found ~A~%" ret)
                  (throw :all-barrier :loop)))
              )
            :finish
            ))
    (warn ";; finish with ~A~%" ret)
    state-history
    ))

(defmacro make-attention-action (name arg-lst &rest forms)
  ;; arg-lst should not contain attention-barrier-return and attention-action-return
  ;; forms should return nil if failed
  `(defun ,name ,arg-lst
     (let (attention-barrier-return)
       (warn ";; exec ~A~%" (quote ,name))
       (setq attention-barrier-return
             (catch :attention-barrier
               (let ((attention-action-return (progn ,@forms)))
                 (unless attention-action-return
                   (throw :attention-barrier :fail))
                 :success
                 )))
       (warn ";; exec ~A finish with ~A~%" (quote ,name) attention-barrier-return)
       (cond
        ((stringp attention-barrier-return)
         (let ((str (send *ri* :get-cancel-flag)))
           (setq attention-barrier-return
                 (intern (string-upcase attention-barrier-return) *keyword-package*))
           ))
        ((null attention-barrier-return)
         (warn ";; attention-barrier-return = nil~%");;
         (setq attention-barrier-return :fail)) ;; just fail action
        ((eq t attention-barrier-return) ;; normal cancel
         (setq attention-barrier-return :fail))
        )
       (send *ri* :clear-cancel-flag)
       (unless (or (eq attention-barrier-return :fail)
                   (eq attention-barrier-return :success))
         (warn ";; throw ~A~%" attention-barrier-return)
         (throw :all-barrier attention-barrier-return))
       attention-barrier-return)))

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
      (eval `(make-attention-action ,fs (&rest args)
                                    (progn
                                      (pprint (list (quote ,fs) args))
                                      (dotimes (i 50)
                                        (unix::usleep ,(round (* (/ sleep 50.0) 1000 1000)))
                                        (if (boundp '*ri*) (send *ri* :try-check-cancel-flag)))
                                      t))))
    func-sym
    ))
#|
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
    func-sym
    ))
|#

;;
;; test function for fridge_demo
;;
(setq *demo-type* :short)
(setq *finish-type* :pass-to-human)
(make-attention-action move-to (&rest args)
  (let ((dst (car (last args)))
        (src (car (last (butlast args)))))
    (pprint (list :move-to src dst))
    (cond
     ((and (eq src start)
           (eq dst frontfridge))
      (case *demo-type*
        (:map (goto-front-of-fridge))
        (t
         ;; do nothing
         t))
      )
     ((and (eq src frontfridge)
           (eq dst pregrasp))
      (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
      (ros::ros-info "GO POS UNSAFE for grasping can")
      (send *ri* :go-pos-unsafe 0 0 (rad2deg 0.38))
      (send *ri* :go-pos-unsafe 0.185 -0.25 0) ;;
      (send *ri* :wait-interpolation)
      )
     ((and (eq src pregrasp)
           (eq dst preclose))
      (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
      (ros::ros-info "GO POS UNSAFE for grasping can")
      (send *ri* :go-pos-unsafe -0.3 0.05 -20)
      (send *ri* :wait-interpolation)
      )
     ((or (and (eq src preclose)
               (eq dst start))
          (and (eq src frontfridge)
               (eq dst start)))
      (case *demo-type*
        (:map
         (goto-initial-position :finish-type *finish-type*))
        (t
         (goto-initial-position
          :return-to-initial-position nil :turnp t
          :finish-type *finish-type*)))
      )
     ) ;; /cond
    nil
    ))

(make-attention-action move-recoverly (&rest args)
  (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
  (send *ri* :go-pos -0.25 -0.10 0)
  ;;(send *ri* :go-pos-unsafe -0.25 -0.10 0)
  (send *ri* :wait-interpolation)
  t
  )

(make-attention-action open-door (&rest args)
  (open-fridge-door :torso-lift 25 :head-pitch 14)
  )

(make-attention-action grasp-can (&rest args)
  (grasp-can :move nil)
  )

(make-attention-action close-door (&rest args)
  (close-fridge)
  )

(make-attention-action try-close (&rest args)
  (close-fridge)
  )

#|
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
|#
