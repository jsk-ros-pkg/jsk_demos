

(ros::load-ros-manifest "roseus_smach")
(ros::roseus "async_join_state_machine_based_fridge_demo")

(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")

(defmethod async-join-state
  (:execute
   (userdata &key (step nil))
   (let (start-time ret clients part-async-client-lst)
     (send self :remap userdata :invert nil)
     (setq start-time (ros::time-now))
     (setq part-async-client-lst (cdr (assoc :part-async userdata)))
     (cond
      ((null part-async-client-lst) ; normal mode
       (setq clients (cdr (assoc :async userdata)))
       (when (atom clients)
         (setq clients (list clients)))
       (warning-message 2 "<normal async join> waiting action client: ~A~%" clients)
       (send-all clients :spin-once)
       (if (member nil (send-all clients :wait-for-result :timeout timeout))
           (setq ret nil) (setq ret t))
       (warn "ret: ~A~%" ret)
       (set-alist :results (send-all clients :get-result) userdata)
       (warn "result: ~A~%" (send-all clients :get-result))
       )
      (t ; partly join
       (warning-message 2 "<part async join> waiting action client:~A" part-async-client-lst)
       (send-all part-async-client-lst :spin-once)
       (if (member nil (send-all part-async-client-lst :wait-for-result :timeout timeout))
           (setq ret nil) (setq ret t))
       (warn "ret: ~A~%" ret)
       (set-alist :results (send-all part-async-client-lst :get-result) userdata)
       (warn "result: ~A~%" (send-all part-async-client-lst :get-result))
       ))
     (send self :remap userdata :invert t)
     (return-from :execute ret))))

(defun actionlib-client-state
  (name client my-controller-lst
        &key (timeout 10) (retry nil) (key #'identity) (return-success :succeeded) (return-fail :failed)
        (async nil) (return-async t))
  (send client :wait-for-server)
  (instance state :init name
            ;; main loop
            `(lambda (userdata)
               (let ((start (ros::time-now)) async-clients)
                 (send ,client :send-goal
                       (funcall (quote ,key) (cdr (assoc :goal userdata))))
                 (if ,async
                     (progn
                       (setq async-clients (cdr (assoc :async userdata)))
                       (set-alist :async (flatten (list async-clients ,client)) userdata)
                       ;; add client to controller-lst in userdata
                       (if (not (null ',my-controller-lst))
                           (progn
                             (dolist (controller ',my-controller-lst)
                               (let (client-lst)
                                 (setq client-lst (cdr (assoc controller userdata)))
                                 (set-alist controller (flatten (list client-lst ,client)) userdata)
                                 ))))

                       (ros::sleep)
                       (send ,client :spin-once)
                       ,return-async)
                   (while (ros::ok)
                     (ros::sleep)
                     (send ,client :spin-once)
                     ;;
                     (cond
                      ((eq (send ,client :get-state) actionlib_msgs::GoalStatus::*SUCCEEDED*)
                       (return ,return-success))
                      ((eq (send ,client :get-state) actionlib_msgs::GoalStatus::*ABORTED*)
                       (if ,retry
                           (send ,client :send-goal
                                 (funcall (quote ,key) (cdr (assoc :goal userdata))))
                         (return ,return-fail)))
                      ((member (send ,client :get-state)
                               (list actionlib_msgs::GoalStatus::*PREEMPTED*
                                     actionlib_msgs::GoalStatus::*RECALLED*
                                     actionlib_msgs::GoalStatus::*REJECTED*))
                       (return ,return-fail))
                      ((member (send ,client :get-state)
                               (list actionlib_msgs::GoalStatus::*ACTIVE*
                                     actionlib_msgs::GoalStatus::*PENDING*))
                       ;; user cancel
                       (if (and (vectorp (cdr (assoc :cancel userdata)))
                                (elt (cdr (assoc :cancel userdata)) 0))
                           (send ,client :cancel-goal))
                       ;; time out
                       (if (and (numberp ,timeout)
                                (< ,timeout (send (ros::time- (ros::time-now) start) :to-sec)))
                           (send ,client :cancel-goal))))
                     )))))
  )

(defun smach-exec (sm)
  (let ((insp (instance state-machine-inspector :init sm))
        (mydata (list (cons 'count 0) (cons 'hoge (list 1 2 3)))))
    (unix:sleep 1) ;; for advertize topics successfully
    (send sm :reset-state)
    (send insp :publish-structure)
    (send insp :publish-status mydata)
    (while (not (send sm :goal-reached))
      ;; (unix:sleep 1)
      (ros::spin-once)
      (send insp :publish-status mydata)
      ;; you can execute state-machine step by step
      (send sm :execute mydata :step -1))
    (send sm :active-state) ;; return reached goal state
    ))

(defun smach-exec-in-stages (sm)
  (let ((insp (instance state-machine-inspector :init sm))
        (mydata (list (cons 'count 0) (cons 'hoge (list 1 2 3)))))
    (unix:sleep 1) ;; for advertize topics successfully
    (send sm :reset-state)
    (send insp :publish-structure)
    (send insp :publish-status mydata)
    (while (not (send sm :goal-reached))
      ;; (unix:sleep 1)
      (warn "Enter:")
      (read-line)
      (ros::spin-once)
      (send insp :publish-status mydata)
      ;; you can execute state-machine step by step
      (send sm :execute mydata :step -1))
    (send sm :active-state) ;; return reached goal state
    ))

(defun start (&optional userdata)
  (ros::ros-info "    Info: start function is being executed.")
  ;; (send *ri* :angle-vector (send *pr2* :reset-pose))
  :success
  )

(defun convert (val)
  (let ((goal (instance roseus_smach::Sub5ActionGoal :init)))
    (send goal :goal :value val)
    goal))

(defun update-client-lst (userdata)
  (let ((async-client-lst (cdr (assoc :async userdata)))
        (part-async-client-lst (cdr (assoc :part-sync userdata)))
        (base-client-lst (cdr (assoc :base userdata)))
        (head-client-lst (cdr (assoc :head userdata)))
        (rarm-client-lst (cdr (assoc :rarm userdata)))
        (larm-client-lst (cdr (assoc :larm userdata)))
        (rgripper-client-lst (cdr (assoc :rgripper userdata)))
        (lgripper-client-lst (cdr (assoc :lgripper userdata)))
        (torso-client-lst (cdr (assoc :torso userdata)))
        )
    (dolist (client async-client-lst)
      (send client :spin-once)
      (ros::ros-info "client: ~A   goal state: ~A" client (send client :get-state))
      (cond
       ((equal (send client :get-state) actionlib_msgs::GoalStatus::*succeeded*)
        (ros::ros-info "Delete the client(~A)." client)
        ;; delete the client from all lists
        (setq async-client-lst (remove client async-client-lst)
              part-async-client-lst (remove client part-async-client-lst)
              base-client-lst (remove client base-client-lst)
              head-client-lst (remove client head-client-lst)
              rarm-client-lst (remove client rarm-client-lst)
              larm-client-lst (remove client larm-client-lst)
              rgripper-client-lst (remove client rgripper-client-lst)
              lgripper-client-lst (remove client lgripper-client-lst)
              torso-client-lst (remove client torso-client-lst)
              ))
       (t
        ;; nothing
        )
       ))
    ;; set those to userdata
    (set-alist :async async-client-lst userdata)
    (set-alist :part-async part-async-client-lst userdata)
    (set-alist :base base-client-lst userdata)
    (set-alist :head head-client-lst userdata)
    (set-alist :rarm rarm-client-lst userdata)
    (set-alist :larm larm-client-lst userdata)
    (set-alist :rgripper rgripper-client-lst userdata)
    (set-alist :lgripper lgripper-client-lst userdata)
    (set-alist :torso torso-client-lst userdata)
    (ros::ros-info "updated userdata: ~A" userdata)
    ))

(defun set-part-async-client-lst (userdata my-controller-lst)
  (let ((part-async-client-lst (cdr (assoc :part-async userdata)))
        hazard-assoc-lst
        )
    (setq hazard-assoc-lst
          '((:base :rgripper :lgripper)
            (:head)
            (:rarm :rgripper :base)
            (:larm :lgripper :base)
            (:rgripper :rarm :base :torso)
            (:lgripper :larm :base :torso)
            (:torso :rgripper :lgripper)
            ))
    (dolist (controller my-controller-lst)
      (dolist (ctr (assoc controller hazard-assoc-lst))
        (let ((client-lst (cdr (assoc ctr userdata))))
          (cond
           ((null (car client-lst))
            ;;no clients
            )
           (t
            (setq part-async-client-lst (append part-async-client-lst client-lst))
            )))))
    (setq part-async-client-lst (remove-duplicates part-async-client-lst))
    (set-alist :part-async part-async-client-lst userdata)
    ))

(defmacro make-check-function (user-defined-function my-controller-lst)
  `(let ()
     (cond
      ((not (functionp ',user-defined-function))
       '(lambda (userdata)
          ;; update
          (update-client-lst userdata)
          (ros::ros-info "    user function not defined. Check structure hazard.")
          (cond
           ((equal (car (cdr (assoc :async userdata))) nil)
            t)
           (t
            (cond
             ((null ',my-controller-lst)
              (warning-message 2 "my-controller-lst not defined. Wait all clients.~%")
              (set-alist :part-async nil userdata)
              nil
              )
             (t
              ;; There are some clients waiting result.
              (set-part-async-client-lst userdata ',my-controller-lst)
              (let ((part-async-client-lst (cdr (assoc :part-async userdata)))
                    )
                (cond
                 ((null part-async-client-lst)
                  t)
                 (t
                  nil))
                )))))))
      (t
       '(lambda (userdata)
          ;; update
          (update-client-lst userdata)
          (ros::ros-info "    user function found. Exec user function.")
          (,user-defined-function userdata)
          t
          )))
     ))

(defun make-hazard-remap-lst (&optional my-controller-lst (all nil))
  (let ((remap-lst '((:async . async-clients)
                     (:part-async . part-async-clients))))
    (when all
      (setq my-controller-lst '(:base :head :rarm :larm :rgripper :lgripper :torso)))
    (dolist (controller my-controller-lst)
      (case controller
        ((:base)
         (setq remap-lst (append remap-lst '((:base . base-clients)))))
        ((:head)
         (setq remap-lst (append remap-lst '((:head . head-clients)))))
        ((:rarm)
         (setq remap-lst (append remap-lst '((:rarm . rarm-clients)))))
        ((:larm)
         (setq remap-lst (append remap-lst '((:larm . larm-clients)))))
        ((:rgripper)
         (setq remap-lst (append remap-lst '((:rgripper . rgripper-clients)))))
        ((:lgripper)
         (setq remap-lst (append remap-lst '((:lgripper . lgripper-clients)))))
        ((:torso)
         (setq remap-lst (append remap-lst '((:torso . torso-clients)))))
        ))
    remap-lst
    ))

(defmacro make-async-join-based-structure-hazard-check-state-machine
  (sm-name &optional (my-controller-lst nil) (check-func nil))
  "make hazard-check-state-machine"
  `(let (st-check st-join st-exec ac-name)
     (cond
      ((boundp ',sm-name)
       (ros::ros-warn "state-machine(~A) is already created. Use different name." ',sm-name)
       ,sm-name
       )
      (t
       (setq st-check (intern (string-upcase (format nil "~A-check" ',sm-name)) *keyword-package*))
       (setq st-join (intern (string-upcase (format nil "~A-join" ',sm-name)) *keyword-package*))
       (setq st-exec (intern (string-upcase (format nil "~A-exec" ',sm-name)) *keyword-package*))
       (setq ac-name  (string-downcase (substitute  #\_ #\- (format nil "~A" ',sm-name))))
       (setq ,sm-name (instance state-machine :init))
       (setq client (instance ros::simple-action-client :init
                              ac-name roseus_smach::Sub5Action))
       (send ,sm-name :add-node
             (instance state :init st-check
                       (make-check-function ,check-func ,my-controller-lst)))
       (send (send ,sm-name :node st-check) :remap-list (make-hazard-remap-lst nil t))
       (send ,sm-name :add-node (instance async-join-state :init st-join :timeout 50
                                          :remap-list (make-hazard-remap-lst)))
       (send ,sm-name :add-node (actionlib-client-state st-exec client ',my-controller-lst :async t :timeout 40 :retry t :key 'convert))
       (send (send ,sm-name :node st-exec) :remap-list (make-hazard-remap-lst ',my-controller-lst))
       (send ,sm-name :goal-state (list :success :failure))
       (send ,sm-name :add-transition st-check st-exec t)
       (send ,sm-name :add-transition st-check st-join nil)
       (send ,sm-name :add-transition st-join st-exec t)
       (send ,sm-name :add-transition st-exec :success t)
       (send ,sm-name :add-transition st-exec :failure nil)
       (send ,sm-name :start-state st-check)
       (send ,sm-name :arg-keys
             'async-clients 'part-async-clients 'base-clients
             'head-clients 'rarm-clients 'larm-clients
             'rgripper-clients 'lgripper-clients 'torso-clients)
       ,sm-name
       ))))

(defmacro make-hsm (state-machine-name state-machine-information-lst)
  (let ((prev-sm (gensym))
        (next-sm (gensym))
        )
    `(let (state-machine-lst)
       (cond
        ((boundp ',state-machine-name)
         (ros::ros-warn "state-machine(~A) is already created. Use different name." ',state-machine-name)
         ,state-machine-name)
        (t
         (setq ,state-machine-name (instance state-machine :init))
         (send ,state-machine-name :add-node (instance state :init :start 'start))
         (dolist (sm-info ,state-machine-information-lst)
           (cond
            ((and (not (listp sm-info))
                  (derivedp (eval sm-info) state-machine))
             (ros::ros-info "Detected state-machine(~A)." sm-info)
             (send ,state-machine-name :add-node
                   (instance state :init
                             (intern (string sm-info) *keyword-package*) (eval sm-info)))
             (setq state-machine-lst (append state-machine-lst (list (intern (string sm-info) *keyword-package*))))
             )
            (t
             (send ,state-machine-name :add-node
                   (instance state :init (intern (string (car sm-info)) *keyword-package*)
                             (eval `(make-async-join-based-structure-hazard-check-state-machine
                                     ,(car sm-info) ,(cadr sm-info) ,(caddr sm-info)))))
             (setq state-machine-lst (append state-machine-lst (list (intern (string (car sm-info)) *keyword-package*))))
             )))
         (send ,state-machine-name :arg-keys
               'async-clients 'part-async-clients 'base-clients
               'head-clients 'rarm-clients 'larm-clients
               'rgripper-clients 'lgripper-clients 'torso-clients)
         (send ,state-machine-name :start-state :start)
         (send ,state-machine-name :goal-state (list :success))
         (setq prev-sm :start)
         (dolist (st-mn state-machine-lst)
           (setq next-sm st-mn)
           (send ,state-machine-name :add-transition prev-sm next-sm :success)
           (setq prev-sm st-mn)
           )
         (setq next-sm :success)
         (send ,state-machine-name :add-transition prev-sm next-sm :success)
         ,state-machine-name
         )))))

(defun demo (&optional (stages nil))
  (make-hsm
   fridge-demo
   '((base-go-to-fridge-init-pose (:rarm :larm :rgripper :lgripper :torso))
     (base-go-to-fridge (:base))
     (open-fridge-door-initial-pose (:rarm :larm :rgripper :lgripper :torso :head))
     (move-to-and-open-fridge-door (:base :rarm :rgripper))
     (move-forward-larm (:larm))
     (detach-fridge-handle (:larm :lgripper))
     (swipe-fridge-door (:larm))
     (grasp-can-posing (:rarm :larm :head))
     (move-to-can-spot (:base))
     (grasp-can-init (:rarm :larm :rgripper :lgripper :torso))
     (grasp-can-motion (:larm :lgripper))
     (go-back-from-fridge (:base))
     (close-fridge (:rarm))
     ))
  (cond
   (stages
      (smach-exec-in-stages fridge-demo)
      )
   (t
    (smach-exec fridge-demo)
    )))

(defun demo2 (&optional (stages nil))
  (make-hsm
   goto-fridge
   '((base-go-to-fridge-init-pose  (:rarm :larm :rgripper :lgripper :torso))
     (base-go-to-fridge (:base))
     ))
  (make-hsm
   open-fridge-door
   '((open-fridge-door-initial-pose (:rarm :larm :rgripper :lgripper :torso :head))
     (move-to-and-open-fridge-door (:base :rarm :rgripper))
     (move-forward-larm (:larm))
     (detach-fridge-handle (:larm :lgripper))
     (swipe-fridge-door (:larm))
     (grasp-can-posing (:rarm :larm :head))
     ))
  (make-hsm
   grasp-can
   '((move-to-can-spot (:base))
     (grasp-can-init (:rarm :larm :rgripper :lgripper :torso))
     (grasp-can-motion (:larm :lgripper))
     ))
  (make-hsm
   close
   '((go-back-from-fridge (:base))
     (close-fridge (:rarm))
     ))
  (make-hsm
   fridge-demo
   '(goto-fridge
     open-fridge-door
     grasp-can
     close
     ))
  (cond
   (stages
      (smach-exec-in-stages fridge-demo)
      )
   (t
    (smach-exec fridge-demo)
    )))

(defun demo3 (&optional (stages nil))
  (make-hsm
   fridge-demo
   '((base-go-to-fridge-init-pose)
;     (base-go-to-fridge)
     (open-fridge-door-initial-pose)
     (move-to-and-open-fridge-door)
     (move-forward-larm)
     (detach-fridge-handle)
     (swipe-fridge-door)
     (grasp-can-posing)
     (move-to-can-spot)
     (grasp-can-init)
     (grasp-can-motion)
     (go-back-from-fridge)
     (close-fridge)
     ))
  (cond
   (stages
      (smach-exec-in-stages fridge-demo)
      )
   (t
    (smach-exec fridge-demo)
    )))

(defun return-t (userdata)
  (ros::ros-info "return-t func called")
  )

(defun demo4 (&optional (stages nil))
  (make-hsm
   fridge-demo
   '((base-go-to-fridge-init-pose nil return-t)
     (base-go-to-fridge nil return-t)
     (open-fridge-door-initial-pose nil return-t)
;     (look-at-fridge nil return-t)
     (move-to-and-open-fridge-door nil return-t)
     (move-forward-larm nil return-t)
     (detach-fridge-handle nil return-t)
     (swipe-fridge-door nil return-t)
     (grasp-can-posing nil return-t)
     (move-to-can-spot nil return-t)
     (grasp-can-init nil return-t)
     (grasp-can-motion nil return-t)
     (go-back-from-fridge nil return-t)
     (close-fridge nil return-t)
     ))
  (cond
   (stages
      (smach-exec-in-stages fridge-demo)
      )
   (t
    (smach-exec fridge-demo)
    )))

(defun ppr2 ()
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  )



(warn "~%")
(warn "~%")
(warn "Caution######################################################~%")
(warn "async-join-state class differs from original one.~%")
(warn "actionlib-client-state function differs from original one.~%")
(warn "smach-exec function differs from original one.~%")
(warn "#############################################################~%")
(warn "~%")
(warn "~%")

