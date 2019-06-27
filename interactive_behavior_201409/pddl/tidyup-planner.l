;; tidyup-planner.l
;; Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "mbtk")

(ros::roseus "tidyup_planner")

(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :tidyup-action "package://mbtk/euslisp/tidyup-action.l")

;; base plan
(defun make-domain ()
  (let ((domain (instance pddl-domain :init :name 'tidyup-domain)))
    (send domain :requirements '(:typing :action-costs))
    (send domain :types '(item spot))
    (send domain :predicates
          '((ON ?OBJ - item ?SPOT - spot)
            (ONHAND ?OBJ - item)
            (INACCESSIBLE ?AT - spot)
            (OCCUPIED ?AT - spot)
            (OPENABLE ?AT - spot)
            (AT ?SPOT - spot)))
    (send domain :functions '((total-cost) (distance ?FROM ?TO) (prob-cost ?OBJ ?AT)))
    (mapcar #'(lambda (ac) (send domain :add :action ac))
            (list
             (instance pddl-action :init
                       :name "open-door"
                       :parameters '((?AT spot))
                       :precondition '((AT ?AT)
                                       (OPENABLE ?AT)
                                       (INACCESSIBLE ?AT))
                       :effect '((NOT (INACCESSIBLE ?AT))))
             (instance pddl-action :init
                       :name "close-door"
                       :parameters '((?AT spot))
                       :precondition '((AT ?AT)
                                       (OPENABLE ?AT)
                                       (NOT (INACCESSIBLE ?AT)))
                       :effect '((INACCESSIBLE ?AT)))
             (instance pddl-action :init
                       :name "pick"
                       :parameters '((?OBJ item) (?AT spot))
                       :precondition '((ON ?OBJ ?AT)
                                       (AT ?AT))
                       :effect '((NOT (ON ?OBJ ?AT))
                                 (ONHAND ?OBJ)
                                 (increase (total-cost) (prob-cost ?OBJ ?AT))))
             (instance pddl-action :init
                       :name "pick_f"
                       :parameters '((?OBJ item) (?AT spot))
                       :precondition '((ON ?OBJ ?AT)
                                       (AT ?AT))
                       :effect '((increase (total-cost) (prob-cost ?OBJ ?AT)))) ;; TODO: find
             (instance pddl-action :init
                       :name "place"
                       :parameters '((?OBJ item) (?AT spot))
                       :precondition '((ONHAND ?OBJ)
                                       (AT ?AT)
                                       (NOT (OCCUPIED ?AT))
                                       (NOT (INACCESSIBLE ?AT)))
                       :effect '((NOT (ONHAND ?OBJ))
                                 (ON ?OBJ ?AT)
                                 (increase (total-cost) (prob-cost ?OBJ ?AT))))
             (instance pddl-action :init
                       :name "place_f"
                       :parameters '((?OBJ item) (?AT spot))
                       :precondition '((ONHAND ?OBJ)
                                       (AT ?AT)
                                       (NOT (INACCESSIBLE ?AT)))
                       :effect '(;;(INACCESSIBLE ?AT)
                                 (OCCUPIED ?AT)
                                 (increase (total-cost) (prob-cost ?OBJ ?AT))))
             (instance pddl-action :init
                       :name "move-to"
                       :parameters '((?FROM ?TO spot))
                       :precondition '((NOT (AT ?TO))
                                       (AT ?FROM))
                       :effect '((NOT (AT ?FROM))
                                 (AT ?TO)
                                 (increase (total-cost) (distance ?FROM ?TO))))
             ))
    domain))

(defun make-problem (dists costs)
  (let ((problem (instance pddl-problem :init
                           :name 'tidyup
                           :domain 'tidyup-domain))
        (objs '((TARGET . item)
                (TARGETFRONT . spot)
                (START . spot)))
        (init '((AT START)
                (ON TARGET TARGETFRONT)
                (INACCESSIBLE SPOT0) ;; DEBUG
                (OPENABLE SPOT0)     ;; DEBUG
                (= (total-cost) 0)
                (= (distance START TARGETFRONT) 1)
                (= (distance TARGETFRONT START) 100000)
                (= (prob-cost TARGET TARGETFRONT) 0)))
        (goal '((INACCESSIBLE SPOT0)))  ;; DEBUG
        spot-syms)

    ;; spots
    (dolist (c dists)
      (push (car c) spot-syms)
      (push (cadr c) spot-syms))
    (setq spot-syms (remove 'TARGETFRONT (remove-duplicates spot-syms)))

    ;; objects
    (dolist (s spot-syms) (push `(,s . spot) objs))
    (ros::ros-info "objects: ~A" objs)
    (send problem :objects objs)

    ;; init conds
    ;; distance
    (mapcar #'(lambda (c)
                (push `(= (distance ,(car c) ,(cadr c)) ,(caddr c)) init))
            dists)
    ;; (dolist (s spot-syms)
    ;;   (push `(= (distance TARGETFRONT ,s) 100000) init))
    ;; prob cost
    (mapcar #'(lambda (c)
                (push `(= (prob-cost TARGET ,(car c)) ,(cadr c)) init))
            costs)
    (ros::ros-info "initial-condition: ~A" init)
    (send problem :initial-condition init)

    ;; goal conds
    (push `(OR ,@(mapcar #'(lambda (s)
                            `(ON TARGET ,s)) spot-syms)) goal)
    (ros::ros-info "goal-condition: ~A" goal)
    (send problem :goal-condition goal)
    (send problem :metric '(minimize (total-cost)))

    problem))

(setq *pose-scale* 1.0)
(defun pose->distance (p1 p2)
  (let ((c1 (ros::tf-pose->coords p1))
        (c2 (ros::tf-pose->coords p2)))
    (ros::ros-error "dist: ~A" (norm (send c1 :difference-position c2)))
    (floor (* *pose-scale* (norm (send c1 :difference-position c2))))))

(defun tidyup-goal-cb (server goal)
  (ros::ros-info "goal cb")
  (let* ((n (length (send goal :goal :distribution :poses)))
         (syms (dotimes (i n) (read-from-string "SPOT~d" i)))
         (poses (append (send-all (send goal :goal :distribution :poses) :pose)
                        (list (send goal :goal :pose :pose))))
         dists costs)
    (setq *domain* (make-domain))

    (speak-jp (format nil "~Aを片付けます" (send goal :goal :label)))

    (dotimes (i (length poses))
      (dotimes (j (length poses))
        (unless (= i j)
          (push (list
                 (read-from-string
                  (if (= i n) "TARGETFRONT"
                    (format nil "SPOT~d" i)))
                 (read-from-string
                  (if (= j n) "TARGETFRONT"
                    (format nil "SPOT~d" j)))
                 (pose->distance (elt poses i) (elt poses j)))
                dists))))

    (dotimes (i (length (send goal :goal :distribution :weights)))
      (push (list
             (read-from-string (format nil "SPOT~d" i))
             (floor (* 1000.0
                       (/ 1.0 (elt (send goal :goal :distribution :weights) i)))))
            costs))

    (setq *problem* (make-problem dists costs))

    (setq *graph* (pddl-plan-to-graph
                   nil
                   :domain *domain*
                   :problem *problem*
                   :failed-nodes (list 'place 'put)
                   :debug nil))
    (send *graph* :name (send *domain* :name))

    (send *graph* :write-to-pdf "action_graph.pdf")
    (unix::system "mv -f action_graph.pdf /tmp/action_graph.pdf")
    (ros::ros-info "output result to /tmp/action_graph.pdf")

    (setq *sm* (convert-smach *graph*
                              :return-success t
                              :return-fail nil
                              :add-self-return-failure-path t
                              :use-userdata t
                              ))
    (dump-loadable-structure "/tmp/action_state_machine.l" *sm*)
    (ros::ros-info "output result to /tmp/action_state_machine.l")
    (send *sm* :arg-keys :goal)
    (send *sm* :reset-state)
    (setq result (exec-state-machine *sm* `((:goal . ,(send goal :goal)))))
    (ros::ros-info "result: ~A" result)

    (when server
      (let ((res (send server :result)))
        (send res :result :success (eq (car (send *sm* :goal-state)) result))
        (send server :set-succeeded res)))
    t))

(setq *server* (instance ros::simple-action-server :init
                         "/tidyup" mbtk::TidyupAction
                         :execute-cb 'tidyup-goal-cb))

(defun main ()
  ;; main loop
  (ros::rate 1)
  (ros::ros-info "initialized. waiting for client")
  (while (ros::ok)
    (send *server* :worker)
    (ros::spin-once)
    (ros::sleep))
  (exit))

;; for debug
(defun make-sample-goal (&optional (n 3))
  (let ((msg (instance mbtk::tidyupactiongoal :init)))
    (send msg :header :stamp (ros::time-now))
    (send msg :goal :label "cup")
    (send msg :goal :pose
          (ros::coords->tf-pose-stamped
            (make-coords :pos #f(1000 2000 3000))
            "base_link"))
    (send msg :goal :distribution :header
          (instance std_msgs::header :init
                    :frame_id "base_link" :stamp (ros::time-now)))
    (send msg :goal :distribution :poses
          (mapcar #'(lambda (pos)
                      (instance geometry_msgs::posewithcovariance :init
                                :pose (ros::coords->tf-pose
                                        (make-coords :pos pos))))
                  (random-vectors n 3)))
    (let ((weights (v-abs (gaussian-random n) (make-sequence float-vector n))))
      (send msg :goal :distribution :weights (normalize-vector weights)))
    msg))

;;(tidyup-goal-cb nil (make-sample-goal))
(main)
