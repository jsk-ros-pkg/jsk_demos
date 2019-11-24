(require :state-machine
         "package://roseus_smach/src/state-machine.l")
(require :state-machine-actionlib
         "package://roseus_smach/src/state-machine-actionlib.l")

(load "package://elevator_move_base_pr2/src/check-elevator-open.l")
(load "package://elevator_move_base_pr2/src/utils.l")


(defun move-inside-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (instance state :init :set-param-for-elevator-outside
                                 'tolerance-loose))
    (send sm :add-node (actionlib-client-state
                        :move-elevator-front move-client :timeout 30 :retry nil
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-elevator-front)
          :remap-list '((:goal . elevator-front-coords)))
    (send sm :add-node (instance state :init :set-param-for-elevator-inside
                                 'tolerance-strict))
    (send sm :add-node
          (instance state :init :check-elevator-open
                    '(lambda (x) (check-elevator-open :timeout 300))))
    (send sm :add-node (actionlib-client-state
                        :move-elevator-inside move-client :timeout 40 :retry nil
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-elevator-inside)
          :remap-list '((:goal . inside-coords)))
    (send sm :arg-keys 'elevator-front-coords 'inside-coords)
    (send sm :goal-state (list :success :fail))
    (send sm :start-state :set-param-for-elevator-outside)
    (send sm :add-transition
          :set-param-for-elevator-outside :move-elevator-front t)
    (send sm :add-transition
          :move-elevator-front :set-param-for-elevator-inside :succeeded)
    (send sm :add-transition :move-elevator-front :fail :failed)
    (send sm :add-transition
          :set-param-for-elevator-inside :check-elevator-open t)
    (send sm :add-transition :check-elevator-open :move-elevator-inside t)
    (send sm :add-transition :check-elevator-open :fail nil)
    (send sm :add-transition :move-elevator-inside :success :succeeded)
    (send sm :add-transition :move-elevator-inside :fail :failed)
    sm))
