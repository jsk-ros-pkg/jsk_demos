(ros::roseus "jsk_demo_test")
(load "package://jsk_2011_07_pr2_semantic/euslisp/actions.l")
(load "package://jsk_demo_common/euslisp/pr2-move.l")
(load "package://jsk_demo_common/euslisp/pr2-action.l")
(load "package://pr2eus/pr2-interface.l")


(unless (boundp '*pr2*) (pr2))
(unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))

(pr2-tuckarm-pose :rarm)

(goto-front-of-fridge)
(open-fridge-door)
(close-fridge)
