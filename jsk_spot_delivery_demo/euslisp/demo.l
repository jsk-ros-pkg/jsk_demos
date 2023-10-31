#!/usr/bin/env roseus

(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")

(load "package://spoteus/spot-interface.l")

(ros::roseus "jsk_spot_delivery_demo")

(defun deliver-to (target-node-id)
  (let: ((start-node-id ""))
    ;; wait for package to be put
    ;;; TODO

    ;; start delivery
    (if (send *ri* :go-to-spot target-node-id)
        t
        (progn
    ;;; TODO: Add recovery
          (ros::error "Failed to deliver package to ~A" target-node-id)
          (return-from deliver-to nil)))

    ;;
    (send *ri* :speak-jp (format nil "~Aからお荷物をお持ちしました" start-node-id))
    ;;; waiting for package to be taken
    (reset-body-force)
    (while (ros::ok)
        (if (check-package-taken)
          (break)
          nil))

    t))

(defun main ()
  (let ((mode 0))
      )
    )
