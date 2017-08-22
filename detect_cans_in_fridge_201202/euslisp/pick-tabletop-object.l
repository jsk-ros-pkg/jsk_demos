;; pick-tabletop-object.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(defparameter *detection-topic* "/kinect_head/depth_registered/ObjectDetection")
(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")

(defun pr2-pick-tray-pose (&key (arm :arms) (torso))
  (when torso
    (send *pr2* :torso :waist-z :joint-angle torso))
  (case arm
    (:larm
     (send *pr2* :larm :shoulder-p :joint-angle -20)
     (send *pr2* :larm :shoulder-r :joint-angle 90)
     (send *pr2* :larm :collar-y :joint-angle 60)
     (send *pr2* :larm :elbow-p :joint-angle -117)
     (send *pr2* :larm :elbow-r :joint-angle 0)
     (send *pr2* :larm :wrist-p :joint-angle -6)
     (send *pr2* :larm :wrist-r :joint-angle 0))
    (:rarm
     (send *pr2* :rarm :shoulder-p :joint-angle -20)
     (send *pr2* :rarm :shoulder-r :joint-angle -90)
     (send *pr2* :rarm :collar-y :joint-angle -60)
     (send *pr2* :rarm :elbow-p :joint-angle -117)
     (send *pr2* :rarm :elbow-r :joint-angle 0)
     (send *pr2* :rarm :wrist-p :joint-angle -6)
     (send *pr2* :rarm :wrist-r :joint-angle 0))
    (:arms
     (pr2-pick-tray-pose :arm :larm :torso torso)
     (pr2-pick-tray-pose :arm :rarm :torso torso)))
  (send *pr2* :angle-vector))

(defun pick-tabletop-object (obj &key (offset 100) (arm :larm) (flip-z-if-fail t) (rotation-axis))
  (let ((cds (send obj :copy-worldcoords))
        pregrasp-av grasp-av)
    (send cds :translate (float-vector (- offset) 0 0))
    (setq pregrasp-av
          (send *pr2* :inverse-kinematics cds
                      :move-target (send *pr2* arm :end-coords)
                      :rotation-axis rotation-axis
                      :link-list (send *pr2* :link-list
                                       (send *pr2* arm :end-coords :parent)
                                       (send *pr2* :link "toarso_lift_link"))
                      :debug-view nil))
    (unless pregrasp-av
      (unless flip-z-if-fail (return-from pick-tabletop-object nil))
      (send cds :translate (float-vector offset 0 0))
      (send cds :rotate pi :z)
      (send cds :translate (float-vector (- offset) 0 0))
      (setq pregrasp-av
            (send *pr2* :inverse-kinematics cds
                        :move-target (send *pr2* arm :end-coords)
                        :rotation-axis rotation-axis
                        :link-list (send *pr2* :link-list
                                         (send *pr2* arm :end-coords :parent)
                                         (send *pr2* :link "toarso_lift_link"))
                        :debug-view nil))
      (unless pregrasp-av (return-from pick-tabletop-object nil)))
    ;;
    (setq cds (send *pr2* arm :end-coords :copy-worldcoords))
    (send cds :translate (float-vector offset 0 0))
    (setq grasp-av
          (send *pr2* :inverse-kinematics cds
                        :move-target (send *pr2* arm :end-coords)
                        :link-list (send *pr2* :link-list
                                         (send *pr2* arm :end-coords :parent)
                                         (send *pr2* :link "toarso_lift_link"))
                        :debug-view nil))
    (unless grasp-av
      (return-from pick-tabletop-object nil))
    ;;
    (send *ri* :stop-grasp arm :wait t)
    (send *ri* :angle-vector-sequence
          (list pregrasp-av grasp-av) (list 5000 2000))
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp arm)))
;;
(defun pick-serial (&optional (arm :larm))
  (pr2-pick-tray-pose :arm arm :torso 180)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (let ((serial (check-detection :type "serial" :speak nil)))
    (pick-tabletop-object serial :offset 150)))
;;
(defun pick-bowl (&optional (arm :rarm))
  (pr2-pick-tray-pose :arm arm :torso 180)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (let ((bowl (check-detection :type "red_bowl" :speak nil)))
    (send bowl :rotate -pi/2 :y)
    (send bowl :translate (float-vector -70 0 0) :world)
    (pick-tabletop-object bowl :arm arm :offset 100)))

(provide :pick-tabletop-object) ;; end of pick-tabletop-object.l
