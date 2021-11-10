(when (not (boundp '*room73b2*))
  (require "models/room73b2-scene.l")
  (setq *room73b2* (room73b2)))

(when (not (boundp '*nao*))
  (require "package://naoeus/nao.l")
  (setq *nao* (NaoH25V50)))

(when (not (boundp '*fetch*))
  (load "package://fetcheus/fetch.l")
  (load "package://fetcheus/fetch-interface.l")
  ;;(setq *fetch* (fetch))
  (fetch-init)
  )

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2.l")
  (setq *pr2* (pr2)))

(objects (list *room73b2* *nao* *fetch* *pr2*))

(defun wait-for-human ()
  ;;(send *pr2* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *fetch* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  
  (send *irtviewer* :draw-objects)
  )

(defun write()
  (send *fetch* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *fetch* :reset-pose)
  (send *fetch* :rarm :move-end-pos #f(0 -400 250) :local :rotation-axis t :debug-view t)
  (send *fetch* :rarm :move-end-rot -90 :z :local :debug-view t)
  (send *fetch* :rarm :move-end-rot -90 :x :local :debug-view t)
  (send *irtviewer* :draw-objects)
  )


;reportの代わりにボトル
(setq *bottle* (send *room73b2* :object "room73b2-bottle"))
(send (car (send *bottle* :handle)) :draw-on :flush t)

(defun wait-for-report ()
  (send *pr2* :move-to (make-coords :pos #f(3000 500 0) :rpy (float-vector pi/2 0 0)) :world)
  (send *irtviewer* :draw-objects)
)

(defun catch-report ()
  (if (send *bottle* :parent) (send (send *bottle* :parent) :dissoc *bottle*))
  (send *pr2* :larm :inverse-kinematics
    (car (send *bottle* :handle))
    :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
    :move-target (send *pr2* :larm :end-coords)
    :rotation-axis :z
    :debug-view t)
  (send *pr2* :larm :end-coords :assoc *bottle*)
  (send *irtviewer* :draw-objects)
)

(defun carry-report ()
  (send *pr2* :larm :move-end-pos #f(0 -200 50) :world :debug-view t :look-at-target t)
  (send *pr2* :move-to (make-coords :pos #f(3000 0 0) :rpy (float-vector pi/2 0 0)) :world)
  (send *irtviewer* :draw-objects)
  (send *pr2* :move-to (make-coords :pos #f(3000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *irtviewer* :draw-objects)
  (send *pr2* :move-to (make-coords :pos #f(1000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *irtviewer* :draw-objects)
)

(defun all-task ()
  (wait-for-report)
  (catch-report)
  (carry-report)
)


;一覧
;("room73b2-73b2-ground" "room73b2-locker2" "room73b2-locker1" "room73b2-door-left" "room73b2-door-right" "room73b2-foldable-desk" "room73b2-gifuplastic-900-cart" "room73b2-kirin-steppy-black-ladder" "room73b2-broom" "room73b2-bamboo-broom" "room73b2-azuma-broom" "room73b2-azuma-short-broom" "room73b2-coe-800-shelf" "room73b2-coe-450-shelf" "room73b2-uchida-shelf-1100" "room73b2-uchida-shelf-1300" "room73b2-uchida-shelf-1300" "room73b2-uchida-shelf-1300" "room73b2-uchida-shelf-1300" "room73b2-uchida-shelf-1300" "room73b2-uchida-shelf-1300" "room73b2-bariera-1200-right" "room73b2-bariera-1200-middle3-0" "room73b2-bariera-1200-middle3-1" "room73b2-bariera-1200-middle2" "room73b2-bariera-1200-middle-0" "room73b2-bariera-1200-middle-1" "room73b2-bariera-1200-middle-2" "room73b2-bariera-1200-corner" "room73b2-wanda" "room73b2-iemon" "room73b2-georgia-emerald-mountain" "room73b2-hitachi-fiesta-refrigerator" "room73b2-chair1" "room73b2-chair0" "room73b2-empty-box" "room73b2-bottle" "room73b2-sushi-cup2" "room73b2-trashbox0" "room73b2-karimoku-table" "room73b2-hrp2-parts-drawer" "room73b2-plus-590-locker" "room73b2-sharp-52-aquostv" "room73b2-askul-1200x700-desk-0" "room73b2-askul-1200x700-desk-1" "room73b2-askul-1200x700-desk-2" "room73b2-askul-1200x700-desk-3" "room73b2-askul-1200x700-desk-4" "room73b2-askul-1200x700-desk-5" "room73b2-askul-1200x700-desk-6" "room73b2-uchida-shelf-1100" "room73b2-askul-1000x700-desk" "room73b2-uchida-shelf-1300" "room73b2-askul-1200x700-desk-7" "room73b2-cupboard-right" "room73b2-cupboard-left" "room73b2-toshiba-clacio-refrigerator" "room73b2-bariera-1400-middle" "room73b2-bariera-900-middle-0" "room73b2-bariera-900-middle-1" "room73b2-bariera-900-left" "room73b2-askul-1400-desk" "room73b2-desk-0" "room73b2-desk-1" "room73b2-desk-2" "room73b2-unknown-1200-desk-0" "room73b2-unknown-1200-desk-1" "room73b2-unknown-1200-desk-2" "room73b2-unknown-1200-desk-3" "room73b2-unknown-1200-desk-4" "room73b2-external-wall-4" "room73b2-external-wall-3" "room73b2-external-wall-2" "room73b2-external-wall-1" "room73b2-external-wall-0" "room73b2-panelwall-0" "room73b2-sushi-cup" "room73b2-mug-cup" "room73b2-tray" "room73b2-kettle" "room73b2-knife" "room73b2-sponge" "room73b2-cup" "room73b2-dish" "room73b2-kitchen-shelf" "room73b2-kitchen" "/eng2/7f/room73B2-front-of-kitchenboard" "/eng2/7f/room73B2-sink-front" "/eng2/7f/room73B2-beside-chair" "/eng2/7f/room73B2-far-chair-back" "/eng2/7f/room73B2-tmp-chair-back" "/eng2/7f/room73B2-chair-back" "/eng2/7f/room73B2-table-front" "/eng2/7f/room73B2-table-back" "/eng2/7f/room73B2-table-side" "/eng2/7f/room73B2-front-kitchen-table" "/eng2/7f/room73B2-front-of-tv" "door-spot" "coe-spot" "broom-spot" "table-spot" "cook-spot" "fridge-front-spot" "init-spot")


;(:init :kinect_head-depth :kinect_head-rgb :r_forearm_cam :l_forearm_cam :wide_stereo-right :wide_stereo-left :narrow_stereo-right :narrow_stereo-left :init :reset-manip-pose :reset-pose :base_bellow_joint :base_footprint_joint :base_laser_joint :bl_caster_l_wheel_joint :bl_caster_r_wheel_joint :bl_caster_rotation_joint :br_caster_l_wheel_joint :br_caster_r_wheel_joint :br_caster_rotation_joint :double_stereo_frame_joint :fl_caster_l_wheel_joint :fl_caster_r_wheel_joint :fl_caster_rotation_joint :fr_caster_l_wheel_joint :fr_caster_r_wheel_joint :fr_caster_rotation_joint :head_chain_cb :head_mount_joint :head_mount_kinect_ir_joint :head_mount_kinect_ir_optical_frame_joint :head_mount_kinect_rgb_joint :head_mount_kinect_rgb_optical_frame_joint :head_mount_prosilica_joint :head_mount_prosilica_optical_frame_joint :head_pan_joint :head_plate_frame_joint :head_tilt_joint :high_def_frame_joint :high_def_optical_frame_joint :imu_joint :l_elbow_flex_joint :l_forearm_cam_frame_joint :l_forearm_cam_optical_frame_joint :l_forearm_joint :l_forearm_roll_joint :l_gripper_joint :l_gripper_l_finger_joint :l_gripper_l_finger_tip_joint :l_gripper_led_joint :l_gripper_motor_accelerometer_joint :l_gripper_motor_screw_joint :l_gripper_motor_slider_joint :l_gripper_palm_joint :l_gripper_r_finger_joint :l_gripper_r_finger_tip_joint :l_gripper_tool_joint :l_shoulder_lift_joint :l_shoulder_pan_joint :l_torso_lift_side_plate_joint :l_upper_arm_joint :l_upper_arm_roll_joint :l_wrist_flex_joint :l_wrist_roll_joint :laser_tilt_joint :laser_tilt_mount_joint :left_arm_chain_cb :narrow_stereo_frame_joint :narrow_stereo_l_stereo_camera_frame_joint :narrow_stereo_l_stereo_camera_optical_frame_joint :narrow_stereo_optical_frame_joint :narrow_stereo_r_stereo_camera_frame_joint :narrow_stereo_r_stereo_camera_optical_frame_joint :projector_wg6802418_child_frame_joint :projector_wg6802418_frame_joint :r_elbow_flex_joint :r_forearm_cam_frame_joint :r_forearm_cam_optical_frame_joint :r_forearm_joint :r_forearm_roll_joint :r_gripper_joint :r_gripper_l_finger_joint :r_gripper_l_finger_tip_joint :r_gripper_led_joint :r_gripper_motor_accelerometer_joint :r_gripper_motor_screw_joint :r_gripper_motor_slider_joint :r_gripper_palm_joint :r_gripper_r_finger_joint :r_gripper_r_finger_tip_joint :r_gripper_tool_joint :r_shoulder_lift_joint :r_shoulder_pan_joint :r_torso_lift_side_plate_joint :r_upper_arm_joint :r_upper_arm_roll_joint :r_wrist_flex_joint :r_wrist_roll_joint :right_arm_chain_cb :sensor_mount_frame_joint :torso_lift_joint :torso_lift_motor_screw_joint :wide_stereo_frame_joint :wide_stereo_l_stereo_camera_frame_joint :wide_stereo_l_stereo_camera_optical_frame_joint :wide_stereo_optical_frame_joint :wide_stereo_r_stereo_camera_frame_joint :wide_stereo_r_stereo_camera_optical_frame_joint :links :base_bellow_link_lk :base_laser_link_lk :bl_caster_l_wheel_link_lk :bl_caster_r_wheel_link_lk :bl_caster_rotation_link_lk :br_caster_l_wheel_link_lk :br_caster_r_wheel_link_lk :br_caster_rotation_link_lk :fl_caster_l_wheel_link_lk :fl_caster_r_wheel_link_lk :fl_caster_rotation_link_lk :fr_caster_l_wheel_link_lk :fr_caster_r_wheel_link_lk :fr_caster_rotation_link_lk :head_chain_cb_link_lk :head_mount_kinect_ir_optical_frame_lk :head_mount_kinect_rgb_optical_frame_lk :head_mount_kinect_rgb_link_lk :head_mount_kinect_ir_link_lk :head_mount_prosilica_optical_frame_lk :head_mount_prosilica_link_lk :head_mount_link_lk :projector_wg6802418_child_frame_lk :projector_wg6802418_frame_lk :narrow_stereo_l_stereo_camera_optical_frame_lk :narrow_stereo_r_stereo_camera_optical_frame_lk :narrow_stereo_r_stereo_camera_frame_lk :narrow_stereo_l_stereo_camera_frame_lk :narrow_stereo_optical_frame_lk :narrow_stereo_link_lk :wide_stereo_l_stereo_camera_optical_frame_lk :wide_stereo_r_stereo_camera_optical_frame_lk :wide_stereo_r_stereo_camera_frame_lk :wide_stereo_l_stereo_camera_frame_lk :wide_stereo_optical_frame_lk :wide_stereo_link_lk :double_stereo_link_lk :high_def_optical_frame_lk :high_def_frame_lk :sensor_mount_link_lk :head_plate_frame_lk :head_tilt_link_lk :head_pan_link_lk :imu_link_lk :l_forearm_cam_optical_frame_lk :l_forearm_cam_frame_lk :l_gripper_l_finger_tip_link_lk :l_gripper_l_finger_link_lk :l_gripper_led_frame_lk :l_gripper_motor_accelerometer_link_lk :l_gripper_motor_screw_link_lk :l_gripper_motor_slider_link_lk :l_gripper_l_finger_tip_frame_lk :l_gripper_r_finger_tip_link_lk :l_gripper_r_finger_link_lk :l_gripper_tool_frame_lk :l_gripper_palm_link_lk :left_arm_chain_cb_link_lk :l_wrist_roll_link_lk :l_wrist_flex_link_lk :l_forearm_link_lk :l_forearm_roll_link_lk :l_elbow_flex_link_lk :l_upper_arm_link_lk :l_upper_arm_roll_link_lk :l_shoulder_lift_link_lk :l_shoulder_pan_link_lk :l_torso_lift_side_plate_link_lk :laser_tilt_link_lk :laser_tilt_mount_link_lk :r_forearm_cam_optical_frame_lk :r_forearm_cam_frame_lk :r_gripper_l_finger_tip_link_lk :r_gripper_l_finger_link_lk :r_gripper_led_frame_lk :r_gripper_motor_accelerometer_link_lk :r_gripper_motor_screw_link_lk :r_gripper_motor_slider_link_lk :r_gripper_l_finger_tip_frame_lk :r_gripper_r_finger_tip_link_lk :r_gripper_r_finger_link_lk :r_gripper_tool_frame_lk :r_gripper_palm_link_lk :right_arm_chain_cb_link_lk :r_wrist_roll_link_lk :r_wrist_flex_link_lk :r_forearm_link_lk :r_forearm_roll_link_lk :r_elbow_flex_link_lk :r_upper_arm_link_lk :r_upper_arm_roll_link_lk :r_shoulder_lift_link_lk :r_shoulder_pan_link_lk :r_torso_lift_side_plate_link_lk :torso_lift_link_lk :torso_lift_motor_screw_link_lk :base_link_lk :base_footprint_lk :torso-waist-z :larm-collar-y :larm-shoulder-p :larm-shoulder-r :larm-elbow-p :larm-elbow-r :larm-wrist-p :larm-wrist-r :rarm-collar-y :rarm-shoulder-p :rarm-shoulder-r :rarm-elbow-p :rarm-elbow-r :rarm-wrist-p :rarm-wrist-r :head-neck-y :head-neck-p :init-ending :change-visual :fullbody-inverse-kinematics :make-collision-model-for-links :init-ending :rarm-end-coords :larm-end-coords :rleg-end-coords :lleg-end-coords :head-end-coords :torso-end-coords :rarm-root-link :larm-root-link :rleg-root-link :lleg-root-link :head-root-link :torso-root-link :limb :inverse-kinematics-loop-for-look-at :gripper :camera :force-sensor :imu-sensor :get-sensor-method :get-sensors-method-by-limb :force-sensors :imu-sensors :cameras :larm :rarm :lleg :rleg :head :torso :arms :legs :look-at-hand :inverse-kinematics :inverse-kinematics-loop :look-at-target :init-pose :torque-vector :distribute-total-wrench-to-torque-method-default :calc-force-from-joint-torque :fullbody-inverse-kinematics :joint-angle-limit-nspace-for-6dof :joint-order :print-vector-for-robot-limb :calc-zmp-from-forces-moments :foot-midcoords :fix-leg-to-coords :move-centroid-on-foot :calc-walk-pattern-from-footstep-list :draw-gg-debug-view :gen-footstep-parameter :footstep-parameter :go-pos-params->footstep-list :go-pos-quadruped-params->footstep-list :support-polygons :support-polygon :make-support-polygons :make-sole-polygon :make-default-linear-link-joint-between-attach-coords :calc-static-balance-point :init :init-ending :links :joint-list :link :joint :end-coords :bodies :faces :update-descendants :angle-vector :find-link-route :link-list :make-joint-min-max-table :make-min-max-table-using-collision-check :plot-joint-min-max-table-common :plot-joint-min-max-table :calc-target-axis-dimension :calc-union-link-list :calc-target-joint-dimension :calc-inverse-jacobian :calc-gradh-from-link-list :calc-jacobian-from-link-list :calc-joint-angle-speed :calc-joint-angle-speed-gain :collision-avoidance-links :collision-avoidance-link-pair-from-link-list :collision-avoidance-calc-distance :collision-avoidance-args :collision-avoidance :move-joints :find-joint-angle-limit-weight-old-from-union-link-list :reset-joint-angle-limit-weight-old :calc-weight-from-joint-limit :calc-inverse-kinematics-weight-from-link-list :calc-nspace-from-joint-limit :calc-inverse-kinematics-nspace-from-link-list :move-joints-avoidance :inverse-kinematics-args :draw-collision-debug-view :inverse-kinematics-loop :inverse-kinematics :ik-convergence-check :calc-vel-from-pos :calc-vel-from-rot :collision-check-pairs :self-collision-check :calc-grasp-matrix :inverse-kinematics-for-closed-loop-forward-kinematics :calc-jacobian-for-interlocking-joints :calc-vel-for-interlocking-joints :set-midpoint-for-interlocking-joints :interlocking-joint-pairs :check-interlocking-joint-angle-validity :update-mass-properties :calc-inertia-matrix-from-link-list :calc-cog-jacobian-from-link-list :cog-jacobian-balance-nspace :calc-vel-for-cog :difference-cog-position :cog-convergence-check :max-torque-vector :torque-ratio-vector :calc-torque-buffer-args :calc-torque :calc-torque-without-ext-wrench :calc-torque-from-vel-acc :calc-root-coords-vel-acc-from-pos :calc-av-vel-acc-from-pos :calc-torque-from-ext-wrenches :calc-zmp :calc-cop-from-force-moment :wrench-vector->wrench-list :wrench-list->wrench-vector :calc-contact-wrenches-from-total-wrench :draw-torque :weight :centroid :inertia-tensor :preview-control-dynamics-filter :parent :descendants :inheritance :leaves :assoc :dissoc :clear-assoc :obey :disobey :newcoords :changed :worldrot :worldpos :worldcoords :copy-worldcoords :update :parentcoords :transform-vector :rotate-vector :inverse-transform-vector :inverse-transformation :transformation :transform :move-to :rotate-with-matrix :rotate :orient-with-matrix :orient :parent-vector :parent-orientation :manager :init :inverse-rotate-vector :make-pqpmodel :make-btmodel :make-collisionmodel :dimension :rot :pos :x-axis :y-axis :z-axis :name :newcoords :replace-rot :replace-pos :replace-coords :copy-rot :copy-pos :copy-coords :coords :worldrot :worldpos :worldcoords :copy-worldcoords :parentcoords :changed :reset-coords :move-to :rotate-vector :transform-vector :inverse-transform-vector :inverse-transformation :transformation :transform :rotate-with-matrix :rotate :orient-with-matrix :orient :parent-vector :parent-orientation :translate :locate :scale :euler :euler-angle :rpy :rpy-angle :rotation-angle :4x4 :prin1 :create :init :axis :difference-position :difference-rotation :move-coords :inverse-rotate-vector :vertices :draw-on :plist :get :put :name :remprop :prin1 :prin1 :warning :error :slots :methods :super :get-val :set-val)

;; (send *fetch* :rarm :move-end-pos #f(50 0 50) :local :rotation-axis t :debug-view t)
;; (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)

(defun write_I()
  
  )


(defun move-forward ()
  (send *pr2* :translate #f(100 0 0 )))

;; (send dest :draw-on :flush t :size 1000)

(defun move-arms () ;; まずは体の位置を基準に腕を動かす
  (setq dest (send *pr2* :copy-coords)) ;; pr2の座標
  (send dest :translate #f(700 -200 1000)) ;; (2400 200 700)
  (send *pr2* :rarm :inverse-kinematics
	dest
	:link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	:move-target (send *pr2* :rarm :end-coords)
	:rotation-axis nil
	:thre 30
	:debug-view t)
  (objects (list *room73b2* *nao* *fetch* *pr2* (send *pr2* :rarm :end-coords))))

