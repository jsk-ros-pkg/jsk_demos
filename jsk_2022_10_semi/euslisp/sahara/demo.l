(load "package://fetcheus/fetch-interface.l")
;git clone https://github.com/jsk-ros-pkg/jsk_pr2eus.git
;rosdep install --from-path . --ignore-src -y -r
;catkin build
;rlwrap roseus

(unless (boundp '*ri*) (fetch-init))
(objects (list *fetch*))

(send *fetch* :reset-pose)
(send *irtviewer* :draw-objects)

;send *ri* :state :potentio-vector
;send *fetch* :angle-vector (send *ri* :state :potentio-vector)
;send *ri* :angle-vector (send *fetch* :angle-vector) 5000

;(setq *rarm-end* (send *fetch* :rarm :end-coords :copy-worldcoords))
;(send *rarm-end* :translate (float-vector 10.0 10.0 -10.0))
;(send *fetch* :rarm :inverse-kinematics *rarm-end* 2000)
(send *fetch* :rarm :shoulder-y :joint-angle)
(send *fetch* :rarm :shoulder-y :joint-angle 30)
(send *fetch* :rarm :shoulder-r :joint-angle -120)
(send *fetch* :rarm :elbow-r :joint-angle 30)
(send *irtviewer* :draw-objects)

;(#<bodyset-link #X55cc21e03b50 shoulder_pan_link  32.65 0.0 746.01 / 1.309 0.0 0.0> #<bodyset-link #X55cc21e132e0 shoulder_lift_link  62.932 113.013 806.01 / 1.309 1.4 -8.327e-17> #<bodyset-link #X55cc21e0c6e0 upperarm_roll_link  72.566 148.968 590.197 / 1.309 1.4 -0.2> #<bodyset-link #X55cc22c459c8 elbow_flex_link  78.417 170.803 459.132 / -1.635 0.018 -3.108> #<bodyset-link #X55cc22d63218 forearm_roll_link  65.816 -25.761 455.538 / -1.635 0.018 -2.933> #<bodyset-link #X55cc22da9518 wrist_flex_link  57.853 -149.985 453.267 / 2.744 -1.351 -1.249> #<bodyset-link #X55cc22dd2d10 wrist_roll_link  30.014 -138.306 588.437 / 2.744 -1.351 -1.249>)
;(20.0 -70.0 80.2141 70.0 98.5487 10.0 70.0 0.0 0.0 0.0)
;(20.7292 43.1076 -34.5316 -56.8115 64.3184 -41.7296 66.5712 -0.006954 -0.753425 1.62224)