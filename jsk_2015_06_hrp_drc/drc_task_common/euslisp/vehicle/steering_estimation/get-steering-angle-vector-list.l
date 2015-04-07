(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")

(setq *buffer* nil)

(hrp2jsknt-init)
(do-until-key
 (setq av (send *ri* :state :potentio-vector))
 (push av *buffer*)
 (warn "~A~%" av)
 (unix::usleep 200)
 )
(dump-loadable-structure "hoge.l" *buffer*)
