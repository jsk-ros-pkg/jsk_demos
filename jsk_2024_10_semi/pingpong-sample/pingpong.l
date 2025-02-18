#!/usr/bin/env roseus
#|

35.E8-irteusgl$ (send *ri* :state :potentio-vector)
;; #<rotational-joint #X55a52b17e400 head_tilt_joint> :joint-angle(75.57) violate max-angle(74.2702)
;; #<rotational-joint #X55a52b25ba20 r_wrist_flex_joint> :joint-angle(-4.76341) violate max-angle(-5.72958)
#f(232.41 63.4563 -3.8434 67.1743 -17.1686 95.4859 -95.1324 251.873 -36.1094 28.2229 -93.097 -107.664 379.134 -5.72958 91.7335 3.48553 74.2702)
36.E8-irteusgl$ (send *ri* :state :potentio-vector)
;; #<rotational-joint #X55a52b17e400 head_tilt_joint> :joint-angle(75.57) violate max-angle(74.2702)
;; #<rotational-joint #X55a52b25ba20 r_wrist_flex_joint> :joint-angle(-4.99275) violate max-angle(-5.72958)
#f(232.41 63.4563 -3.8434 67.1743 -17.1686 95.4826 -95.1324 251.873 -36.0762 26.7834 -79.1134 -106.885 386.621 -5.72958 91.7235 3.47953 74.2702)
37.E8-irteusgl$ (send *ri* :state :potentio-vector)
;; #<rotational-joint #X55a52b17e400 head_tilt_joint> :joint-angle(75.57) violate max-angle(74.2702)
;; #<rotational-joint #X55a52b25ba20 r_wrist_flex_joint> :joint-angle(-5.13983) violate max-angle(-5.72958)
#f(232.409 63.4515 -3.8434 67.1743 -17.1686 95.4859 -95.1324 251.873 -36.0287 20.0704 -75.42 -95.7531 397.257 -5.72958 91.8606 3.52153 74.2702)

|#

(setq r1 #f(232.41 63.4563 -3.8434 67.1743 -17.1686 95.4859 -95.1324 251.873 -36.1094 28.2229 -93.097 -107.664 379.134 -5.72958 91.7335 3.48553 74.2702))
(setq r2 #f(232.41 63.4563 -3.8434 67.1743 -17.1686 95.4826 -95.1324 251.873 -36.0762 26.7834 -79.1134 -106.885 386.621 -5.72958 91.7235 3.47953 74.2702))
(setq r3 #f(232.409 63.4515 -3.8434 67.1743 -17.1686 95.4859 -95.1324 251.873 -36.0287 20.0704 -75.42 -95.7531 397.257 -5.72958 91.8606 3.52153 74.2702))



(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l") ;;*ri*
(require "package://pr2eus/speak.l") ;;pkg for speaking

(setq r1 #f(232.41 63.4563 -3.8434 67.1743 -17.1686 95.4859 -95.1324 251.873 -36.1094 28.2229 -93.097 -107.664 379.134 -5.72958 91.7335 3.48553 74.2702))
(setq r2 #f(232.41 63.4563 -3.8434 67.1743 -17.1686 95.4826 -95.1324 251.873 -36.0762 26.7834 -79.1134 -106.885 386.621 -5.72958 91.7235 3.47953 74.2702))
(setq r3 #f(232.409 63.4515 -3.8434 67.1743 -17.1686 95.4859 -95.1324 251.873 -36.0287 20.0704 -75.42 -95.7531 397.257 -5.72958 91.8606 3.52153 74.2702))


(if (not (boundp '*pr2*)) (pr2-init))

(objects (list *pr2*))

;;(send *ri* :angle-vector r1 1000)
;;(send *ri* :wait-interpolation)
;;(send *irtviewer* :draw-objects)

;;(send *ri* :angle-vector r2 50)
;;(send *ri* :wait-interpolation)
;;(send *irtviewer* :draw-objects)

;;(send *ri* :angle-vector r3 5
(setq tm 300)
(send *ri* :angle-vector-sequence (list r1 r2 r3 r1) (list tm tm tm 1000)
      :default-controller 0.001
      :min-time 0.0001
      :minjerk-interpolation t
      )
(send *ri* :wait-interpolation)
(send *irtviewer* :draw-objects)
