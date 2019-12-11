(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(load "package://fetcheus/fetch-interface.l")
(load "models/foldable-desk-object.l")
;;unless=when not
(unless (boundp '*ri*) (fetch-init))
(objects (list *fetch*))


(ros::roseus-add-msgs "std_msgs")
(setq message_arrived nil)

(defun spot-cb (msg)
  (setq message_arrived t)
  (setq listen-data (send msg :data)))

(ros::subscribe "/atohayoroshiku" std_msgs::Int16 #'spot-cb)

(ros::rate 10)
(while (ros::ok)
  (when message_arrived
    (cond ((= listen-data 1) (send *ri* :speak-jp "プレゼントはお菓子、なのだ！"))
	  ((= listen-data 2) (send *ri* :speak-jp "プレゼントはジーピーユー、なのだ！"))
	  ((= listen-data 3) (send *ri* :speak-jp "プレゼントはぬいぐるみ、なのだ！"))
	  (t (send *ri* :speak-jp "プレゼントがわかりません。"))))
  (setq message_arrived nil)
  (ros::sleep)
  (ros::spin-once))
