(load "door.l")

;; valve
(test-door :fullbody-with-standcoords)
(unless (boundp '*col*)
  (setq *col* (instance collision-link-parameter :init *robot*))
  (send (send *man* :model :collision) :reset-manip-pose)
  (send *col* :init-distance-table))
(setq *ik-profile* (send *ik-solver* :profile :test-num 5 :max-trajectory 30 :max-collision 20))
(send *ik-profile* :write "jaxon_door_ik_stand2.csv")
(exit)
