(load "valve.l")
(load "door.l")

;; valve
(test-valve)
(unless (boundp '*col*)
  (setq *col* (instance collision-link-parameter :init *robot*))
  (send (send *man* :model :collision) :reset-manip-pose)
  (send *col* :init-distance-table))
(setq *ik-solver* (instance ik-solver
                            :init *robot*
                            :rarm
                            ;;:ik-type :use-torso
                            :ik-type :fullbody-with-standcoords
                            :move-target (send *robot* :rarm :end-coords)
                            :trajectory *traj*
                            :collision-parameter *col*
                            :prepare-angle-vector (send *robot* :reset-manip-pose)))
;;(setq *ik-profile* (send *ik-solver* :profile :test-num 10 :max-trajectory 30 :max-collision 40))
(setq *ik-profile* (send *ik-solver* :profile :test-num 5 :max-trajectory 30 :max-collision 20))
;;(setq *ik-profile* (send *ik-solver* :profile :test-num 10 :max-trajectory 30 :max-collision 40))
(send *ik-profile* :write "jaxon_valve_ik_stand2.csv")
(exit)
