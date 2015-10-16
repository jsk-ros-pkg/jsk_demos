(defun create-robot-instance
  (&key (real? nil) 
        (robot-pos #f(0 0 0))
        (robot-dir 0)
        (robot "samplerobot")
        (draw? nil)
        )
  (create-robot-instance-core *robot* *init-robot-coords* 
                              :real? real?
                              :robot-pos robot-pos
                              :robot-dir robot-dir
                              :robot robot
                              :draw? draw?)
  )

(defmacro create-robot-instance-core
  (*robot* *init-robot-coords*
   &key (real? nil) 
        (robot-pos #f(0 0 0))
        (robot-dir 0)
        (robot "samplerobot")
        (draw? nil)
        )
  (list 'progn
        (list 'create-robot-and-ri *robot* ':real? real? ':robot robot)
        (list 'send *robot* ':reset-manip-pose)
        (list 'send *robot* ':fix-leg-to-coords '(make-coords))
        (list 'send *robot* ':newcoords (list 'send (list 'send (list 'send (list 'send *robot* ':worldcoords) 
                                                                      ':copy-worldcoords) ':translate robot-pos :world)
                                              ':rotate (list 'deg2rad robot-dir) ':z :world))
        (list 'when draw? (list 'objects (list 'list *robot*)))
        (list 'setq *init-robot-coords* (list 'send (list 'send *robot* ':worldcoords) ':copy-worldcoords))
        '(print "robot's worldcoords is in *init-robot-coords*.")
        ))

(defmacro create-robot-and-ri
  (*myrobot* 
              &key (real? nil) 
              (robot "samplerobot")
              )
  (list 'progn
        (list 'setq 'real? real?)
        (list 'cond
              (list (list 'equal robot '"samplerobot")
                    '(require "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
                    '(print "load package. OK.")
                    (list 'if '(eq real? nil)
                          (list 'setq *myrobot* '(instance samplerobot-robot :init))
                          (list 'progn '(samplerobot-init) (list 'setq *myrobot* '*sr*))
                          )
                    '(print "create *robot* (and *ri* if you set :real t)")
                    )
              (list (list 'equal robot '"hrp2jsknt")
                    '(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
                    '(print "load package. OK.")
                    (list 'if '(eq real? nil)
                          (list 'setq *myrobot* '(instance hrp2jsknt-robot :init))
                          (list 'progn '(hrp2jsknt-init) (list 'setq *myrobot* '*hrp2jsknt*))
                          )
                    '(print "create *robot* (and *ri* if you set :real t)")
                    )
              (list (list 'equal robot '"hrp2jsk")
                    '(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
                    '(print "load package. OK.")
                    (list 'if '(eq real? nil)
                          (list 'setq *myrobot* '(instance hrp2jsk-robot :init))
                          (list 'progn '(hrp2jsk-init) (list 'setq *myrobot* '*hrp2jsk*))
                          )
                    '(print "create *robot* (and *ri* if you set :real t)")
                    )
              (list (list 'equal robot '"jaxon_red")
                    '(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
                    '(print "load package. OK.")
                    (list 'if '(eq real? nil)
                          (list 'setq *myrobot* '(instance jaxon_red-robot :init))
                          (list 'progn '(jaxon_red-init) (list 'setq *myrobot* '*jaxon_red*))
                          )
                    '(print "create *robot* (and *ri* if you set :real t)")
                    )
              )
        ))


















