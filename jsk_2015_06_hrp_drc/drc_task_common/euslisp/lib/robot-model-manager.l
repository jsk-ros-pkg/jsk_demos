(defclass robot-model-manager
  :super propertied-object
  :slots (robot-name models ri))

(defmethod robot-model-manager
  (:init (arobot-name)                  ;:hrp2 or :jaxon
    (setq robot-name arobot-name)
    (setq models nil)
    self)
  (:generate-model ()
    (cond ((eq robot-name :hrp2)
           (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
           (instance hrp2jsknt-robot :init))
          ((eq robot-name :jaxon)
           (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
           (instance jaxon_red-robot :init))
          (t
           (error "unknown robot model: ~A" robot-name))))
  (:robot-interface ()
    (unless ri
      (cond ((eq robot-name :hrp2)
             (setq ri (hrp2jsknt-init))
             ri)
            ((eq robot-name :jaxon)
             (setq ri (jaxon_red-init))
             ri)))
    ri)
  (:add (name &optional analysis-level)
    (unless (assoc name models)
      (let ((new-model (send self :generate-model)))
        (if analysis-level
            (send-all (send new-model :links) :set-val 'analysis-level :coords))
        (setq models (append models (list (cons name new-model)))))))
  (:get-model (name)
    (cdr (assoc name models)))
  (:model (name &optional analysis-level)
    (send self :add name analysis-level)
    (send self :get-model name))
  )

