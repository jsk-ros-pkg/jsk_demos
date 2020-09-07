(defun grasp(*robot* *obj* arm)
    (send *robot* :head :look-at (send *obj* :worldpos))
    ;; dissoc
    (if (send *obj* :parent) (send (send *obj* :parent) :dissoc *obj*))
    ;; solve ik
    (send *robot* arm :inverse-kinematics
                (car (send *obj* :handle))
                :link-list (send *robot* :link-list (send *robot* arm :end-coords :parent))
                :move-target (send *robot* arm :end-coords)
                :rotation-axis :z
                :debug-view t)
    ;; assoc *obj* and arm
    (send *robot* arm :end-coords :assoc *obj*)
)