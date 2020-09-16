(defun grasp(*robot* arm *obj* handle)
    (send *robot* :head :look-at (send *obj* :worldpos))
    ;; dissoc
    (if (send *obj* :parent) (send (send *obj* :parent) :dissoc *obj*))
    ;; solve ik
    (send *robot* arm :inverse-kinematics
                (car (send *obj* handle))
                :link-list (send *robot* :link-list (send *robot* arm :end-coords :parent))
                :move-target (send *robot* arm :end-coords)
                :rotation-axis :z
                :debug-view t)
    ;; assoc *obj* and arm
    (send *robot* arm :end-coords :assoc *obj*)
)

(defun release(*obj*)
    (if (send *obj* :parent) (send (send *obj* :parent) :dissoc *obj*))
)

(defun wipe-with-rarm(*robot*)
    ;; expected to use reset-manip-pose at first
    (send *robot* :rarm :move-end-pos #f(300 -300 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(50 100 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(50 100 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(10 50 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(10 50 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(10 50 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(0 50 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(0 50 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(0 50 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(-100 20 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(-100 20 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(-100 0 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(-50 0 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(-50 50 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(-50 50 0) (send *robot* :copy-worldcoords) :debug-view t)
    (send *robot* :rarm :move-end-pos #f(-50 0 0) (send *robot* :copy-worldcoords) :debug-view t)
)