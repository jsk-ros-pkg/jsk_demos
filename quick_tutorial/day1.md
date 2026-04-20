## Day1: playing geometric robot model in Euslisp
Before startgin this tutorial, please do `roscore` in other terminal. 

### get information of joint angles of a robot

```lisp
(load "package://fetcheus/fetch-interface.l") 
(fetch) 
```
The `fetch` function creates an instance named `*fetch*` which is a geometrical model of fetch-robot. Now you can view the model by calling the following function:
```lisp
(objects *fetch*)
```
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/quick_tutorial/master/images/day1_1.png" alt="none" title="day1_1" width="200">
</div>

The robot model `*fetch*` contains the information of joints. The fetch robot has 10 joints, so let's look at the state of these joints.
```lisp
(send *fetch* :angle-vector)
;; output: #f(20.0 75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0 0.0 0.0)
```
As you can see, the state of 10 joints is shown as a float-vector. Probably you need to know which value corresponds to which joints. To this end, the following method is useful.
```lisp
(send *fetch* :joint-list :name)
;; output: ("torso_lift_joint" "shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint" "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint" "head_pan_joint" "head_tilt_joint")
```
You get the list of 10 strings of the joint name. The order of this list corresponds to the order of the float-vector you got by `:angle-vector` method. By comparing those two, for example, you can know that angle of `torso_lift_joint` is `20.0`.

Now, let's set a custom angle vector to the robot model. 
```lisp
(setq *av-zero* (float-vector 0 0 0 0 0 0 0 0 0 0))
(send *fetch* :angle-vector *av-zero*)
```
Please click the previously opened IRT-viewer, then you will see the robot model is updated (IRT-viewer is not updated without click!). 
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/quick_tutorial/master/images/day1_2.png" alt="none" title="day1_2" width="200">
</div>

Maybe, you want to set a specific joint instead of setting all the joints angles at once. For the `shoulder_pan_joint` case, for example, this can be done by:
```lisp
(let ((shoulder-pan-joint (send *fetch* :shoulder_pan_joint)))
    (send shoulder-pan-joint :joint-angle 60))
```
(also please check [this note](#access-to-joint-in-jsk-style))

### solving inverse kinematics (IK)
Usually, in robotics, you want to guide the robot arm's end effector to a commanded pose (position and orientation). Thus, before sending an angle vector, you must know an angle vector with which the end effector will be the commanded pose. This can be done by solving inverse kinematics (IK) (if you are not familiar please google it). First, we create a coordinate (or a pose) `*co-target*` by
```lisp
(setq *co-target* (make-coords :pos #f(800 300 800) :rpy #f(0.0 0.1 0.1))) ;; #f(..) is a float-vector
```
Then the following code will solver the IK:
```lisp
(send *fetch* :angle-vector #f(0 0 0 0 0 0 0 0 0 0)) ;; initial solution
(send *fetch* :rarm :inverse-kinematics *co-target* ;; fetch robot has only :rarm. PR2 had both :larm and :rarm
        :rotation-axis t :check-collision t :use-torso nil)
```
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/quick_tutorial/master/images/day1_4.png" alt="none" title="day1_4" width="200">
</div>

In `:inverse-kinematics`, IK is solved and the obtained angle vector is applied to ` *fetch* `. Note that you must care initial solution for IK. In the Euslisp, the angle-vector set to the robot before solving IK is the initial solution of IK. For example, `#f((0 0 0 0 0 0 0 0 0 0)` is the initial solution. In solving IK you can set some key arguments. `:rotation-axis`, `check-collision` and `use-torso` is particularly important. If `:rotation-axis` is `nil` the IK is solved ignoring orientation (rpy). If `:check-collision` is `nil` the collision between the links of the robot is not considered. Please play with changing these arguments. 

<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/quick_tutorial/master/images/day1_6.png" alt="none" title="day1_6" width="200">
</div>
You can see the two coordinates (diplayed by white arrows) are equal to each other.

Note that a solution of IK will be changed if initial solution is changed. (please try different initial solution and solve IK). At the worst case, IK cannot be solved (usually happens). Intuitively speaking, if the target end-effector pose is far from the initial pose, solving IK becomes difficult and likely to be failed. To get over this problem, it is effective to prepare "mid pose" in the middle of the current and targeted end effector pose. Then solve IK for "mid pose" first, and by using obtained angle-vector for "mid-pose" as an initial solution, solve IK for target pose. (TODO: need editing)

### Visualization 
It is quite helpful if geometric relation between current and target coordinate of end effector. You can get the current coordinate of the end effector by 
```
(setq *co-endeffector* (send *fetch* :rarm :end-coords))
```
By using this it is possible to make arrow object:
```
(require "models/arrow-object.l") ;; you need to load this 
(setq *co-endeffector-vis* (arrow))
(send *co-endeffector-vis* :newcoords (send *co-endeffector* :copy-worldcoords))
```
In the same manner let's make arrow object of target coordinate
```
(setq *co-target-vis* (arrow))
(send *co-target-vis* :newcoords (make-coords :pos #f(800 300 800) :rpy #f(0.0 0.1 0.1)))
```
And visualize all by
```
(objects (list *fetch* *co-endeffector-vis* *co-target-vis*))
```
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/quick_tutorial/master/images/vis.png" alt="none" title="vis" width="200">
</div>


### Other important things
#### float vector 

A pitfall for float vector is that, if you call function inside `#f( )` (e.g. `#f(0 0 (deg2rad 30))`) you will get error:
```lisp
error: number expected in read-float-array
```
To avoid this issue, you can simply call `(float-vector 0 0 (deg2rad 30))` instead.

#### access to joint in jsk style 

In the above tutorials, to access a joint object I used like `(send *fetch* :shoulder_pan_joint)`. The another equivalent code to do this is 
```
(send *fetch* :rarm :shoulder-y) ;; y indcates rotation around `yaw` 
```
The mapping from the jsk-style notation to official notation is given in [fetch.yaml](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_fetch_robot/fetcheus/fetch.yaml). Similarly, to access `head_pan_joint`, you can use following two equivalent ways:
```
(send *fetch* :head_pan_joint)
(send *fetch* :head :neck-y);; jsk style
```

### TODO (need editing)
1. add `draw-on`. (will be edited after some response in  https://github.com/euslisp/EusLisp/issues/426)
2. `:methods`
3. `apropos`
