## Day2: workflow of real robot (simulated) control

### Preparation
Before starting this tutorial, in another terminal, please run
```lisp
roslaunch quick_tutorial common.launch
```

### Robot control workflow
As a preparation load some libraries and call `fetch-init` function.
```lisp
(load "package://fetcheus/fetch-interface.l")
(require "models/arrow-object.l")
(fetch-init)
```
Different from `fetch` function that we called in the [previous tutorial](https://raw.githubusercontent.com/HiroIshida/quick_tutorial/master/day2.md), `fetch-init` create robot interface object `*ri*` as well as `*fetch*`. Through *ri*, we can control a real (or simulated) robot and obtain a state of the real robot. Note that joint angles of `*fetch*` are set such that it's the same as the real one. Please call `(objects *fetch*)` and compare the posture of `*fetch*` with the real robot.     

Suppose you want to grasp the handle of a kettle (see gazebo), and *somehow* you know the coordinate of the handle `*co-handle*`(I said *somehow*, but you will get an idea how to do this is next tutorial(TODO))
. You probably want to guide the end-effector to `*co-handle*`. 

```lisp
(setq *co-handle* (arrow))
(send *co-handle* :newcoords (make-coords :pos #f(800 300 1000) :rpy #f(0.0 0.0 1.54)))
```

Now let's visualize the `*fetch*`, `*co-handle*` and coordinate of the end-effector all together:
```lisp
(setq *co-endeffector* (arrow))
(send *co-endeffector* :newcoords (send (send *fetch* :rarm :end-coords) :copy-worldcoords))
(objects (list *fetch* *co-endeffector* *co-handle*))
```

Rather than directory guide the end-effector to `*co-handle*`, let's guide to a coordinate `*co-ik-target*` which is slightly behind the target:
```lisp
(setq *co-ik-target* (arrow))
(send *co-ik-target* :newcoords (send *co-handle* :copy-worldcoords))
(send *co-ik-target* :translate #f(-80 0 0) :local)
```
Because we set `:local`, the translation is done w.r.t. `*co-ik-target*` itself. Note that you can also set `:world`. We don't do it here but you can also rotate the coordinate around a specified axis (x, y, z) by a specified angle. For example, rotation around y-axis by 0.1 rad is `(send *co* :rotate 0.1 :y :local)`.

By calling `(objects (list *fetch* *co-endeffector* *co-handle* *co-ik-target*))` you will see the following figure.
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/quick_tutorial/master/images/day2_1.png" alt="none" title="day2_1" width="300">
</div>
 

Now we solve the IK,
```lisp
(send *fetch* :rarm :inverse-kinematics *co-ik-target*
        :rotation-axis t :check-collision t :use-torso t)
```
and send it to the real robot!
```lisp
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000) ;; command 1
(send *ri* :wait-interpolation)
```
Here, by specifying `1000`, it takes 1000 msec for the real robot to follow the commanded angle-vecttor. Note that you cannot specify smaller value than 1000. 

Then move the end-effector forward by
```lisp
(send *fetch* :rarm :move-end-pos #f(100 0 0) :local)
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000) ;; command 2
(send *ri* :wait-interpolation)
```
In the `:move-end-pos` method, IK is solved. Similar to `:translate` method of coordinate you, can also specify `:world` instead of `:local`. Also, rotation of end-effector is possible by `(send *fetch* :rarm :move-end-rot 20 :x :local)`. Note that, different from `:rot` method, unit of rotation is [deg] for `:move-end-rot`. Here, `:wait-interpolation` is important. Without calling this, commanded angle-vector is overwritten. For example, if you send commands 1 and 2 above without `:wait-interpolation` it is almost the same as sending only `command2`. (explain why I said *almost* :TODO).

Grasp the handle with effort of 100:
```lisp
(send *ri* :start-grasp :effort 100)
```
Lift it:
```lisp
(send *fetch* :rarm :move-end-pos #f(0 0 100) :world)
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
(send *ri* :wait-interpolation)
```
As you probably noticed, the basic workflow is a repetition of 1) solving IK in the Euslisp side (and check the geometry) and then 2) command it to the real robot by `:angle-vector` method. 

The tutorial above is summarized in `day2.l`. If you run it, you can observe robot motion as the following animation.
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/quick_tutorial/master/images/day2_whole.gif" alt="none" title="day2_1" width="400">
</div>

### Other important things
1 you can obtain real (simulated) robot' angle-vector by `(send *ri* :state :potentio-vector)`. 
2 Suppose you want to send two angle-vector's, say `*av1*` and `*av2*`. If you use `:wait-interpolation`, the robot will completely stop when the robot achieves `*av1*`, and then start following `*av2*`. This is too slow. If it's annoying, you can use `unix:usleep` function. For example:
```lisp
(send *ri* :angle-vector `*av1*` 1000)
(unix:usleep 500000) ;; unit is micro-second
(send *ri* :angle-vector `*av2*` 1000)
```
By this, although it takes 1000 ms to follow `*av1*`, the goal is overwritten by `*av2*` just after `500000`. (TODO: add some illustration)
