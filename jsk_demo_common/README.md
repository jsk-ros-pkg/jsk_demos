jsk_demo_common
===

This package includes common action functions for daily assistant robot
and task execution manager which is used for planning and execution with attention observation.

## directories

euslisp/: source codes
sample/: sample codes

## samples

### attention-observation-sample.l

```bash
# terminal 0
$ roscore
```

```bash
# terminal 1
$ rosrun roseus roseus
irteusgl$ (load "package://jsk_demo_common/sample/attention-observation-sample.l)
irteusgl$ (test)
```

while executing actions:

```bash
# terminal 2
$ rosrun roseus roseus
irteusgl$ (load "package://jsk_demo_common/euslisp/attention-observation.l")
irteusgl$ (pr2-init)
irteusgl$ (send *ri* :cancel-all-motion)
```

then robot action is interrupted.

### preemptive-task-execution-sample.l

```bash
$ rosrun roseus roseus
irteusgl$ (load "package://jsk_demo_common/sample/preemptive-task-execution-sample.l")
```

you can see generated state machine by executing follow command before executing sample above:

```bash
$ rosrun smach_viewer smach_viewer.py
```

![place](https://gist.githubusercontent.com/furushchev/37b003f33fd604c8f88a/raw/db7112b145198bde8c8d3e014ee51cb746e492da/pr2_interface.png =100x100)
![smach](https://gist.githubusercontent.com/furushchev/37b003f33fd604c8f88a/raw/7443a00327f220933f6c4fcf0b607d4be728e71a/smach_viewer.png =400x400)
