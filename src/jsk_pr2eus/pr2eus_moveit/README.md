pr2eus_moveit
=============

This package contains euslisp interface file for using moveit

## Setting Up

Check if  pr2 robot with moveit is already running, if not, run `gazebo` and `moveit` by
```
$ roslaunch pr2eus_moveit test-pr2eus-moveit.test  gui:=true
```

Run `roseus`, load required packages and setup `robot-interface` with moveit feature enabled.

```
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
(pr2-init)
(send *ri* :set-moveit-environment (instance pr2-moveit-environment :init))
```

## Send collision-free angle-vector

To send collision-free angle-vector, run following command instead of normal `:angle-vector`

```
(send *ri* :angle-vector-motion-plan (send *pr2* :reset-pose) :move-arm :rarm :use-torso t)
```

## Plan motions and execute

If you want to specify target enc-coords instead of angle-vector, use following command

```
(send *ri* :move-end-coords-plan (make-coords :pos #f(700 0 750)) :move-arm :larm :use-torso t)
```

## Use MoveIt IK service

If you want to use IK service provided by MoveIt, try

```
(setq av (send *ri* :collision-aware-ik (make-coords :pos #f(700 0 850)) :move-arm :rarm :use-torso nil))
(send *pr2* :angle-vector av)

```