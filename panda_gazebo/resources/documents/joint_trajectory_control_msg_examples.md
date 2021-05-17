# Example commands for joint trajectory control

Here you find an example of valid joint trajectory command that can be send to the joint
trajectory action server.

## EXAMPLE POSE 1 (Positions)

```txt
trajectory:
 header:
   seq: 19357
   stamp:
     secs: 0
     nsecs:         0
   frame_id: ''
 joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]
 points:
   -
     positions: [0.0, 0.02, -0.0, -0.07, 0.0, -0.02, 0.91]
     velocities: []
     accelerations: []
     effort: []
     time_from_start:
       secs: 1
       nsecs:         0
```

## EXAMPLE POSE 2 (Positions)

```txt
trajectory:
 header:
   seq: 19357
   stamp:
     secs: 0
     nsecs:         0
   frame_id: ''
 joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]
 points:
   -
     positions: [1.7383799999999998, 0.8461439999999998, -1.796326, -0.07, -1.274812, -0.02, 0.91]
     velocities: []
     accelerations: []
     effort: []
     time_from_start:
       secs: 1
       nsecs:         0
```

## EXAMPLE POSE 3 (Positions)

```txt
trajectory:
 header:
   seq: 19357
   stamp:
     secs: 0
     nsecs:         0
   frame_id: ''
 joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]
 points:
   -
     positions: [1.7383799999999998, 0.8461439999999998, 0.0, -0.07, -1.274812, -0.02, 0.91]
     velocities: []
     accelerations: []
     effort: []
     time_from_start:
       secs: 1
       nsecs:         0
```

## How to use

You can use the `actionlib axclient.py` to send the commands:

```bash
rosrun actionlib axclient.py /panda_arm_controller/follow_joint_trajectory
```
