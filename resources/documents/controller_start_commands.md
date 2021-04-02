# Controller start commands

Here you find some ROS commands which can be used to start the controllers. These commands use the `controller_manager/switch_controllers` service.

## PANDA ARM

### Start Panda arm position controllers stop panda_arm_controller

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_arm_joint1_position_controller', 'panda_arm_joint2_position_controller', 'panda_arm_joint3_position_controller', 'panda_arm_joint4_position_controller',
'panda_arm_joint5_position_controller', 'panda_arm_joint6_position_controller', 'panda_arm_joint7_position_controller']
stop_controllers: ['panda_arm_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

### Start panda_arm_controller and stop Panda arm position controllers

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_arm_controller']
stop_controllers: ['panda_arm_joint1_position_controller', 'panda_arm_joint2_position_controller', 'panda_arm_joint3_position_controller', 'panda_arm_joint4_position_controller',
'panda_arm_joint5_position_controller', 'panda_arm_joint6_position_controller', 'panda_arm_joint7_position_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

### Start Panda arm joint effort controllers and stop panda_arm_controller

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_arm_joint1_effort_controller', 'panda_arm_joint2_effort_controller', 'panda_arm_joint3_effort_controller', 'panda_arm_joint4_effort_controller',
'panda_arm_joint5_effort_controller', 'panda_arm_joint6_effort_controller', 'panda_arm_joint7_effort_controller']
stop_controllers: ['panda_arm_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

### Start Panda arm position group controller

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_arm_joint_group_position_controller']
stop_controllers: ['panda_arm_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

### Start Panda arm effort group controller

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_arm_joint_group_effort_controller']
stop_controllers: ['panda_arm_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

## PANDA HAND

### Start Panda hand position controllers

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_hand_finger1_position_controller', 'panda_hand_finger2_position_controller']
stop_controllers: ['panda_hand_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

### Start Panda hand effort controllers

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_hand_finger1_effort_controller', 'panda_hand_finger2_effort_controller']
stop_controllers: ['panda_hand_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

### Start Panda hand position group controller

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_hand_joint_group_position_controller']
stop_controllers: ['panda_hand_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

### Start Panda hand effort group controller

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['panda_hand_joint_group_effort_controller']
stop_controllers: ['panda_hand_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

## Other commands

-   List controllers: `rosrun controller_manager controller_manager list`
-   Controller gui: `rosrun rqt_controller_manager rqt_controller_manager`
