[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyUSB0 | 2000000   | gripper

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | RH-P12-RN      | 2.0      | gripper        | present_position, torque_enable, goal_position, goal_velocity, goal_current, goal_acceleration, is_moving, present_velocity, present_current
