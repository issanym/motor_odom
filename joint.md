1. In your node's __init__ constructorAdd tracking variables for your absolute cumulative radians and create the joint_states publisher:

```
# Create the publisher
self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

# Initialize variables to keep track of total wheel positions in radians
self.left_wheel_rad = 0.0
self.right_wheel_rad = 0.0

# Calculate standard conversion factors from your existing METERS_PER_TICK
# (Distance / wheel_radius = angle in radians)
self.WHEEL_RADIUS = self.WHEEL_TRACK / 2.0  # Or set your explicit wheel radius here
self.RAD_PER_TICK = self.METERS_PER_TICK / self.WHEEL_RADIUS

```

2. Add this at the bottom of your update_odometry method
Insert this block right after you publish your odom message:

```
        # 7. Calculate cumulative joint positions and velocities
        self.left_wheel_rad += delta_left * self.RAD_PER_TICK
        self.right_wheel_rad += delta_right * self.RAD_PER_TICK

        v_left_rad_s = dist_left / (self.WHEEL_RADIUS * dt)
        v_right_rad_s = dist_right / (self.WHEEL_RADIUS * dt)

        # 8. Publish the JointState message
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        
        # Ensure these names perfectly match the joint tags in your URDF
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_msg.position = [self.left_wheel_rad, self.right_wheel_rad]
        joint_msg.velocity = [v_left_rad_s, v_right_rad_s]

        self.joint_pub.publish(joint_msg)

        self.last_time = current_time
```

Why doing this matters
Now, when you move your robot, robot_state_publisher will immediately notice that left_wheel_joint and right_wheel_joint are rotating. It will calculate the internal TF frames (base_link → left_wheel and base_link → right_wheel) dynamically based on your URDF geometry. Together with your odom → base_link transform, your entire TF tree will link up perfectly.