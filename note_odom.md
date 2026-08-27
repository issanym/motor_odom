To publish odometry from your encoder ticks, your node needs to convert the raw pulse count into linear distance and angular rotation, calculate the robot's change in position over time (dX, dY, dθ), and broadcast both an nav_msgs/msg/Odometry message and a tf2_ros transform.
The calculations below assume a differential drive robot (two wheels).

Step 1: The Core Math Formulas
To do this, your code needs several physical constants about your robot:
- Ticks Per Revolution (N): Total encoder edges per full wheel turn.
- Wheel Radius (R) or Diameter (D): In meters.
- Wheel Track (L): Distance between the center of the two wheels in meters.

1. Distance per individual wheel 
    $$(\text{Distance}=\frac{\text{Delta\ Ticks}}{\text{Ticks\ Per\ Revolution}}\times (2\pi \times R)\)$$
2. Robot Displacement (Centerpoint)Linear Distance (dS): 
    $$\(\frac{\text{Distance}_{\text{left}}+\text{Distance}_{\text{right}}}{2}\)Angular Change (dθ): \(\frac{\text{Distance}_{\text{right}}-\text{Distance}_{\text{left}}}{L}\)$$
3. Update Global Position (X, Y, θ)Because the robot updates iteratively over short time steps (dt), you integrate the changes into global coordinates: 
$$(X_{\text{new}}=X_{\text{old}}+dS\times \cos (\theta +\frac{d\theta }{2})\)$$ 
$$\(Y_{\text{new}}=Y_{\text{old}}+dS\times \sin (\theta +\frac{d\theta }{2})\)$$
$$\(\theta _{\text{new}}=\theta _{\text{old}}+d\theta \)$$

Step 2: ROS 2 Odometry Node Implementation
Here is a clean Python implementation for ROS 2. It tracks left and right ticks, runs the equations, converts the Euler angle θ into a quaternion, and publishes both the Odometry topic and TF Transform.

```
code
```

Troubleshooting Odom Calibration
If you run this code and find the robot's tracking isn't quite accurate when testing, check for these issues:
- Robot spins in a circle but data shows straight line: One of your encoder channels is reversed (counting downwards instead of upwards). Flip the sign of dist_left or dist_right in the script to fix it.
- Robot drifts when turning: Your WHEEL_TRACK (width parameter) is slightly off in the code. If it rotates too much on screen, increase the width parameter. If it doesn't rotate enough on screen, decrease it.