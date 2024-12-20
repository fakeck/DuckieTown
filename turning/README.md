# :twisted_rightwards_arrows: Turning at Crossings (turning)
The node `turn_service` provides a ROS service for turning left/right/U-turn/stop at crossings. The turning will be requested by the state machine and will return when the turn is finished.
TODO: demo video here

The implemetation is an open-loop time control. We send constant tuned velocity commands to the wheels for a tuned period of time. This is not a robust implementation but gives us a relatively consistant performance.

Future work would be to utilize IMU and odometry to calculate angular displacement, and potentially use a PID controller to adjust the wheel speed accordingly. However, from our inspections, the gyroscope's axis roughly aligns with the robot frame when the speed is slow, but its axis changed confusingly when we are rotating faster. This could be due to gyroscopic precession, axis misalignment, or measurement noise. We would need to first figure out this, before proceeding with the algorithm. The odometry is more reliable in this sense, so a simple weighted fusion of gyroscope and odometry could be a solution. Visual feedback of the stopping lines serve as a third input choice.

For the feedback controller, we would need better designed wheel motors. We have tried that when changing the velocity sent to the wheels - even if the ratio between the linear and angular velocity is kept constant - its turning radius changes.