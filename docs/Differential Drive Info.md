## Basics
* Differential drive is a drivetrain with a set of motors on the left and right side of the robot.
* Each set of motors is controlled individually.

## Teleop
The drivetrain can be controlled in several different ways:
* Tank Drive: One joystick controls the left motors, and one joystick controls the right motors.
* Arcade Drive: One joystick controls the forward/reverse speed, and one controls the turning speed.
* Curvature Drive: Same as arcade drive, but turning speed slows down the faster the robot is moving. Spinning in place is only allowed when a button is held.

To implement these controls, WPILib provides the `DifferentialDrive` class.
* `DifferentialDrive.arcadeDrive()`
* `DifferentialDrive.curvatureDrive()`
* `DifferentialDrive.tankDrive()`

## Autonomous
(TODO)