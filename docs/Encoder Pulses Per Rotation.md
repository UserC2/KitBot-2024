## Background
* Encoders are sensors used to track the movement of a mechanism (like an arm or a drivetrain) by measuring rotation.
* Quadrature encoders (e.g. CTRE's SRX Mag Encoder) emit a pulse when they rotate a small amount (the direction of the pulse is determined by the direction the encoder is spun).
* WPILib's `Encoder` class counts these pulses automatically, so we can determine how much the encoder has rotated. By default `Encoder.getDistance()` returns how much the encoder has rotated in pulses.
* However, knowing the amount an encoder has rotated in "pulses" is not typically useful, so it has to be converted into a unit with a useful meaning (for example, the amount of metres a wheel has travelled, or the angle of an arm in degrees).
* To have `Encoder` convert the pulses into a useful unit for us, we can use `Encoder.setDistancePerPulse()` with a conversion factor.

## Calculating the Conversion Factor for the Kitbot Drivetrain
**This is wrong, needs to be fixed**

We have:
* Some number of pulses, output by the encoder.

We would like:
* How many meters we have travelled.

This means we need to find a number we can multiply by the number of pulses to get the amount of distance travelled (a unit conversion).

We start with encoder rotations:

For each encoder rotation, 4096 pulses are emitted (for CTRE's SRX Mag Encoder):
* `encoder rotations = encoder pulses / 4096`

The encoder and drive motors are connected to the wheels through a gearbox, so we also have to account for this.
* The AndyMark ToughBox Mini on the 2024 KoP drivetrain has a 8.45:1 (input : output) gear ratio, meaning that for every rotation of the wheel (the output), the encoder (or motor, the input) rotates 8.45 times. This gives us another number:
* `1 / 8.45`

If we multiply both these numbers, we get `0.00002889238166` (the amount of encoder rotations per wheel rotations).

However, this only gives us how many rotations the wheel has made. In order to find the distance travelled, we can multiply the amount of rotations the wheel made by its circumference (the perimeter of the wheel surface).
* Circumference can be calculated using `Pi * Wheel Diameter`
* The wheels on the kit chassis are 6 inches. We would like our distance in meters, so we need to convert that measurement:
* `6 inches = 0.15 meters`
* So, the wheel's circumference is:
* `Pi * 0.15`

Multiplying all of these numbers gives the following result:
```
circumference * encoder rotations per wheel rotation * encoder pulses per encoder rotation
= (Pi * 0.15) * (1 / 8.45) * (1 / 4096)
= 0.00001361521409 meters per encoder pulse
```

In Java, this looks like:
```java
encoder.setDistancePerRotation(
  (Math.PI * 0.15) * (1 / 8.45) * (1 / 4096)
);
```

## In Summary
"Distance per rotation" is a number multiplied by the encoder output to calculate how far one side of the robot has driven.

To calculate this factor, you need to take into account:
* Encoder pulses per encoder rotation
* Wheel rotations per encoder rotation (the gear ratio between the encoder and the wheels)
* Wheel circumference (distance per wheel rotation)

These values are usually
* 4096 pulses / rotation (CTRE SRX Mag Encoder)
* 8.45 wheel rotations / encoder rotation (**2024** kit chassis)
* Pi * 0.15 meters (wheel circumference; 0.15 meters = 6 inches)

The final equation looks like this:
```
(Math.PI * 0.15) * (1 / 8.45) * (1 / 4096)
```

And the code looks like this:
```java
encoder.setDistancePerRotation(
  (Math.PI * 0.15) * (1 / 8.45) * (1 / 4096)
);
```