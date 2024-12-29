// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: check whether or not navx must be inverted

public class Drive extends SubsystemBase {
  private final Field2d field = new Field2d();

  // motors
  private final PWMTalonSRX leftLeader = new PWMTalonSRX(0);
  private final PWMTalonSRX leftFollower = new PWMTalonSRX(1);
  private final PWMTalonSRX rightLeader = new PWMTalonSRX(0);
  private final PWMTalonSRX rightFollower = new PWMTalonSRX(1);

  // sensors
  private final AHRS gyro = new AHRS();
  private final Encoder leftEncoder = new Encoder(0, 1, true);
  private final Encoder rightEncoder = new Encoder(2, 3, false);

  // vs code seems to think this is 0. that is obviously not particularly useful
  // so if this doesn't work calculate this yourself and sub in the constant
  // see "docs/Encoder Pulses Per Rotation.md"
  private static final double pulsesPerRotation = (0.15 * Math.PI) * (1 / 8.45) * (1 / 4096);

  /* Calculates motor speeds during teleop. */
  public final DifferentialDrive drive = new DifferentialDrive(leftLeader::set, rightLeader::set);

  // See: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html
  // trackWidthMeters is best determined using SysID (measuring it is inaccurate since the wheels slip when driving)
  /* Calculates motor speeds when given a ChassisSpeeds (speed + direction to travel in). */
  public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7);

  /* Calculates the position the robot is in on the field. */
  public final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);

  /* 
   * PIDs and Feedforwards help to convert wheel speeds to motor speeds.
   * These must be tuned for the robot or they will be ineffective. SysID (below)
   * (https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html)
   * can be used to determine the necessary tuning values.
   */
  public final PIDController leftPID = new PIDController(0.0, 0.0, 0.0);
  public final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.0, 1.0, 0.0);
  public final PIDController rightPID = new PIDController(0.0, 0.0, 0.0);
  public final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0.0, 1.0, 0.0);

  /** Creates a new Drive. */
  public Drive() {
    leftLeader.addFollower(leftFollower);
    rightLeader.addFollower(rightFollower);

    // will also invert any followers
    leftLeader.setInverted(true);
    rightLeader.setInverted(false);

    leftEncoder.setDistancePerPulse(pulsesPerRotation);
    rightEncoder.setDistancePerPulse(pulsesPerRotation);

    // see PathPlanner docs for an explanation
    AutoBuilder.configureRamsete(
      this::getPose,
      this::resetPose,
      this::getCurrentSpeeds,
      this::driveWithChassisSpeeds,
      new ReplanningConfig(),
      // whether or not to flip the path (true if the field is on the red side)
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

    // only needs to be called once
    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Speed (m/s)", leftEncoder.getRate());
    SmartDashboard.putNumber("Left Distance Travelled (m)", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Speed (m/s)", rightEncoder.getRate());
    SmartDashboard.putNumber("Right Distance Travelled (m)", rightEncoder.getDistance());
    SmartDashboard.putNumber("Gyro Heading (CCW+, degrees)", gyro.getRotation2d().getDegrees());
    field.setRobotPose(getPose());
  }

  /**
   * Drive the robot according to a ChassisSpeeds object (specifies a direction
   * and speed the robot should be driving at).
   * @param speeds The desired speed and direction of the robot..
   */
  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    leftLeader.setVoltage(
      leftPID.calculate(leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond)
      + leftFF.calculate(wheelSpeeds.leftMetersPerSecond)
    );
    rightLeader.setVoltage(
      rightPID.calculate(rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond)
      + rightFF.calculate(wheelSpeeds.rightMetersPerSecond)
    );
  }

  /**
   * Get the current position of the robot on the field.
   * @return The current position of the robot on the field as a Pose2D.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Get a ChassisSpeeds object representing how fast the robot is going.
   * @return How fast the robot is going as a ChassisSpeeds object.
   */
  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate())
    );
  }

  /**
   * Reset the position of the robot on the field.
   * @param pose The new position of the robot on the field.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
      gyro.getRotation2d(),
      leftEncoder.getDistance(),
      rightEncoder.getDistance(),
      pose
    );
  }
}
