// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  private final PWMTalonSRX launchMotor = new PWMTalonSRX(5);
  private final PWMTalonSRX feedMotor = new PWMTalonSRX(6);

  /** Creates a new Launcher. */
  public Launcher() {
    SmartDashboard.putNumber("Launch Motor Speed", 0);
    SmartDashboard.putNumber("Feed Motor Speed", 0);
  }

  @Override
  public void periodic() {}

  /**
   * Run the motor until disabled, set to a new speed, or stopped.
   * @param speed How fast to run the motor.
   */
  public void runLaunchMotor(double speed) {
    launchMotor.set(speed);
    SmartDashboard.putNumber("Launch Motor Speed", speed);
  }

  /**
   * Run the motor until disabled, set to a new speed, or stopped.
   * @param speed How fast to run the motor.
   */
  public void runFeedMotor(double speed) {
    feedMotor.set(speed);
    SmartDashboard.putNumber("Feed Motor Speed", speed);
  }

  /**
   * Stop the motors.
   */
  public void stopMotors() {
    launchMotor.stopMotor();
    SmartDashboard.putNumber("Launch Motor Speed", 0);
    feedMotor.stopMotor();
    SmartDashboard.putNumber("Feed Motor Speed", 0);
  }

  // Command Factories

  public Command getIntakeNoteCommand() {
    return this.startEnd(
      () -> {
        runLaunchMotor(-1);
        runFeedMotor(-1);
      },
      () -> stopMotors()
    );
  }

  public Command getLaunchNoteCommand() {
    return this.runOnce(() -> runLaunchMotor(1))
      .withTimeout(1)
      .andThen(this.runOnce(() -> runFeedMotor(1)))
      .handleInterrupt(() -> stopMotors());
  }
}
