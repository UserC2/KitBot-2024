// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

public class TeleopDrive extends Command {
  private final Drive drive;
  private final CommandXboxController controller;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this is done here instead of in TeleopDrive() in case it is changed by another command
    drive.drive.setDeadband(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Why leftY is inverted: (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#joystick-and-controller-coordinate-system)
    final double xSpeed = -controller.getLeftY();
    final double rotation = controller.getRightX();
    drive.drive.arcadeDrive(xSpeed, rotation, true);
    SmartDashboard.putNumber("Arcade Drive X Speed", xSpeed);
    SmartDashboard.putNumber("Arcade Drive Rotation", rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
