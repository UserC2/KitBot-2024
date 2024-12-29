// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Launcher;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private final Field2d field = new Field2d();

  /*
   * Subsystems and controllers are marked as private and passed to commands to teach "dependency injection".
   * (https://docs.wpilib.org/en/stable/docs/software/frc-glossary.html#term-dependency-injection)
   * Competition robot code usually has these marked as public static.
   */
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Drive drive = new Drive();
  private final Launcher launcher = new Launcher();

  public RobotContainer() {
    configureBindings();

    // display all autos on NetworkTables
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData(field);
  }

  private void configureBindings() {
    // register commands used in PathPlanner GUI

    NamedCommands.registerCommand("LaunchNote", launcher.getLaunchNoteCommand());
    NamedCommands.registerCommand("IntakeNote", launcher.getIntakeNoteCommand());

    // default commands

    // allow the driver to control the robot when it isn't doing anything else
    drive.setDefaultCommand(new TeleopDrive(drive, controller));

    // controls

    controller.leftBumper().whileTrue(launcher.getIntakeNoteCommand());
    controller.rightBumper().whileTrue(launcher.getLaunchNoteCommand());

    // display the robot, target pose, and path during autonomous
    
    // disabled since Drive already does this
    // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
    //   field.setRobotPose(pose);
    // });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
