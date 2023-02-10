// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveThroughTrajectory;
import frc.robot.commands.Engage;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleConfig;
import frc.robot.subsystems.Underglow;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.OpenMV;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.AprilTagDetection;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.Claw;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final OI m_OI = new OI();
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_driveSubsystem, m_OI);
  private final Bling m_Bling = new Bling();
  private final AprilTagFinder m_AprilTagFinder = new AprilTagFinder(m_driveSubsystem);
  private final Arm m_Arm = new Arm();
  private final Underglow m_Underglow = new Underglow();
  private final Claw m_Claw = new Claw();
  private final Engage m_Engage = new Engage(m_driveSubsystem, 0.25);
  
  private final OpenMV m_openMV = new OpenMV(SerialPort.Port.kUSB);

  public RobotContainer() {

    // Configure Button Bindings
    configureBindings();

    // Set default commands
    CommandScheduler.getInstance().setDefaultCommand(m_driveSubsystem, m_teleopCommand);
  }

  public static void initPreferences() {
    // Initialize Preferences For Subsystem Classes:
    SwerveModuleConfig.initPreferences();
    DriveSubsystem.initPreferences();
    AprilTagDetection.initPreferences();
    Arm.initPreferences();
    Claw.initPreferences();
    Bling.initPreferences();
    OI.initPreferences();
    SwerveModule.initPreferences();
    Underglow.initPreferences();

    // Initialize Preferences For Command Classes:

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    //forward = x, backward = -x, left = y, right = -y
    /* 
    return new SequentialCommandGroup(
      new DriveToPoint(m_driveSubsystem, new Pose2d(0, -3, new Rotation2d()), .5, .5)
      //new DriveToPoint(m_driveSubsystem, new Pose2d(0, 0, new Rotation2d()), .5, .5)
      ); //returns robot position and angle to zero
  }*/

  //test making last y negative and see results
    // ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
    //   waypoints.add(new Pose2d(1.0, 0.0, new Rotation2d()));
    //   waypoints.add(new Pose2d(1.0, 1.0, new Rotation2d()));
    //   waypoints.add(new Pose2d(2.0, 2.0, new Rotation2d(3)));
    // return new SequentialCommandGroup(
    //   new DriveThroughTrajectory(m_driveSubsystem, new Pose2d(0,0, new Rotation2d()), waypoints, 0.5, 
    //   0.8, 0.5, 0.5)
    // );
    return new SequentialCommandGroup(new Engage(m_driveSubsystem, 0.4));
  }
}