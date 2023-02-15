// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTestCommand;
import frc.robot.commands.DriveThroughTrajectory;
import frc.robot.commands.Engage;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.UnderglowSetCommand;
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
  private final Bling m_bling = new Bling();
  private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder(m_driveSubsystem);
//  private final Arm m_arm = new Arm();
  private final Underglow m_underglow = new Underglow();
  private final UnderglowSetCommand m_underglowSetCommand = new UnderglowSetCommand(m_underglow, m_OI);
  private final Claw m_claw = new Claw();
  private final Engage m_engage = new Engage(m_driveSubsystem, 0.25);  
  private final OpenMV m_openMV = new OpenMV(SerialPort.Port.kUSB);
  
  //Auto Chooser
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kBasicEngage = "Basic Engage";
  private static final String kEngagePlus = "Engage Plus";
  private static final String kLeaveCommunity = "Leave Community";
  private static final String kTestMode = "Test Mode";
  private static final String kScoreHybrid = "Score Hybrid";
  private static final String kTrajectoryWaypoint = "Traj Waypoint";

  // private final SendableChooser<String> m_robotLocation = new SendableChooser<String>();
  // private static final String kPose1 = "Position 1";
  // private static final String kPose2 = "Position 2";
  // private static final String kPose3 = "Position 3";


  public RobotContainer() {

    // Set default commands
    CommandScheduler.getInstance().setDefaultCommand(m_driveSubsystem, m_teleopCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_underglow, m_underglowSetCommand);
   
    m_chooser.setDefaultOption("Basic Engage", kBasicEngage);
    m_chooser.addOption("Engage Plus", kEngagePlus);
    m_chooser.addOption("Leave Community", kLeaveCommunity);
    m_chooser.addOption("Test Mode", kTestMode);
    m_chooser.addOption("Score Hybrid", kScoreHybrid);
    m_chooser.addOption("Traj Waypoint", kTrajectoryWaypoint);
    SmartDashboard.putData("Auto Chooser", m_chooser);

  //   m_robotLocation.setDefaultOption("Position 1", kPose1);
  //   m_robotLocation.addOption("Position 2", kPose2);
  //   m_robotLocation.addOption("Position 3", kPose3);
  //   SmartDashboard.putData("Robot Position Selector", m_robotLocation);
  // 
  }

  public static void initPreferences() {
    System.out.println("RobotContainer: init Preferences.");
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

  private void configureBindings() {
    System.out.println("RobotContainer: configure Bindings");
  }

  public void setTestMode() {
    DriveTestCommand dtc = new DriveTestCommand(m_driveSubsystem, m_OI);    
    dtc.schedule();
    m_underglow.setLEDIntensity(0.7, 0.7, 0.0); // Orangeish.

    System.out.println("Robot Container: Test mode set");
  }

  public Command getAutonomousCommand() {
    //forward = x, backward = -x, left = y, right = -y
    /* 
    return new SequentialCommandGroup(
      new DriveToPoint(m_driveSubsystem, new Pose2d(0, -3, new Rotation2d()), .5, .5)
      //new DriveToPoint(m_driveSubsystem, new Pose2d(0, 0, new Rotation2d()), .5, .5)
      ); //returns robot position and angle to zero
  }*/

  /*test making last y negative and see results
  * used for MPR testing
  */
//     ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
//       waypoints.add(new Pose2d(1.0, 0.0, new Rotation2d()));
//       waypoints.add(new Pose2d(1.5, 1.0, new Rotation2d()));
//       waypoints.add(new Pose2d(2.0, 2.0, new Rotation2d()));
//       waypoints.add(new Pose2d(2.5, 1.0, new Rotation2d()));
//       waypoints.add(new Pose2d(3.0, 0.0, new Rotation2d()));
//       waypoints.add(new Pose2d(3.5, 1.0, new Rotation2d(3)));
//     return new SequentialCommandGroup(
//       new DriveThroughTrajectory(m_driveSubsystem, new Pose2d(0,0, new Rotation2d()), waypoints, 0.5, 
//       0.8, 0.5, 0.5)
//     );

    //used for 107 testing
    //  ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
     //   waypoints.add(new Pose2d(0.0, 0.0, new Rotation2d()));
     //   waypoints.add(new Pose2d(0.5, 0.0, new Rotation2d()));
//       waypoints.add(new Pose2d(2.0, 2.0, new Rotation2d()));
//     //   waypoints.add(new Pose2d(2.5, 1.0, new Rotation2d()));
//       waypoints.add(new Pose2d(3.0, 0.0, new Rotation2d()));////
//       waypoints.add(new Pose2d(1.0, 0.0, new Rotation2d(3)));
     // return new SequentialCommandGroup(
     //   new DriveThroughTrajectory(m_driveSubsystem, new Pose2d(0,0, new Rotation2d()), waypoints, 0.5, 
     //   0.8, 0.5, 0.5)
     // );

   // return new SequentialCommandGroup(new Engage(m_driveSubsystem, 0.3));

    //return new DriveTestCommand(m_driveSubsystem, m_OI);

    System.out.println(m_chooser.getSelected());

    switch (m_chooser.getSelected()) {
      case kBasicEngage:
        return basicEngage();
      case kEngagePlus:
        return engagePlus();
      case kLeaveCommunity:
        return leaveCommunity();
      case kTestMode:
        return testMode();
      case kScoreHybrid:
        return scoreHybrid();
      case kTrajectoryWaypoint:
        return trajectoryWaypoint();
      default:
        System.out.println("No Auto Selected -_-");
        return null;
    }
  }

  public Command basicEngage() {
    return new SequentialCommandGroup(new Engage(m_driveSubsystem, 0.3));
  }

  public Command engagePlus() {
    System.out.println("Left Community and then engaged");
    return null;
  }

  public Command leaveCommunity() {
    // switch (m_robotLocation.getSelected()) {
    //   case kPose1:
    //     System.out.println("Leaving Community from position 1");
    //     return null;
    //   case kPose2:
    //     System.out.println("Leaving Community from position 2");
    //     return null;
    //   case kPose3:
    //     System.out.println("Leaving Community from position 3");
    //     return null;
    //   default:
    //     return null;
    // }
    return null;
  }

  public Command testMode() {
    System.out.println("Test Mode on");
    return new DriveTestCommand(m_driveSubsystem, m_OI);
  }

  public Command scoreHybrid() {
    System.out.println("Hybrid Scored");
    return null;
  }

  public Command trajectoryWaypoint() {
    System.out.println("Waypoint Beginning");

    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, new Rotation2d()));
        waypoints.add(new Pose2d(0.5, 0.0, new Rotation2d()));
//       waypoints.add(new Pose2d(2.0, 2.0, new Rotation2d()));
//     //   waypoints.add(new Pose2d(2.5, 1.0, new Rotation2d()));
//       waypoints.add(new Pose2d(3.0, 0.0, new Rotation2d()));////
       waypoints.add(new Pose2d(1.0, 0.0, new Rotation2d(3)));
     // return new SequentialCommandGroup(
     //   new DriveThroughTrajectory(m_driveSubsystem, new Pose2d(0,0, new Rotation2d()), waypoints, 0.5, 0.8, 0.5, 0.5));

    return new SequentialCommandGroup(new DriveThroughTrajectory(m_driveSubsystem, new Pose2d(0,0, new Rotation2d()), waypoints, 0.5, 0.8, 0.5, 0.5));
  }

  public void setStartupLighting()
  {
    // Pick intensity based on driver station connection.
    double intensity = 0.3; // Default to dim.
    if (DriverStation.isDSAttached()) {
      intensity = 1.0; // Bright if attached.
    }
    // Set lighting to driver station aliance color.
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      m_underglow.setLEDIntensity(0, 0, intensity);
    }
    else {
      m_underglow.setLEDIntensity(intensity, 0, 0);
    }
  }

}