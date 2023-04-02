// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.AlignToGamePiece;
import frc.robot.commands.AllianceUnderglow;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmSetPosition;
import frc.robot.commands.ArmSplinePosition;
import frc.robot.commands.BlingTeleopCommand;
import frc.robot.commands.DriveTestCommand;
import frc.robot.commands.DriveThroughTrajectory;
import frc.robot.commands.Engage;
import frc.robot.commands.EngageBalance;
import frc.robot.commands.TeleopClaw;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopSetArm;
import frc.robot.commands.UnderglowSetCommand;
import frc.robot.commands.UpdateMotorEncoders;
import frc.robot.commands.CollectCommand;
import frc.robot.commands.DepositCommand;
import frc.robot.commands.EngageDriveUp;
import frc.robot.commands.EngageForward;
import frc.robot.commands.ParkingBrake;
import frc.robot.commands.PlannedArmPath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GamePieceFinder;
import frc.robot.subsystems.SwerveModuleConfig;
import frc.robot.subsystems.Underglow;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.OpenMV;
import frc.robot.subsystems.PathPlanner;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.Claw;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final OI m_OI = new OI();
  private final Bling m_bling = new Bling();
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_driveSubsystem, m_OI, m_bling);
  private final BlingTeleopCommand m_blingTeleopCommand = new BlingTeleopCommand(m_bling, m_OI);
  // private final AprilTagFinder m_aprilTagFinder = new
  // AprilTagFinder(m_driveSubsystem, null, null);
  private final AprilTagFinder m_frontCamera = new AprilTagFinder(m_driveSubsystem, "FrontVision",
      new Transform3d(new Translation3d(0.2159, 0.1397, 0.508), new Rotation3d(0, 0.2617, 0)));
  private final GamePieceFinder m_gamePieceFinder = new GamePieceFinder(m_driveSubsystem, "FrontVision");
  // private final AprilTagFinder m_rearCamera = new
  // AprilTagFinder(m_driveSubsystem, "RearVision",
  // new Transform3d(new Translation3d(0.2159, -0.1397, 0.508), new Rotation3d(0,
  // -0.2617, 0)));
  private final Arm m_arm = new Arm();
  private final PathPlanner m_pathPlanner = new PathPlanner();
  private final TeleopSetArm m_armSetCommand = new TeleopSetArm(m_arm, m_OI);
  private final Underglow m_underglow = new Underglow();
  // private final UnderglowSetCommand m_underglowSetCommand = new
  // UnderglowSetCommand(m_underglow, m_OI);
  private final Claw m_claw = new Claw();
  private final TeleopClaw m_clawCommand = new TeleopClaw(m_claw, m_OI);
  private final AllianceUnderglow m_allianceUnderglow = new AllianceUnderglow(m_underglow);

  // private final OpenMV m_openMV = new OpenMV(SerialPort.Port.kUSB);

  // Auto Chooser
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kNoAuto = "No Autonomous";
  private static final String kBasicEngage = "Basic Engage";
  private static final String kSplitEngage = "Split Engage";
  private static final String kEngagePlus = "Engage Plus";
  private static final String kLeaveCommunity = "Leave Community";
  private static final String kTestMode = "Test Mode";
  private static final String kAlignToAprilTag = "Align To AprilTag";
  private static final String kScoreCube = "Score Cube";
  private static final String kScoreCubeAndEngage = "Score Cube and Engage";
  private static final String kCubeEngageLeaveCommand = "Score Cube, Leave Community, Engage";
  private static final String kCubeLeaveCommand = "Score Cube and Leave Community";
  private static final String kShootHighCubeLeaveEngage = "Shoot Cube, Leave Commmunity, Engage";
  private static final String kShootHighCubeLeaveBarrierCollect = "Shoot Cube BarrierLeave Collect";

  private static final String kArmTest = "Arm Test Command";
  private static final String kTrajectoryTest = "Drive trajectory Test";
  // private static final String kScoreHybrid = "Score Hybrid";
  // private static final String kTrajectoryWaypoint = "Traj Waypoint";

  // private final SendableChooser<String> m_robotLocation = new
  // SendableChooser<String>();
  // private static final String kPose1 = "Position 1";
  // private static final String kPose2 = "Position 2";
  // private static final String kPose3 = "Position 3";

  /**
   * Robot container constructor
   * Set default commands, adds options to the auto chooser.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(m_driveSubsystem, m_teleopCommand);
    // CommandScheduler.getInstance().setDefaultCommand(m_arm, m_armCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_arm, m_armSetCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_claw, m_clawCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_bling, m_blingTeleopCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_underglow, m_allianceUnderglow);

    m_chooser.setDefaultOption("No Autonomous", kNoAuto);
    m_chooser.setDefaultOption("Split Engage", kSplitEngage);
    m_chooser.addOption("Leave (All)", kLeaveCommunity);
    m_chooser.addOption("Cube and Engage (2)", kScoreCubeAndEngage);
    m_chooser.addOption("Cube and Leave (All)", kCubeLeaveCommand);
    m_chooser.addOption("Cube, Leave, and Engage (2)", kCubeEngageLeaveCommand);
    m_chooser.addOption("1_B_2023_1073", kScoreCube);
    m_chooser.addOption("2_B_2023_1073", kScoreCube);
    m_chooser.addOption("3_B_2023_1073", kScoreCube);
    m_chooser.addOption("2_C_2023_1073", kBasicEngage);
    m_chooser.addOption("2_D_2023_1073", kEngagePlus);
    m_chooser.addOption("Arm test", kArmTest);
    m_chooser.addOption("Drive Trajectory Test", kTrajectoryTest);
    m_chooser.addOption("Shoot Cube Leave Engage", kShootHighCubeLeaveEngage);
    m_chooser.addOption("Shoot Cube Barrier LeaveCollect", kShootHighCubeLeaveBarrierCollect);

    // m_chooser.addOption("Test Mode", kTestMode);
    // m_chooser.addOption("Align To AprilTag", kAlignToAprilTag);
    // m_chooser.addOption("Arm Set Position test", kArmTest);

    // WEEK 0: commented out superfluous auto choices so DT wouldn't accidentally
    // choose them
    // m_chooser.addOption("Score Hybrid", kScoreHybrid);
    // m_chooser.addOption("Traj Waypoint", kTrajectoryWaypoint);
    SmartDashboard.putData("Auto Chooser", m_chooser);

    // m_robotLocation.setDefaultOption("Position 1", kPose1);
    // m_robotLocation.addOption("Position 2", kPose2);
    // m_robotLocation.addOption("Position 3", kPose3);
    // SmartDashboard.putData("Robot Position Selector", m_robotLocation);
    configureBindings();
    //
  }

  // Initialize Preferences For Subsystem Classes:
  public static void initPreferences() {
    System.out.println("RobotContainer: init Preferences.");
    SwerveModuleConfig.initPreferences();
    DriveSubsystem.initPreferences();
    AprilTagFinder.initPreferences();
    Arm.initPreferences();
    Claw.initPreferences();
    Bling.initPreferences();
    OI.initPreferences();
    SwerveModule.initPreferences();
    Underglow.initPreferences();
    EngageBalance.initPreferences();
    EngageDriveUp.initPreferences();
    EngageForward.initPreferences();

    // Initialize Preferences For Command Classes:
    Preferences.initDouble("Stow Step 1 shoulder", -1.9);
    Preferences.initDouble("Stow Step 1 elbow", 3.3);
    Preferences.initDouble("Stow Step 2 shoulder", -3.84);
    Preferences.initDouble("Stow Step 2 elbow", 2.95);
    Preferences.initDouble("Double Substation shoulder", -2.0576);
    Preferences.initDouble("Double Substation elbow", 3.69);
    Preferences.initDouble("Middle Node shoulder", -1.48);
    Preferences.initDouble("Middle Node elbow", 3.75);
    Preferences.initDouble("High Node shoulder", -0.652);
    Preferences.initDouble("High Node elbow", 3.3086);
    Preferences.initDouble("Cube Ground Aim Shoulder", -0.949);
    Preferences.initDouble("Cube Ground Aim Elbow", 5.1665);
  }

  public void diagnostics() {
    // TODO insert diagnostics routines
    boolean allGood = false;

    String armDiagnostics = m_arm.getDiagnostics();
    String blingDiagnostics = m_bling.getDiagnostics();
    String clawDiagnostics = m_claw.getDiagnostics();
    String driveSubDiagnostics = m_driveSubsystem.getDiagnostics();
    String underglowDiagnostics = m_underglow.getDiagnostics();
    String oiDiagnostics = m_OI.getDiagnostics();
    String aprilTagDiagnostics = m_frontCamera.getDiagnostics();

    SmartDashboard.putString("Diag/Arm", armDiagnostics);
    SmartDashboard.putString("Diag/Bling", blingDiagnostics);
    SmartDashboard.putString("Diag/Underglow", underglowDiagnostics);
    SmartDashboard.putString("Diag/Claw", clawDiagnostics);
    SmartDashboard.putString("Diag/Drive Subsystem", driveSubDiagnostics);
    SmartDashboard.putString("Diag/OI", oiDiagnostics);
    SmartDashboard.putString("Diag/AprilTag", aprilTagDiagnostics);

    if (armDiagnostics.isEmpty() && blingDiagnostics.isEmpty() && clawDiagnostics.isEmpty()
        && driveSubDiagnostics.isEmpty() && oiDiagnostics.isEmpty() && aprilTagDiagnostics.isEmpty()) {
      allGood = true;
      SmartDashboard.putBoolean("Diag/All Good", allGood);
    } else {
      SmartDashboard.putBoolean("Diag/All Good", allGood);
    }

  }

  // called when robot initializes. Sets parking brake to false
  public void teleopInit() {
    m_driveSubsystem.parkingBrake(false);
    // m_arm.initializeShoulder();
  }

  public void disableInit() {
    m_arm.disableMotors();
  }

  // Configures the button mappings for controllers
  private void configureBindings() {
    System.out.println("RobotContainer: configure Bindings");

    // Trigger for the arm to stow
    Trigger stowTrigger = new Trigger(m_OI::getOperatorAButton);
    stowTrigger.onTrue(armStowCommand(m_OI));

    Trigger alignToAprilTag = new Trigger(m_OI::getYButton);
    alignToAprilTag.whileTrue(alignToAprilTag(-0.09, 0.8));

    Trigger leftAlignToAprilTag = new Trigger(m_OI::getXButton);
    leftAlignToAprilTag.whileTrue(alignToAprilTag(0.44, 1.0));

    Trigger rightAlignToAprilTag = new Trigger(m_OI::getBButton);
    rightAlignToAprilTag.whileTrue(alignToAprilTag(-0.62, 1.0));

    Trigger operatorCubeMode = new Trigger(m_OI::getOperatorViewButton);
    operatorCubeMode.onTrue(new InstantCommand(m_OI::setCubeMode));

    Trigger operatorConeMode = new Trigger(m_OI::getOperatorMenuButton);
    operatorConeMode.onTrue(new InstantCommand(m_OI::setConeMode));

    Trigger midTrigger = new Trigger(m_OI::getOperatorXButton);
    midTrigger.onTrue(midScoreCommand(m_OI));

    Trigger highTrigger = new Trigger(m_OI::getOperatorYButton);
    highTrigger.onTrue(highScoreCommand(m_OI));

    Trigger doubleSubstationTrigger = new Trigger(m_OI::getOperatorBButton);
    doubleSubstationTrigger.onTrue(doubleSubstationScore(m_OI));

    Trigger groundPickupTrigger = new Trigger(m_OI::getOperatorDPadDown);
    groundPickupTrigger.onTrue(groundPickupCommand(m_OI));

      Trigger singleSubstationTrigger = new Trigger(m_OI :: getOperatorDPadRight);
      singleSubstationTrigger.onTrue(singleSubstation(m_OI));

      Trigger alignToGamePiece = new Trigger(m_OI:: getRightBumper);
      alignToGamePiece.whileTrue(alignToGamePiece(m_OI));

      // Trigger alternateStowTrigger = new Trigger(m_OI :: getOperatorDPadUp);
      // alternateStowTrigger.onTrue(alternateArmStowCommand());
      
    }

  // Trigger updateMotorEncodersTrigger = new Trigger(m_OI ::
  // getOperatorLeftTriggerButton);
  // updateMotorEncodersTrigger.onTrue(updateMotorEncoders());

  /**
   * Sets test mode
   */
  public void setTestMode() {
    DriveTestCommand dtc = new DriveTestCommand(m_driveSubsystem, m_OI);
    dtc.schedule();
    m_underglow.setLEDIntensity(0.7, 0.7, 0.0); // Orangeish.

    System.out.println("Robot Container: Test mode set");
  }

  /**
   * Uses autonomous commands stored in auto chooser to run the command that is
   * chosen from shuffleboard
   * 
   * @return Autonomous command that is intended to be run
   */
  public Command getAutonomousCommand() {

    System.out.println(String.format("Autonomous Command Selected: %s", m_chooser.getSelected()));

    switch (m_chooser.getSelected()) {
      case kNoAuto:
        return null;
      // no command ^^^
      case kBasicEngage:
        return basicEngage();
      case kEngagePlus:
        return engagePlus();
      // case kEngageExperimental:
      // return engageExperimental();
      case kSplitEngage:
        return splitEngage();
      case kLeaveCommunity:
        return leaveCommunity();
      case kTestMode:
        return testMode();
      // case kScoreHybrid:
      // return scoreHybrid();
      // case kTrajectoryWaypoint:
      // return trajectoryWaypoint();
      case kAlignToAprilTag:
        return alignToAprilTag(-0.9, 0.8);
      case kArmTest:
        return armSetTest();
      case kScoreCube:
        return scoreHighCubeCommand();
      case kScoreCubeAndEngage:
        return scoreHighCubeAndEngageCommand();
      case kCubeEngageLeaveCommand:
        return cubeEngageLeaveCommand();
      case kCubeLeaveCommand:
        return cubeLeaveCommand();
      case kTrajectoryTest:
        return testTrajectory();
      case kShootHighCubeLeaveEngage:
        return shootHighCubeLeaveEngage();
      case kShootHighCubeLeaveBarrierCollect:
        return shootHighCubeLeaveBarrierCollect();
      default:
        System.out.println("No Auto Selected -_-");
        return null;
    }
  }

  /**
   * First positions the arm to higher position to avoid clipping ridges on nodes
   * or human player stations. After,
   * it stows the arm to resting position.
   * 
   * @return Command that sets arm position
   */
  public Command armStowCommand(OI oi) {
    ArrayList<Arm.JointWaypoints> waypoints = new ArrayList<Arm.JointWaypoints>();
    waypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.8));
    waypoints.add(m_arm.new JointWaypoints(-3.87, 2.9, -1.21, 2.9));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);
    return new SequentialCommandGroup(

        new ArmSplinePosition(m_arm, waypoints, velocity, 0));
  }

  public Command alignToGamePiece(OI oi) {
    return new ConditionalCommand(
      new AlignToGamePiece(m_driveSubsystem, m_bling, m_gamePieceFinder, 0.3, true),
      new AlignToGamePiece(m_driveSubsystem, m_bling, m_gamePieceFinder, 0.3, false),
      oi::isCubeMode);
  }

  // public Command alternateArmStowCommand(){
  // Arm.JointVelocities velocity = m_arm.new JointVelocities(0.6, 0.6, 0.6);
  // return new PlannedArmPath(m_arm, m_pathPlanner, 16, velocity);

  // }

  public Command groundPickupCommand(OI oi) {
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    // cubeWaypoints.add(m_arm.new JointWaypoints(-1.78, 3.98, -1.19, 2.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.19, 5.38, -1.21, 4.0));

    ArrayList<Arm.JointWaypoints> coneWaypoints = new ArrayList<Arm.JointWaypoints>();
    coneWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    // coneWaypoints.add(m_arm.new JointWaypoints(-1.78, 3.98, 0.0, 2.0));
    coneWaypoints.add(m_arm.new JointWaypoints(-1.73, 4.59, 0.67, 4.0));
    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    return new ConditionalCommand(
        new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
        new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
        oi::isCubeMode);
  }

  /**
   * Positions arm to pick up from double substation
   * 
   * @return Command that sets arm position
   */

  public Command doubleSubstationScore(OI oi) {
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 2.0));
    // cubeWaypoints.add(m_arm.new JointWaypoints(-2.5, 3.3, -0.5, 3.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.45, 3.47, -0.33, 4.0));

    ArrayList<Arm.JointWaypoints> coneWaypoints = new ArrayList<Arm.JointWaypoints>();
    coneWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 2.0));
    // coneWaypoints.add(m_arm.new JointWaypoints(-2.35, 2.95, 0.1, 3.0));
    coneWaypoints.add(m_arm.new JointWaypoints(-2.11, 3.1, 1.41, 4.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    return new ConditionalCommand(
        new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
        new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
        oi::isCubeMode);
  }

  /**
   * Positions arm to score in the middle node for both cone and cube
   * 
   * @return Command that sets arm position
   */
  public Command midScoreCommand(OI oi) {
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.5));
    // cubeWaypoints.add(m_arm.new JointWaypoints(-2.16, 3.3, -0.73, 2.25));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.72, 3.8, -0.26, 3.0));

    ArrayList<Arm.JointWaypoints> coneWaypoints = new ArrayList<Arm.JointWaypoints>();
        coneWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.5));
        //coneWaypoints.add(m_arm.new JointWaypoints(-2.25, 3.13, 0.0, 2.25));
        coneWaypoints.add(m_arm.new JointWaypoints(-1.72, 3.545, 1.189, 3.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    return new ConditionalCommand(
        new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
        new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
        oi::isCubeMode);
  }

  /**
   * Positions the arm to score in the high node for both cone and cube.
   * 
   * @return Command that sets arm position
   */
  public Command highScoreCommand(OI oi) {
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.5));
    // cubeWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.19, -0.77, 2.25));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.483, 3.570, -0.337, 3.0));

    ArrayList<Arm.JointWaypoints> coneWaypoints = new ArrayList<Arm.JointWaypoints>();
        coneWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.5));
        //coneWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.0, 0.0, 2.25));
        coneWaypoints.add(m_arm.new JointWaypoints(-1.207, 3.105, 1.361, 3.0));

        Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    return new ConditionalCommand(
      new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
      new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
      oi :: isCubeMode);
  }

  public Command singleSubstation(OI oi){
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
        cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.5));
        //cubeWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.19, -0.77, 2.25));
        cubeWaypoints.add(m_arm.new JointWaypoints(-3.85, 2.89, 0.25, 3.0));

    ArrayList<Arm.JointWaypoints> coneWaypoints = new ArrayList<Arm.JointWaypoints>();
        coneWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.5));
        //coneWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.0, 0.0, 2.25));
        coneWaypoints.add(m_arm.new JointWaypoints(-2.21, 4.549, -1.209, 3.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    return new ConditionalCommand(
        new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
        new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
        oi::isCubeMode);
  }

  /**
   * First step in the sequence to pick a cube off of the ground. Positions arm
   * above cube to aim before grabbing
   * 
   * @return Command that sets arm position
   */

  /**
   * Second step in the sequence to pick up a cube off of the ground. Positions
   * arm so that end effector is up against the cube
   * 
   * @return Command that sets arm position
   */

  /**
   * An autonomous command to drive up and engage on the charging station using
   * the Engage command
   * 
   * @return Command to do the autonomous outlined above
   */
  public Command basicEngage() {
    return new SequentialCommandGroup(new Engage(m_driveSubsystem, 0.5, false));
  }

  /**
   * Autonomous command that aligns to april tag, scores a cube in high node, and
   * then engages on the charge station.
   * 
   * @return Command to do the autonomous outlined above
   */
  public Command scoreHighCubeAndEngageCommand() {
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.6, 2.8, -1.2, 1.5));
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.19, -0.77, 2.25));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.483, 3.570, -0.337, 3.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.6, 0.6, 0.6);

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0.5),
            // new AlignToAprilTag(m_driveSubsystem, m_bling, m_frontCamera, 0.5, 0),
            new CollectCommand(m_claw, true, 0.8)),
        new DepositCommand(m_claw, true, 1.0),
        new ParallelCommandGroup(
            armStowCommand(m_OI),
            new SequentialCommandGroup(
                new EngageDriveUp(m_driveSubsystem, Preferences.getDouble("EngageDriveUp.maxSpeed", 0.9), false),
                new EngageForward(m_driveSubsystem, Preferences.getDouble("EngageForward.maxSpeed", 0.7), false),
                new EngageBalance(m_driveSubsystem, Preferences.getDouble("EngageBalance.maxSpeed", 0.7), false),
                new ParkingBrake(m_driveSubsystem, m_bling))));
  }

  /**
   * Autonomous command that scores a cube in high node, leaves the community, and
   * then engages on the charge station.
   * 
   * @return Command to do the autonomous outlined above
   */
  public Command shootHighCubeLeaveEngage() {
    ArrayList<Pose2d> communityWaypoints = new ArrayList<Pose2d>();
    communityWaypoints.add(new Pose2d(2.5, 0, new Rotation2d(3.14)));

    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new SequentialCommandGroup(
            new DriveThroughTrajectory(m_driveSubsystem, communityWaypoints, 0.8,
                1.0, 0.5, 0.5),
            new EngageDriveUp(m_driveSubsystem, Preferences.getDouble("EngageDriveUp.maxSpeed", 0.9), true),
            new EngageForward(m_driveSubsystem, Preferences.getDouble("EngageForward.maxSpeed", 0.7), true),
            new EngageBalance(m_driveSubsystem, Preferences.getDouble("EngageBalance.maxSpeed", 0.7), true),
            new ParkingBrake(m_driveSubsystem, m_bling)));
  }

  public Command shootHighCubeLeaveBarrierCollect() {
    ArrayList<Pose2d> leaveWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      leaveWaypoints.add(new Pose2d(1.3, -.3, new Rotation2d(Math.PI)));
      leaveWaypoints.add(new Pose2d(2, -0.1, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.58, 0, new Rotation2d(0)));
    } else {
      leaveWaypoints.add(new Pose2d(2.4, -0.15, new Rotation2d(0)));
      // TODO: actually put in waypoints for blue alliance
    }

    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.05, 5.38, -1.21, 4.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.9, 0.9, 0.9);

    ArrayList<Pose2d> returnWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      returnWaypoints.add(new Pose2d(2, -0.1, new Rotation2d(0)));
      returnWaypoints.add(new Pose2d(1.3, -.3, new Rotation2d(Math.PI)));
      returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));  
    } else {
      returnWaypoints.add(new Pose2d(2.4, -0.15, new Rotation2d(0)));
      // TODO: actually put in waypoints for blue alliance
    }


    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new ParallelDeadlineGroup(
          new DriveThroughTrajectory(m_driveSubsystem, leaveWaypoints, 1.5, 1.2, 0.5, 0.7),
          new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0.5),
          new CollectCommand(m_claw, true, 12)),
        new ParallelDeadlineGroup(
            new DriveThroughTrajectory(m_driveSubsystem, returnWaypoints, 1.5, 1.2, 0.5, 0.7),
            armStowCommand(m_OI)),
        new ParallelDeadlineGroup(
          midScoreCommand(m_OI),
          alignToAprilTag(-0.04, 0.6)),
        //new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        armStowCommand(m_OI));

  }

  /**
   * Autonomous command that aligns to april tags, scores a cube, leaves
   * community, and then comes back to engage.
   * 
   * @return Command to do the autonomous outlined above
   */
  public Command cubeEngageLeaveCommand() {

    ArrayList<Pose2d> communityWaypoints = new ArrayList<Pose2d>();
    communityWaypoints.add(new Pose2d(2.5, 0, new Rotation2d(3.14)));

    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.6, 2.8, -1.2, 1.5));
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.19, -0.77, 2.25));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.483, 3.570, -0.337, 3.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.6, 0.6, 0.6);

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0.5),
            new CollectCommand(m_claw, true, 0.8)),
        new DepositCommand(m_claw, true, 1.0),
        new ParallelDeadlineGroup(
            new DriveThroughTrajectory(m_driveSubsystem, communityWaypoints, 0.8,
                1.0, 0.5, 0.5),
            armStowCommand(m_OI)),
        new EngageDriveUp(m_driveSubsystem, Preferences.getDouble("EngageDriveUp.maxSpeed", 0.9), true),
        new EngageForward(m_driveSubsystem, Preferences.getDouble("EngageForward.maxSpeed", 0.7), true),
        new EngageBalance(m_driveSubsystem, Preferences.getDouble("EngageBalance.maxSpeed", 0.7), true),
        new ParkingBrake(m_driveSubsystem, m_bling));

  }

  /**
   * Autonomous command to score a cube, and then leaves the community. Drives
   * slightly diagonally
   * to avoid clipping the charging station. Ends up with robot turned 180 degrees
   * in order to pick up cube
   * after moving into teleop.
   * 
   * @return Command to score cube and leave community
   */
  public Command cubeLeaveCommand() {

    ArrayList<Pose2d> communityWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      communityWaypoints.add(new Pose2d(2.4, 0.15, new Rotation2d(0)));
    } else {
      communityWaypoints.add(new Pose2d(2.4, -0.15, new Rotation2d(0)));
    }
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.6, 2.8, -1.2, 1.5));
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.19, -0.77, 2.25));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.483, 3.570, -0.337, 3.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.6, 0.6, 0.6);

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0.5),
            new CollectCommand(m_claw, true, 0.8)),
        new DepositCommand(m_claw, true, 1.0),
        new ParallelDeadlineGroup(
            new DriveThroughTrajectory(m_driveSubsystem, communityWaypoints, 1.0, 0.8, 0.5, 0.7),
            armStowCommand(m_OI)));
  }

  /**
   * Autonomous command that aligns to April Tag and activates vacuum in parallel,
   * extends arm to high node position, deactivates
   * vacuum, and stows arm.
   * 
   * @return Command that scores a cube after aligning to the april tag and then
   *         stows
   */
  public Command scoreHighCubeCommand() {
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.6, 2.8, -1.2, 1.5));
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.19, -0.77, 2.25));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.483, 3.570, -0.337, 3.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.6, 0.6, 0.6);

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0.5),
            new CollectCommand(m_claw, true, 0.8)),
        new DepositCommand(m_claw, true, 1.0),
        armStowCommand(m_OI));
  }

  public Command cubeLeaveCubeEngage() {
    ArrayList<Pose2d> communityWaypoints = new ArrayList<Pose2d>();
    communityWaypoints.add(new Pose2d(4, 0, new Rotation2d(3.14)));

    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.6, 2.8, -1.2, 1.5));
    cubeWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.19, -0.77, 2.25));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.483, 3.570, -0.337, 3.0));

    ArrayList<Arm.JointWaypoints> cubeGroundWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeGroundWaypoints.add(m_arm.new JointWaypoints(-2.6, 2.8, -1.2, 1.5));
    cubeGroundWaypoints.add(m_arm.new JointWaypoints(-2.0, 3.19, -0.77, 2.25));
    cubeGroundWaypoints.add(m_arm.new JointWaypoints(-1.483, 3.570, -0.337, 3.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.6, 0.6, 0.6);

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0.5),
            new CollectCommand(m_claw, true, 0.8)),
        new DepositCommand(m_claw, true, 1.0),
        new ParallelDeadlineGroup(
            new DriveThroughTrajectory(m_driveSubsystem, communityWaypoints, 0.5,
                1.0, 0.5, 0.5),
            armStowCommand(m_OI)));
  }

  /**
   * Moves robot back to score a preloaded game piece onto hybrid node, drives
   * over the charging station to leave community,
   * then moves backwards onto the charging station and engages using the Engage
   * command
   * 
   * @return Command that scores preload into hybrid node, leaves community, and
   *         engages
   */
  public Command engagePlus() {
    ArrayList<Pose2d> communityWaypoints = new ArrayList<Pose2d>();
    ArrayList<Pose2d> scoreWaypoints = new ArrayList<Pose2d>();

    scoreWaypoints.add(new Pose2d(-0.3, 0, new Rotation2d(3.14)));
    communityWaypoints.add(new Pose2d(2.25, 0, new Rotation2d(3.14)));

    return new SequentialCommandGroup(
        new DriveThroughTrajectory(m_driveSubsystem, scoreWaypoints, 1.0, 0.8, 0.5, 0.5),
        new DriveThroughTrajectory(m_driveSubsystem, communityWaypoints, 1.0, 0.8, 0.5, 0.5),
        new Engage(m_driveSubsystem, 0.5, true));
    // WEEK 0: changed max velocity in both drive through trajectories to 1.0 from
    // 0.5, and set engage max speed to 0.5 from 0.3
  }

  /**
   * Engages the robot on the charging station using a split up version of the
   * engage command. First the
   * robot drives up to charging station, then drives up the charging station,
   * then goes to balance, and then parks
   * 
   * @return a command to engage the robot on the charging station.
   */
  public Command splitEngage() {
    return new SequentialCommandGroup(
        new EngageDriveUp(m_driveSubsystem, Preferences.getDouble("EngageDriveUp.maxSpeed", 0.9), false),
        new EngageForward(m_driveSubsystem, Preferences.getDouble("EngageForawrd.maxSpeed", 0.7), false),
        new EngageBalance(m_driveSubsystem, Preferences.getDouble("EngageBalance.maxSpeed", 0.7), false),
        new ParkingBrake(m_driveSubsystem, m_bling));
  }

  /**
   * Autonomous command to leave the community. Uses DrivethroughTrajectory to
   * carry robot out of community.
   * 
   * @return a sequential command group containing a DrivethroughTrajectory
   *         command
   */
  public Command leaveCommunity() {

    System.out.println("Leave Community");

    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
    waypoints.add(new Pose2d(2.5, 0.0, new Rotation2d(3.14)));
    waypoints.add(new Pose2d(2.5, -1.0, new Rotation2d(3.14)));
    waypoints.add(new Pose2d(2.5, -1, new Rotation2d(0)));

    return new SequentialCommandGroup(new DriveThroughTrajectory(m_driveSubsystem, waypoints, 1, 1, 0.5, 0.5));

  }

  /**
   * Tests ArmSetPosition
   * 
   * @return an ArmSetPosition Command
   */
  public Command armSetTest() {
    ArrayList<Arm.JointWaypoints> waypoints = new ArrayList<Arm.JointWaypoints>();
    waypoints.add(m_arm.new JointWaypoints(-1.0, 2.6, 0.0, 6.0));
    waypoints.add(m_arm.new JointWaypoints(-1.55, 2.4, 0.0, 9.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.6, 0.6, 0.6);
    return new ArmSplinePosition(m_arm, waypoints, velocity, 0.5);
  }

  public Command testTrajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
    waypoints.add(new Pose2d(0.5, 0, new Rotation2d(3.14)));
    waypoints.add(new Pose2d(0.5, 0.5, new Rotation2d(3.14)));
    waypoints.add(new Pose2d(0, 0.5, new Rotation2d(0)));
    waypoints.add(new Pose2d(0.1, 0.2, new Rotation2d(3.14)));
    return new DriveThroughTrajectory(m_driveSubsystem, waypoints, 0.3, 0.4, 0.5, 0.5);
  }

  public Command updateMotorEncoders() {
    // encoder health is already checked in periodic using u`dateMagnateHealth
    return new UpdateMotorEncoders(m_arm);

  }

  /**
   * A way to test OI and DriveSubsystem while debug mode in DriveSubsystem is on
   * 
   * @return the Command that tests the subsystems
   */
  public Command testMode() {
    System.out.println("Test Mode on");
    return new DriveTestCommand(m_driveSubsystem, m_OI);
  }

  // public Command scoreHybrid() {
  // System.out.println("Hybrid Scored");
  // return null;
  // }

  // public Command trajectoryWaypoint() {
  // System.out.println("Waypoint Beginning");

  // ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();

  // FORWARD TEST

  // waypoints.add(new Pose2d(0.0, 0.0, new Rotation2d()));
  // waypoints.add(new Pose2d(0.5, 0.0, new Rotation2d()));
  // waypoints.add(new Pose2d(1.0, 0.0, new Rotation2d()));
  // waypoints.add(new Pose2d(1.0, 0.0, new Rotation2d()));

  // BOX TEST
  // waypoints.add(new Pose2d(0.0, 0.0, new Rotation2d()));
  // waypoints.add(new Pose2d(1.0, 0.0, new Rotation2d()));
  // waypoints.add(new Pose2d(1.0, -1.0, new Rotation2d()));
  // waypoints.add(new Pose2d(0.0, -1.0, new Rotation2d()));

  // waypoints.add(new Pose2d(0.0, 0.0, new Rotation2d(3.1)));
  // return new SequentialCommandGroup(
  // new DriveThroughTrajectory(m_driveSubsystem, new Pose2d(0,0, new
  // Rotation2d()), waypoints, 0.5, 0.8, 0.5, 0.5));

  // return new SequentialCommandGroup(new
  // DriveThroughTrajectory(m_driveSubsystem, new Pose2d(0,0,
  // new Rotation2d()), waypoints, 0.5, 0.8, 0.5, 0.5));
  // }

  /**
   * Uses AlignToAprilTag to align to an AprilTag
   * 
   * @param offset - the lateral distance away from the AprilTag the Robot should
   *               align to
   * @return A command that moves the robot to the correct alignment
   */
  public Command alignToAprilTag(double yOffset, double xOffset) {
    return new AlignToAprilTag(m_driveSubsystem, m_bling, m_frontCamera, 0.52, yOffset, xOffset);
  }

  /**
   * Sets the bling and underglow of the Robot on startup. Underglow to the color
   * of the alliance.
   * bling to green if shoulder angle initializes correctly and red if it doesn't.
   */
  public void setStartupLighting() {
    // Pick intensity based on driver station connection.
    
    double intensity = 0.3; // Default to dim.
    if (DriverStation.isDSAttached()) {
      intensity = 1.0; // Bright if attached.
    }
    // Set lighting to driver station aliance color.
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      m_underglow.setLEDIntensity(0, 0, intensity);
    } else {
      m_underglow.setLEDIntensity(intensity, 0, 0);
    }
    if (m_arm.getJointAngles().shoulder > 3.0) {
      m_bling.clearLEDs();
      m_bling.setColorRGBAll(255, 0, 0);

    } else if (m_arm.getJointAngles().shoulder < -3.0) {
      m_bling.clearLEDs();
      m_bling.setColorRGBAll(0, 255, 0);
    } else {
      m_bling.clearLEDs();
      m_bling.setColorRGBAll(0, 0, 255);
    }
  }
}