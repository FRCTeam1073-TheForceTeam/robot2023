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
import frc.robot.commands.DriveForward;
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
  //private static final String kBasicEngage = "Basic Engage";
  //private static final String kSplitEngage = "Split Engage";
  //private static final String kEngagePlus = "Engage Plus";
  //private static final String kLeaveCommunity = "Leave Community";
  private static final String kTestMode = "Test Mode";
  //private static final String kAlignToAprilTag = "Align To AprilTag";
  //private static final String kScoreCube = "Score Cube";
  private static final String kCenterArmCubeEngage = "CenterArmCubeEngage";
  private static final String kCenterArmCubeLeaveEngage = "CenterArmCubeLeaveEngage";
  //private static final String kCenterArmCubeLeave = "Score Cube and Leave Community";
  private static final String kCenterShootCubeLeaveEngage = "CenterShootCubeLeaveEngage";
  private static final String kBarrierShootCubeLeaveCollectCube = "BarrierShootCubeLeaveCollectCube";
  private static final String kBarrierShootCubeLeaveCollectCubeAlign = "Barrier Shoot Cube Leave Collect Cube(Align)";
  private static final String kProtectorShootCubeLeaveCollectCube = "ProtectorShootCubeLeaveCollectCube";
  private static final String kProtectorShootCubeLeaveCollectCubeAlign = "ProtectorShootCubeLeaveCollectCube(Align)";
  private static final String kCenterShootCubeLeaveCollectEngage = "CenterShootCubeLeaveCollectEngage";
  private static final String kCenterShootCubeLeaveCollectEngageAlign = "CenterShootCubeLeaveCollectEngage(Align)";
  private static final String kBarrierArmLinkConeCubeConeAlign = "BarrierArmLinkConeCubeCone(Align)";

  private static final String kArmTest = "ArmTest";
  private static final String kTrajectoryTest = "DriveTrajectoryTest";

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
    m_chooser.addOption("Center Arm Cube Engage", kCenterArmCubeEngage);
    m_chooser.addOption("Center Arm Cube Leave Engage", kCenterArmCubeLeaveEngage);
    m_chooser.addOption("Center Shoot Cube Leave Engage", kCenterShootCubeLeaveEngage);
    m_chooser.addOption("Barrier Shoot Cube Leave Collect Cube", kBarrierShootCubeLeaveCollectCube);
    m_chooser.addOption("Barrier Shoot Cube Leave Collect Cube(Align)", kBarrierShootCubeLeaveCollectCubeAlign);
    m_chooser.addOption("Protector Shoot Cube Leave Collect Cube", kProtectorShootCubeLeaveCollectCube);
    m_chooser.addOption("Protector Shoot Cube Leave Collect Cube(Align)", kProtectorShootCubeLeaveCollectCubeAlign);
    m_chooser.addOption("Center Shoot Cube Leave Collect Engage", kCenterShootCubeLeaveCollectEngage);
    m_chooser.addOption("CenterShootCubeLeaveCollectEngage(Align)", kCenterShootCubeLeaveCollectEngageAlign);
    m_chooser.addOption("Arm test", kArmTest);
    m_chooser.addOption("Drive Trajectory Test", kTrajectoryTest);
    m_chooser.addOption("Barrier Arm Link (Cone, Cube, Cone)(Align)", kBarrierArmLinkConeCubeConeAlign);

    SmartDashboard.putData("Auto Chooser", m_chooser);

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

      Trigger alternateStowTrigger = new Trigger(m_OI :: getOperatorDPadUp);
      alternateStowTrigger.onTrue(alternateArmStowCommand());

      Trigger armZeroCommand = new Trigger(m_OI::getOperatorDPadLeft);
      armZeroCommand.onTrue(zeroArm()); 
    }

    public Command zeroArm() {
      return new UpdateMotorEncoders(m_arm);
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
      case kTestMode:
        return testMode();
      case kArmTest:
        return armTest();
      case kCenterArmCubeEngage:
        return centerArmCubeEngageCommand();
      case kCenterArmCubeLeaveEngage:
        return centerArmCubeLeaveEngageCommand();
      case kTrajectoryTest:
        return testTrajectoryCommand();
      case kCenterShootCubeLeaveEngage:
        return centerShootCubeLeaveEngageCommand();
      case kBarrierShootCubeLeaveCollectCube:
        return barrierShootCubeLeaveCollectCube();
      case kBarrierShootCubeLeaveCollectCubeAlign:
        return barrierShootCubeLeaveCollectCubeAlign();
      case kProtectorShootCubeLeaveCollectCube:
        return protectorShootCubeLeaveCollectCube();
      case kProtectorShootCubeLeaveCollectCubeAlign:
        return protectorShootCubeLeaveCollectCubeAlign();
      case kCenterShootCubeLeaveCollectEngage:
        return centerShootCubeLeaveCollectEngage();
      case kCenterShootCubeLeaveCollectEngageAlign:
        return centerShootCubeLeaveCollectEngageAlign();
      case kBarrierArmLinkConeCubeConeAlign:
        return barrierArmLinkConeCubeCone();
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

    new PlannedArmPath(m_arm, m_pathPlanner, 0, velocity));
  }

  public Command alignToGamePiece(OI oi) {
    return new ConditionalCommand(
      new AlignToGamePiece(m_driveSubsystem, m_bling, m_gamePieceFinder, 0.2, true, 10, 8),
      new AlignToGamePiece(m_driveSubsystem, m_bling, m_gamePieceFinder, 0.2, false, 10, 8),
      oi::isCubeMode);
  }

  public Command alternateArmStowCommand(){
    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);
    return new PlannedArmPath(m_arm, m_pathPlanner, 9, velocity);
  }

  public Command groundPickupCommand(OI oi) {
    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    // cubeWaypoints.add(m_arm.new JointWaypoints(-1.78, 3.98, -1.19, 2.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-1.05, 5.38, -1.21, 4.0));


    ArrayList<Arm.JointWaypoints> coneWaypoints = new ArrayList<Arm.JointWaypoints>();
    coneWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    // coneWaypoints.add(m_arm.new JointWaypoints(-1.78, 3.98, 0.0, 2.0));
    coneWaypoints.add(m_arm.new JointWaypoints(-1.63, 4.66, 0.61, 4.0));
    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    // return new ConditionalCommand(
    //     new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
    //     new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
    //     oi::isCubeMode);

    return new ConditionalCommand(
        new PlannedArmPath(m_arm, m_pathPlanner, 7, velocity),
        new PlannedArmPath(m_arm, m_pathPlanner, 8, velocity),
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

    // return new ConditionalCommand(
    //     new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
    //     new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
    //     oi::isCubeMode);

    return new ConditionalCommand(
        new PlannedArmPath(m_arm, m_pathPlanner, 5, velocity),
        new PlannedArmPath(m_arm, m_pathPlanner, 6, velocity),
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

    // return new ConditionalCommand(
    //     new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
    //     new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
    //     oi::isCubeMode);

    return new ConditionalCommand(
        new PlannedArmPath(m_arm, m_pathPlanner, 2, velocity),
        new PlannedArmPath(m_arm, m_pathPlanner, 4, velocity),
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

    // return new ConditionalCommand(
    //   new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
    //   new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
    //   oi :: isCubeMode);

    return new ConditionalCommand(
        new PlannedArmPath(m_arm, m_pathPlanner, 1, velocity),
        new PlannedArmPath(m_arm, m_pathPlanner, 3, velocity),
        oi::isCubeMode);
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

    // return new ConditionalCommand(
    //     new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0),
    //     new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0),
    //     oi::isCubeMode);

    return new ConditionalCommand(
        new PlannedArmPath(m_arm, m_pathPlanner, 10, velocity),
        new PlannedArmPath(m_arm, m_pathPlanner, 11, velocity),
        oi::isCubeMode);
  }

  /**
   * Autonomous command that aligns to april tag, scores a cube in high node, and
   * then engages on the charge station.
   * 
   * @return Command to do the autonomous outlined above
   */
  public Command centerArmCubeEngageCommand() {
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
                new ParkingBrake(m_driveSubsystem, m_bling, false))));
  }

  /**
   * Autonomous command that scores a cube in high node, leaves the community, and
   * then engages on the charge station.
   * 
   * @return Command to do the autonomous outlined above
   */
  public Command centerShootCubeLeaveEngageCommand() {
    ArrayList<Pose2d> communityWaypoints = new ArrayList<Pose2d>();
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      communityWaypoints.add(new Pose2d(2.5, 0, new Rotation2d(3.14)));
    }
    else{
      communityWaypoints.add(new Pose2d(2.5, 0, new Rotation2d(3.14)));
    }

    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new SequentialCommandGroup(
            new DriveThroughTrajectory(m_driveSubsystem, communityWaypoints, 0.8,
                1.0, 0.5, 0.5),
            new EngageDriveUp(m_driveSubsystem, Preferences.getDouble("EngageDriveUp.maxSpeed", 0.9), true),
            new EngageForward(m_driveSubsystem, Preferences.getDouble("EngageForward.maxSpeed", 0.7), true),
            new EngageBalance(m_driveSubsystem, Preferences.getDouble("EngageBalance.maxSpeed", 0.7), true),
            new ParkingBrake(m_driveSubsystem, m_bling, false)));
  }

  public Command barrierShootCubeLeaveCollectCube() {
    ArrayList<Pose2d> leaveWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      leaveWaypoints.add(new Pose2d(1.3, -.3, new Rotation2d(Math.PI)));
      leaveWaypoints.add(new Pose2d(1.5, -0.1, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.58, -0.05, new Rotation2d(0)));
    } else {
      leaveWaypoints.add(new Pose2d(1.3, .3, new Rotation2d(Math.PI)));
      leaveWaypoints.add(new Pose2d(1.5, 0.1, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.58, 0.07, new Rotation2d(0)));
    }

    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-0.87, 5.348, -1.19, 4.0));


    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.9, 0.9, 0.9);

    ArrayList<Pose2d> returnWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      returnWaypoints.add(new Pose2d(2, -0.1, new Rotation2d(0)));
      returnWaypoints.add(new Pose2d(1.3, -.2, new Rotation2d(Math.PI)));
      returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));  
    } else {
        returnWaypoints.add(new Pose2d(2, 0.1, new Rotation2d(0)));
        returnWaypoints.add(new Pose2d(1.3, .3, new Rotation2d(Math.PI)));
        returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));  
    }


    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new ParallelDeadlineGroup(
          new DriveThroughTrajectory(m_driveSubsystem, leaveWaypoints, 1.5, 1.2, 0.5, 0.7),
          new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0.5),
          new CollectCommand(m_claw, true, 12)),
        new ParallelDeadlineGroup(
            new DriveThroughTrajectory(m_driveSubsystem, returnWaypoints, 1.5
            , 1.2, 0.5, 0.7),
            armStowCommand(m_OI)),
        new ParallelDeadlineGroup(
          midScoreCommand(m_OI),
          alignToAprilTag(-0.04, 0.6)),
        //new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        armStowCommand(m_OI));
  }

  public Command barrierShootCubeLeaveCollectCubeAlign() {
    ArrayList<Pose2d> leaveWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      leaveWaypoints.add(new Pose2d(1.3, -.3, new Rotation2d(Math.PI)));
      leaveWaypoints.add(new Pose2d(1.5, -0.1, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.4, -.25, new Rotation2d(0)));
    } else {
      leaveWaypoints.add(new Pose2d(1.3, .3, new Rotation2d(Math.PI)));
      leaveWaypoints.add(new Pose2d(1.5, 0.1, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.4, 0.25, new Rotation2d(0)));
    }

    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-0.87, 5.348, -1.19, 4.0));


    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.9, 0.9, 0.9);

    ArrayList<Pose2d> returnWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      returnWaypoints.add(new Pose2d(2, -0.1, new Rotation2d(0)));
      returnWaypoints.add(new Pose2d(1.3, -.3, new Rotation2d(Math.PI)));
      returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));  
    } else {
        returnWaypoints.add(new Pose2d(2, 0.1, new Rotation2d(0)));
        returnWaypoints.add(new Pose2d(1.3, .3, new Rotation2d(Math.PI)));
        returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));  
    }


    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new DriveThroughTrajectory(m_driveSubsystem, leaveWaypoints, 1.5, 1.5, 0.5, 0.7),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new WaitCommand(1),
            new ArmSplinePosition(m_arm, cubeWaypoints, velocity, 0.5)),
          new AlignToGamePiece(m_driveSubsystem, m_bling, m_gamePieceFinder, 0.2, true, 10, 8),
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

  public Command barrierArmLinkConeCubeCone(){
    ArrayList<Arm.JointWaypoints> coneWaypoints = new ArrayList<Arm.JointWaypoints>();
    coneWaypoints.add(m_arm.new JointWaypoints(-1.118, 3.101, 1.25, 1.0));


    ArrayList<Arm.JointWaypoints> cubePickupWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubePickupWaypoints.add(m_arm.new JointWaypoints(-0.87, 5.348, -1.19, 1.0));

    ArrayList<Pose2d> collectWaypoints = new ArrayList<Pose2d>();
    collectWaypoints.add(new Pose2d(0.222, 0.324, new Rotation2d(Math.PI)));
    collectWaypoints.add(new Pose2d(0.522, 0.324, new Rotation2d(0)));

    ArrayList<Pose2d> scoreCubeWaypoints = new ArrayList<Pose2d>();
    scoreCubeWaypoints.add(new Pose2d(0.05, 0.324, new Rotation2d(Math.PI)));
    
    ArrayList<Pose2d> conePickupWaypoints = new ArrayList<Pose2d>();
    conePickupWaypoints.add(new Pose2d(0.244, -0.1277, new Rotation2d(Math.PI)));
    conePickupWaypoints.add(new Pose2d(0.7250, -0.1277, new Rotation2d(0)));

    ArrayList<Arm.JointWaypoints> coneArmPickupWaypoints = new ArrayList<Arm.JointWaypoints>();
    coneArmPickupWaypoints.add(m_arm.new JointWaypoints(-1.5859, 4.5584, 0.5278, 1.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.5, 0.5, 0.5);

    return new SequentialCommandGroup(
      new ArmSplinePosition(m_arm, coneWaypoints, velocity, 0.5),
      new WaitCommand(2),
      new DepositCommand(m_claw, false, 1),
      armStowCommand(m_OI), 
      new DriveThroughTrajectory(m_driveSubsystem, collectWaypoints, 0.5, 0.5, 0.5, 0.7),
      new AlignToGamePiece(m_driveSubsystem, m_bling, m_gamePieceFinder, 0.2, true, 9, 9),
      new ArmSplinePosition(m_arm, cubePickupWaypoints, velocity, 0.5),
      new ParallelCommandGroup(
        new DriveForward(m_driveSubsystem, 0.1, false),
        new CollectCommand(m_claw, true, 1.5)),
      armStowCommand(m_OI),
      new DriveThroughTrajectory(m_driveSubsystem, scoreCubeWaypoints, 0.5, 0.5, 0.5, 0.7),
      new ParallelDeadlineGroup(
        new WaitCommand(0.8),
        alignToAprilTag(-0.09, 0.8)),
      highScoreCommand(m_OI),
      new DepositCommand(m_claw, true, 0.5),
      armStowCommand(m_OI),
      new WaitCommand(0.5),
      new DriveThroughTrajectory(m_driveSubsystem, conePickupWaypoints, 0.5, 0.5, 0.5, 0.7),
      new ArmSplinePosition(m_arm, coneArmPickupWaypoints, velocity, 0.5),
      new CollectCommand(m_claw, false, 1.5),
      armStowCommand(m_OI)
    );  
  }


  public Command protectorShootCubeLeaveCollectCube(){
    ArrayList<Pose2d> leaveWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      leaveWaypoints.add(new Pose2d(1.986, 0.2137, new Rotation2d(Math.PI)));
      leaveWaypoints.add(new Pose2d(2.22, 0.15, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.7, 0.15, new Rotation2d(0)));
    } else {
      leaveWaypoints.add(new Pose2d(1.986, -0.2137, new Rotation2d(Math.PI)));
      leaveWaypoints.add(new Pose2d(2.22, -0.15, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.7, -0.15, new Rotation2d(0)));
    }

    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-0.87, 5.348, -1.19, 4.0));


    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    ArrayList<Pose2d> returnWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      returnWaypoints.add(new Pose2d(2, 0.1, new Rotation2d(Math.PI)));
      returnWaypoints.add(new Pose2d(1.986, 0.2137, new Rotation2d(Math.PI)));
      returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));

    } else {
      returnWaypoints.add(new Pose2d(2, -0.1, new Rotation2d(Math.PI)));
      returnWaypoints.add(new Pose2d(1.986, -0.2137, new Rotation2d(Math.PI)));
      returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));
    }


    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new ParallelDeadlineGroup(
          new DriveThroughTrajectory(m_driveSubsystem, leaveWaypoints, 2, 2, 0.5, 0.7),
          new SequentialCommandGroup(
            new WaitCommand(2.0),
            new ArmSplinePosition(m_arm, cubeWaypoints, velocity, .5))),
        new CollectCommand(m_claw, true, 1),
        new SequentialCommandGroup(
          armStowCommand(m_OI),
          new DriveThroughTrajectory(m_driveSubsystem, returnWaypoints, 2, 2, 0.5, 0.7)),
        new ParallelDeadlineGroup(
          midScoreCommand(m_OI),
          alignToAprilTag(-0.04, 0.6)),
        new DepositCommand(m_claw, true, 0.5),
        armStowCommand(m_OI));
        //TODO: need to speed up so can fit in 15 sec
  }

  public Command protectorShootCubeLeaveCollectCubeAlign(){
    ArrayList<Pose2d> leaveWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      leaveWaypoints.add(new Pose2d(1.986, 0.2137, new Rotation2d(Math.PI/2)));
      leaveWaypoints.add(new Pose2d(2.22, 0.15, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.6, 0.15, new Rotation2d(0)));
    } else {
      leaveWaypoints.add(new Pose2d(1.986, -0.2137, new Rotation2d(Math.PI/2)));
      leaveWaypoints.add(new Pose2d(2.22, -0.15, new Rotation2d(0)));
      leaveWaypoints.add(new Pose2d(2.6, -0.15, new Rotation2d(0)));
    }

    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-0.87, 5.348, -1.19, 4.0));


    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    ArrayList<Pose2d> returnWaypoints = new ArrayList<Pose2d>();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      returnWaypoints.add(new Pose2d(2, 0.1, new Rotation2d(Math.PI/2)));
      returnWaypoints.add(new Pose2d(1.986, 0.2137, new Rotation2d(Math.PI/2)));
      returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));

    } else {
      returnWaypoints.add(new Pose2d(2, -0.1, new Rotation2d(Math.PI/2)));
      returnWaypoints.add(new Pose2d(1.986, -0.2137, new Rotation2d(Math.PI/2)));
      returnWaypoints.add(new Pose2d(0, 0, new Rotation2d(Math.PI)));
    }


    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new DriveThroughTrajectory(m_driveSubsystem, leaveWaypoints, 1.5, 1.5, 0.5, 0.7),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new ArmSplinePosition(m_arm, cubeWaypoints, velocity, .5),
              new WaitCommand(1.0)),
            new AlignToGamePiece(m_driveSubsystem, m_bling, m_gamePieceFinder, 0.2, true, 10, 8),
            new CollectCommand(m_claw, true, 12)),
        new SequentialCommandGroup(
          armStowCommand(m_OI),
          new DriveThroughTrajectory(m_driveSubsystem, returnWaypoints, 1.5, 1.5, 0.5, 0.7)),
        new ParallelDeadlineGroup(
          midScoreCommand(m_OI),
          alignToAprilTag(-0.04, 0.6)),
        new DepositCommand(m_claw, true, 0.5),
        armStowCommand(m_OI));
        //TODO: need to speed up so can fit in 15 sec
  }

  public Command centerShootCubeLeaveCollectEngage(){
    ArrayList<Pose2d> chargeStationWaypoints = new ArrayList<Pose2d>();
    chargeStationWaypoints.add(new Pose2d(1.7579, 0.0116, new Rotation2d(Math.PI)));

    ArrayList<Pose2d>  collectWaypoints = new ArrayList<Pose2d>();
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      collectWaypoints.add(new Pose2d(1.95,-0.096, new Rotation2d(Math.PI)));
      //collectWaypoints.add(new Pose2d(2.48,0.269, new Rotation2d(0)));
      collectWaypoints.add(new Pose2d(2.5, 0.50, new Rotation2d(0)));
      collectWaypoints.add(new Pose2d(2.9,0.50, new Rotation2d(0)));  
    }
    else{
      collectWaypoints.add(new Pose2d(1.95,0.096, new Rotation2d(Math.PI)));
      //collectWaypoints.add(new Pose2d(2.48,-0.269, new Rotation2d(0)));
      collectWaypoints.add(new Pose2d(2.5, -0.50, new Rotation2d(0)));
      collectWaypoints.add(new Pose2d(2.9,-0.50, new Rotation2d(0)));  
    }
   
    ArrayList<Pose2d> returnWaypoints = new ArrayList<Pose2d>();
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
    {
      returnWaypoints.add(new Pose2d(2.4, -0.4, new Rotation2d(Math.PI / 2)));
      returnWaypoints.add(new Pose2d(2.0,-0.4, new Rotation2d(Math.PI)));
    }
    else{
      returnWaypoints.add(new Pose2d(2.4, 0.4, new Rotation2d(Math.PI / 2)));
      returnWaypoints.add(new Pose2d(2.0, 0.4, new Rotation2d(Math.PI)));
    }


    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-0.92, 5.348, -1.19, 4.0));


    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new SequentialCommandGroup(
            new DriveThroughTrajectory(m_driveSubsystem, chargeStationWaypoints, 2.4,2.0, 0.5, 0.5),
            new WaitCommand(0.1),
            new ParallelDeadlineGroup(
              new DriveThroughTrajectory(m_driveSubsystem, collectWaypoints, 2.4, 2.0, 0.5, 0.5),
              new ArmSplinePosition(m_arm, cubeWaypoints, velocity, .5),
              new CollectCommand(m_claw, true, 5.25)),
            new WaitCommand(0.25),
            new ParallelDeadlineGroup(
              armStowCommand(m_OI),
              new DriveThroughTrajectory(m_driveSubsystem, returnWaypoints, 2.2, 2.0, 0.5, 0.5),
              new CollectCommand(m_claw, true, 2.0)),
            new EngageDriveUp(m_driveSubsystem, 0.3, true),
            new EngageForward(m_driveSubsystem, 0.2, true),
            new CollectCommand(m_claw, true, 0.25),
            new EngageBalance(m_driveSubsystem, Preferences.getDouble("EngageBalance.maxSpeed", 0.7), true),
            new ParkingBrake(m_driveSubsystem, m_bling, false)));
  }

  public Command centerShootCubeLeaveCollectEngageAlign(){
    ArrayList<Pose2d> chargeStationWaypoints = new ArrayList<Pose2d>();
    chargeStationWaypoints.add(new Pose2d(1.7579, 0.0116, new Rotation2d(Math.PI)));

    ArrayList<Pose2d>  collectWaypoints = new ArrayList<Pose2d>();
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      collectWaypoints.add(new Pose2d(1.95,-0.096, new Rotation2d(Math.PI)));
      collectWaypoints.add(new Pose2d(2.48,0.269, new Rotation2d(0)));
      collectWaypoints.add(new Pose2d(2.5, 0.50, new Rotation2d(0)));
      //collectWaypoints.add(new Pose2d(2.9,0.50, new Rotation2d(0)));  
    }
    else{
      collectWaypoints.add(new Pose2d(1.95,0.096, new Rotation2d(Math.PI)));
      collectWaypoints.add(new Pose2d(2.48,-0.269, new Rotation2d(0)));
      collectWaypoints.add(new Pose2d(2.5, -0.50, new Rotation2d(0)));
      //collectWaypoints.add(new Pose2d(2.9,-0.50, new Rotation2d(0)));  
    }
   
    ArrayList<Pose2d> returnWaypoints = new ArrayList<Pose2d>();
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
    {
      returnWaypoints.add(new Pose2d(2.4, -0.4, new Rotation2d((Math.PI / 2))));
      returnWaypoints.add(new Pose2d(2.0,-0.4, new Rotation2d(3 *Math.PI /  4)));
    }
    else{
      returnWaypoints.add(new Pose2d(2.4, 0.4, new Rotation2d((Math.PI) / 2)));
      returnWaypoints.add(new Pose2d(2.0, 0.4, new Rotation2d(3 * Math.PI / 4)));
    }


    ArrayList<Arm.JointWaypoints> cubeWaypoints = new ArrayList<Arm.JointWaypoints>();
    cubeWaypoints.add(m_arm.new JointWaypoints(-3.23, 3.375, -1.2, 1.0));
    cubeWaypoints.add(m_arm.new JointWaypoints(-0.87, 5.348, -1.19, 4.0));


    Arm.JointVelocities velocity = m_arm.new JointVelocities(1.4, 1.4, 1.4);

    return new SequentialCommandGroup(
        new CollectCommand(m_claw, true, 0.5),
        new DepositCommand(m_claw, true, 0.5),
        new SequentialCommandGroup(
            new DriveThroughTrajectory(m_driveSubsystem, chargeStationWaypoints, 2.0,2.0, 0.5, 0.5),
            new WaitCommand(0.1),
            new DriveThroughTrajectory(m_driveSubsystem, collectWaypoints, 2.0, 2.0, 0.5, 0.5),
            new ParallelDeadlineGroup(
              new ArmSplinePosition(m_arm, cubeWaypoints, velocity, .5),
              new AlignToGamePiece(m_driveSubsystem, m_bling, m_gamePieceFinder, 0.2, true, 10, 8),
              new CollectCommand(m_claw, true, 6)),
            new WaitCommand(0.25),
            new ParallelDeadlineGroup(
              armStowCommand(m_OI),
              new DriveThroughTrajectory(m_driveSubsystem, returnWaypoints, 2.0, 2.0, 0.5, 0.5),
              new CollectCommand(m_claw, true, 2.0)),
            new EngageDriveUp(m_driveSubsystem, Preferences.getDouble("EngageDriveUp.maxSpeed", 0.9), true),
            new EngageForward(m_driveSubsystem, Preferences.getDouble("EngageForward.maxSpeed", 0.7), true),
            new CollectCommand(m_claw, true, 0.25),
            new EngageBalance(m_driveSubsystem, Preferences.getDouble("EngageBalance.maxSpeed", 0.7), true),
            new ParkingBrake(m_driveSubsystem, m_bling, false)));
  }

  /**
   * Autonomous command that aligns to april tags, scores a cube, leaves
   * community, and then comes back to engage.
   * 
   * @return Command to do the autonomous outlined above
   */
  public Command centerArmCubeLeaveEngageCommand() {
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
        new ParkingBrake(m_driveSubsystem, m_bling, false));

  }

  /**
   * Tests ArmSetPosition
   * 
   * @return an ArmSetPosition Command
   */
  public Command armTest() {
    ArrayList<Arm.JointWaypoints> waypoints = new ArrayList<Arm.JointWaypoints>();
    waypoints.add(m_arm.new JointWaypoints(-1.0, 2.6, 0.0, 6.0));
    waypoints.add(m_arm.new JointWaypoints(-1.55, 2.4, 0.0, 9.0));

    Arm.JointVelocities velocity = m_arm.new JointVelocities(0.6, 0.6, 0.6);
    return new ArmSplinePosition(m_arm, waypoints, velocity, 0.5);
  }

  public Command testTrajectoryCommand() {
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
  /**
   * Uses AlignToAprilTag to align to an AprilTag
   * 
   * @param offset - the lateral distance away from the AprilTag the Robot should
   *               align to
   * @return A command that moves the robot to the correct alignment
   */
  public Command alignToAprilTag(double yOffset, double xOffset) {
    return new AlignToAprilTag(m_driveSubsystem, m_bling, m_frontCamera, m_claw, 0.52, yOffset, xOffset, m_OI.isCubeMode(), 1.7);
  }

  /**
   * Sets the bling and underglow of the Robot on startup. Underglow to the color
   * of the alliance.
   * bling to green if shoulder angle initializes correctly and red if it doesn't.
   */
  public void setStartupLighting() {
    // Pick intensity based on driver station connection.
    
    double intensity = 0.3; // Default to dim.
    new WaitCommand(30);

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