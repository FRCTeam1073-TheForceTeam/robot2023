// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleConfig;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.OpenMV;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final OI m_OI = new OI();
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_driveSubsystem, m_OI);
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

    // Initialize Preferences For Command Classes:

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}