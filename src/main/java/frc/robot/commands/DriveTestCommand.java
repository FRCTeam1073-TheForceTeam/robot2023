// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OI;

public class DriveTestCommand extends CommandBase 
{
  DriveSubsystem m_driveSubsystem;
  OI m_OI;

  // Teleop drive velocity scaling:
  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second

  /** Creates a new Teleop. */
  public DriveTestCommand(DriveSubsystem ds, OI oi){
    super.setName("DriveTestCommand");
    m_driveSubsystem = ds;
    m_OI = oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_driveSubsystem.setDebugMode(true);
    System.out.println("DriveTestCommand initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(m_OI.getOperatorAButton()) {
      m_driveSubsystem.setDebugDrivePower(0.5);
      m_driveSubsystem.setDebugAngle(0);
      System.out.println("Button A pressed");
    }
    else {
      m_driveSubsystem.setDebugDrivePower(0);
      m_driveSubsystem.setDebugAngle(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveTestCommand ended.");
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);
    m_driveSubsystem.setChassisSpeeds(chassisSpeeds); 
    m_driveSubsystem.setDebugMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
