// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EngageRateBalance extends CommandBase 
{
  DriveSubsystem drivetrain;
  boolean inverted;
  double maxSpeed;
  double linearVelocity;

  /** Creates a new EngageForward. */
  public EngageRateBalance(DriveSubsystem drivetrain, double maxSpeed, boolean inverted) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.maxSpeed = maxSpeed;
    this.inverted = inverted;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (inverted)
    {
      linearVelocity = -maxSpeed; // sets the direction engage goes
    }
    else
    {
      linearVelocity = maxSpeed;
    }
  }

  public static void initPreferences()
  {
    Preferences.initDouble("EngageDriveUp.maxSpeed", 0.7); // sets the speed the robot goes
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( // sets the speed
      linearVelocity,
      0, 
      0,
      Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if (Math.abs(drivetrain.getPitchRate()) > 25) // triggers when the robot reaches the halfway point
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
