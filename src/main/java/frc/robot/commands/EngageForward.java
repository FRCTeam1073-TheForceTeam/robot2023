// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EngageForward extends CommandBase {
  DriveSubsystem drivetrain;
  boolean inverted;
  double maxSpeed;
  double linearVelocity;

  /** Creates a new EngageForward. */
  public EngageForward(DriveSubsystem drivetrain, double maxSpeed, boolean inverted) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.maxSpeed = maxSpeed;
    this.inverted = inverted;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (inverted)
        {
            linearVelocity = -maxSpeed;
        }
        else
        {
            linearVelocity = maxSpeed;
        }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
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
  public boolean isFinished() {
    if (Math.abs(drivetrain.getPitch()) < 6) 
        {
            
            return true;
        }
        else
        {
            return false;
        }
  }
}
