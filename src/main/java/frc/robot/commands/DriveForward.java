// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForward extends CommandBase {
  DriveSubsystem driveSubsystem;
  double startTime;
  double time;
  boolean inverted;
  /** Creates a new DriveForward. */
  public DriveForward(DriveSubsystem driveSubsystem, double time, boolean inverted){
    this.driveSubsystem = driveSubsystem;
    this.time = time;
    this.inverted = inverted;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    ChassisSpeeds speeds;
    if (!inverted){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(-0.25, 0.0, 0.0), new Rotation2d(driveSubsystem.getWrappedHeading()));
    }else{
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(0.25, 0.0, 0.0), new Rotation2d(driveSubsystem.getWrappedHeading()));
    }
    
    driveSubsystem.setChassisSpeeds(speeds);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, Rotation2d.fromDegrees(driveSubsystem.getHeading())));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime > time);
  }
}
