// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class Engage extends Command 
{
  /** Creates a new Engage. */
  PersistentParkingBrake brake;
  int drivePhase; //start, drive, climb, stop
  double distanceTolerance = 0.1;
  double angleTolerance = 0.05;
  boolean forward; //camera faces april tag
  boolean inverted;

  DriveSubsystem drivetrain;
  ChassisSpeeds chassisSpeeds;
  Pose2d targetPose;
  Pose2d robotPose;
  private double maxLinearVelocity;   // Meters/second
  private double linearVelocity;
  private double startTime;
  
  /*
   * 
   * This command is not currently used
   * Instead of this command, use a combination of EngageDriveUp at the beginning,
   * ParkingBrake at the end, and some other Engage files, such as EngageForward and EngageBalance.
   * 
   */

  public Engage(DriveSubsystem ds, double maxSpeed, boolean inverted)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = ds;
    maxLinearVelocity = maxSpeed;
    this.inverted = inverted;
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    drivePhase = 0; //starts  moving
    brake = new PersistentParkingBrake(drivetrain);
    if (inverted)
    {
      linearVelocity = -maxLinearVelocity;
    }
    else
    {
      linearVelocity = maxLinearVelocity;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Drive Phase", drivePhase);
    if (drivePhase == 0) 
    {
      robotPose = drivetrain.getOdometry();
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          linearVelocity,
          0, 
          0,
          Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
        drivetrain.setChassisSpeeds(chassisSpeeds);
      
      if (Math.abs(drivetrain.getPitch()) > 13) 
      {
        drivePhase = 1; //starts climbing
      }
    } //end of phase 0

    if (drivePhase == 1)
    {
      if (Math.abs(drivetrain.getPitch()) < 5) 
      {
        drivePhase = 2;
        startTime = Timer.getFPGATimestamp();

      }
    }

    if(drivePhase == 2)
    {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity * -0.5,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
      drivetrain.setChassisSpeeds(chassisSpeeds);
      //drivetrain.parkingBrake();
      if (Timer.getFPGATimestamp() > startTime + 0.5 && Math.abs(drivetrain.getPitch()) < 2)
      {
        drivePhase = 3;
      }
      //brake.execute();
    }
    if (drivePhase == 3)
    {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, Rotation2d.fromDegrees((drivetrain.getHeading())));
      drivetrain.setChassisSpeeds(chassisSpeeds);
      drivetrain.parkingBrake(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.parkingBrake(false);
    // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, Rotation2d.fromDegrees((drivetrain.getHeading())));
    // drivetrain.parkingBrake();
  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
