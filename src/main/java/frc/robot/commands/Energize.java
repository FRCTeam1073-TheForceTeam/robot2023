// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Energize extends CommandBase {
  /** Creates a new Energize. */

  int drivePhase; //start, drive, climb stop
  double distanceTolerance = 0.1;
  double angleTolerance = 0.05;
  boolean forward; //camera faces april tag

  DriveSubsystem drivetrain;
  ChassisSpeeds chassisSpeeds;
  Pose2d targetPose;
  Pose2d robotPose;
  private double maxLinearVelocity;   // Meters/second
  

  public Energize(DriveSubsystem ds, double maxSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = ds;
    maxLinearVelocity = maxSpeed;
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePhase = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (drivePhase == 0) {
    robotPose = drivetrain.getOdometry();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        maxLinearVelocity,
        0, 
        0,
        Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
      drivetrain.setChassisSpeeds(speeds);
    //double yVelocity = - 0.8 * difference.getY();
    
    if (drivetrain.getPitch() > 10) {
      drivePhase = 1;
    }
  } //end of phase 0
  if (drivePhase == 1) {
    robotPose = drivetrain.getOdometry();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        maxLinearVelocity,
        0, 
        0,
        Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
      drivetrain.setChassisSpeeds(speeds);
      System.out.println("DrivePhase 1");
    //double yVelocity = - 0.8 * difference.getY();

    if (drivetrain.getPitch() < 15 && drivePhase == 1) {
      drivePhase = 2;
      ChassisSpeeds speeds2 = new ChassisSpeeds(-maxLinearVelocity * 0.5, 0, 0); // go backwards slightly
      drivetrain.setChassisSpeeds(speeds2);
    }
    
    if (drivePhase == 2)
    {
      //ChassisSpeeds speeds3 = new ChassisSpeeds(0, 0, 0);
      //drivetrain.setChassisSpeeds(speeds3);
      if (Math.abs(drivetrain.getPitch()) < 6)
      {
        System.out.println("DrivePhase 2");
        drivePhase = 3;
      }
    }
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    if (drivePhase == 3) 
    {
      System.out.println("DrivePhase 3");
      drivetrain.parkingBrake();

      return true;
    }
    else {
      return false;
    }
  }
}
