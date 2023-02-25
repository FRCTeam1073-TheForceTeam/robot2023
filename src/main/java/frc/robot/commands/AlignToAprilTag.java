// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class AlignToAprilTag extends CommandBase {

  // Saved construction parameters:
  private DriveSubsystem drivetrain;
  private AprilTagFinder finder;
  private double maxVelocity;
  private double maxAngularVelocity;
  private double tolerance;


  // State variables for execution:
  int targetTagID;
  double targetTagDistance;
  private ChassisSpeeds chassisSpeeds;


  public AlignToAprilTag(DriveSubsystem drivetrain, AprilTagFinder finder, double maxVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.maxVelocity = maxVelocity;
    this.finder = finder;
    this.maxAngularVelocity = 0.5;
    this.tolerance = 0.05;
    // Create these just once and reuse them in execute loops.
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int closestID = finder.getClosestID(); // Get the closest tag ID, will be -1 if there is no tracked tag.
    if(closestID != -1){
      System.out.println(String.format("AlignToAprilTag Initialized to Tag: %d", closestID));
      targetTagID = closestID;
    } else {
      targetTagID = -1;
      System.out.println("AlingToAprilTag Initialize Failed: No Tag In Sight!");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    int closestID = finder.getClosestID();    // Closest tag ID from finder.
    Pose3d targetPose = finder.getClosestPose(); // Closest tag pose. (Can be NULL!)

    // If we get a pose and the closestID is the one we were targeting => drive towards alignment left/right.
    if (closestID == targetTagID && targetPose != null) {
      
      // Robot relative movement:
      chassisSpeeds.vxMetersPerSecond = 0.0;
      // chassisSpeeds.omegaRadiansPerSecond = targetPose.getRotation().getZ() * 0.5; // Rotate such that Z rotation goes to zero.
      chassisSpeeds.omegaRadiansPerSecond = 0.0; // For now..
      chassisSpeeds.vyMetersPerSecond = targetPose.getTranslation().getY() * 1.0;   // Slide along such that Y offset goes to zero.
      chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -maxVelocity, maxVelocity);
      drivetrain.setChassisSpeeds(chassisSpeeds);
      
      if(Math.abs(targetPose.getTranslation().getY()) < tolerance){
        targetTagID = -1;  // Stop tracking when we're close enough.
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds.vxMetersPerSecond = 0.0;
    chassisSpeeds.vyMetersPerSecond = 0.0;
    chassisSpeeds.omegaRadiansPerSecond = 0.0;
    drivetrain.setChassisSpeeds(chassisSpeeds); // Stop moving.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int closestID = finder.getClosestID();      // If closest ID is -1 or not the one we are tracking.
    if (targetTagID < 0 || closestID < 0 || closestID != targetTagID){
      System.out.println("AlignToAprilTag Finished.");
      return true;
    } else{
      return false;
    }
  }
}
