// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
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
  /** Creates a new AlignToAprilTag. */
  int id;
  ArrayList<Integer> visibleTagIDs; 
  ChassisSpeeds chassisSpeeds;
  DriveSubsystem drivetrain;
  Pose2d robotPose;
  Pose2d tagPose;
  double maxVelocity;
  AprilTagFinder finder;
  double tolerance;
  Pose3d difference;
  private NetworkTable apriltagConnect;
  private NetworkTableEntry apriltagConnectEntry;

  //double maxRotationlVelocity;

  public AlignToAprilTag(int aprilTagID, DriveSubsystem drivetrain, double maxVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    id = aprilTagID;
    this.drivetrain = drivetrain;
    this.maxVelocity = maxVelocity;
    finder = new AprilTagFinder(drivetrain);
    apriltagConnect = NetworkTableInstance.getDefault().getTable("Vision");
    apriltagConnectEntry = apriltagConnect.getEntry("Closest ID");

 //   for(int i = 0; i < finder.getVisibleTags().size(); i++){
 //     visibleTagIDs.add(finder.getVisibleTags().get(i).getId());
 //   }
 //   addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(visibleTagIDs.contains(id)){
      difference = finder.getRelativePose(id);
      double yVelocity = 0.5 * difference.getY();
      if(yVelocity > maxVelocity){
        yVelocity = maxVelocity;
      }
      if(yVelocity < -maxVelocity){
        yVelocity = -maxVelocity;
      }

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, yVelocity, 0, 
      Rotation2d.fromDegrees(drivetrain.getHeading()));
    }
  
    else{
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, 
      Rotation2d.fromDegrees(drivetrain.getHeading()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, 
      Rotation2d.fromDegrees(drivetrain.getHeading()));
      drivetrain.resetOdometry(new Pose2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(difference.getY() < tolerance){
      return true;
    }
    return false;
  }
}
