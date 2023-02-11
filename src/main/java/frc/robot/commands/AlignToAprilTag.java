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
  /** Creates a new AlignToAprilTag. */
  ArrayList<Integer> visibleTagIDs; 
  ChassisSpeeds chassisSpeeds;
  DriveSubsystem drivetrain;
  Pose2d robotPose;
  Pose2d tagPose;
  double maxVelocity;
  AprilTagFinder finder;
  double tolerance;
 // Pose3d difference;
  private NetworkTable apriltagConnect;
  private NetworkTableEntry apriltagConnectEntryID;
  private NetworkTableEntry apriltagConnectEntryY;
  private NetworkTableEntry apriltagConnectEntryYaw;
  int targetTagID;
  int targetTagY;
  int robotYaw;
  double maxAngularSpeed;
  double maxSpeed;
  double xSpeed;
  double ySpeed;
  double angularVelocity;
  int phase = 0;
  double initialHeading;

  //double maxRotationlVelocity;

  public AlignToAprilTag(DriveSubsystem drivetrain, double maxVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.maxVelocity = maxVelocity;
    finder = new AprilTagFinder(drivetrain);
    maxAngularSpeed = 0.5;
    addRequirements(drivetrain);
  }

 //   for(int i = 0; i < finder.getVisibleTags().size(); i++){
 //     visibleTagIDs.add(finder.getVisibleTags().get(i).getId());
 //   }
 //   addRequirements(drivetrain);

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    apriltagConnect = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    //if (AprilTagFinder.AnyAprilTagVisible() == true){
    apriltagConnectEntryID = apriltagConnect.getEntry("Closest ID");
    apriltagConnectEntryY = apriltagConnect.getEntry("Closest Y");
    apriltagConnectEntryY = apriltagConnect.getEntry("Tag Rotation Y");
    robotYaw = (int)apriltagConnectEntryYaw.getDouble(0.0);
    targetTagID = (int)apriltagConnectEntryID.getDouble(0.0);
    targetTagY = (int)apriltagConnectEntryY.getDouble(0.0);
    initialHeading = drivetrain.getHeading();
    phase = 0;
    //}else{
      //targetTagID = 0;
     // phase = 3;
    //}
    System.out.println("Initialize AlingToAprilTag");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double difference;
    double heading = drivetrain.getOdometry().getRotation().getDegrees();
    if (phase == 0){
      //lines the robot up parralel to the grid
      if (drivetrain.getHeading() > 0){
        difference = 180 - heading;
      }else{
        difference = -180 - heading;
      }

      if (Math.abs(difference) > .1){
        angularVelocity = - 0.8 * difference;
        angularVelocity = MathUtil.clamp(angularVelocity, -maxAngularSpeed, maxAngularSpeed);
        xSpeed = 0;
        ySpeed = 0;
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, angularVelocity, Rotation2d.fromDegrees(drivetrain.getHeading()));
      }else{
        phase = 1;
      }
    }
    
    if (phase == 1){
      //Moves sideways until the robot can detect the apriltag again
      if (initialHeading < 0){
        xSpeed = 0;
        ySpeed = .5;
        angularVelocity = 0;
        int currentTagID = (int)apriltagConnectEntryID.getDouble(0.0);
        if (currentTagID != targetTagID){
          ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, angularVelocity, Rotation2d.fromDegrees(drivetrain.getHeading()));
            drivetrain.setChassisSpeeds(speeds);
        }else{
          phase = 2;
        }
      }else{
        xSpeed = 0;
        ySpeed = -.5;
        angularVelocity = 0;
        int currentTagID = (int)apriltagConnectEntryID.getDouble(0.0);
        if (currentTagID != targetTagID){
          ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, angularVelocity, Rotation2d.fromDegrees(drivetrain.getHeading()));
            drivetrain.setChassisSpeeds(speeds);
        }else{
          phase = 2;
        }
      }
    }

    if(phase == 2){
      //Moves to the center of the tag
      double currentY = apriltagConnectEntryY.getDouble(0.0);
      if(Math.abs(currentY) > 0.05){  
        xSpeed = 0;
        ySpeed = -currentY * 0.5;
        angularVelocity = 0;
        ySpeed = MathUtil.clamp(ySpeed, -0.5, 0.5);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          0,ySpeed,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
        drivetrain.setChassisSpeeds(speeds);
      }else{
        phase = 3;
      }
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
    //if(difference.getY() < tolerance){
      //return true;
    //}
    return (phase == 3);
    //return false;
  }
}
