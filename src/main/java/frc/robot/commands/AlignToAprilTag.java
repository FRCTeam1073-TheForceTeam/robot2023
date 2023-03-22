// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AlignToAprilTag extends CommandBase {

  // Saved construction parameters:
  private DriveSubsystem drivetrain;
  private Bling bling;
  private AprilTagFinder finder;
  private double maxVelocity;
  private double maxAngularVelocity;
  private double linearTolerance;
  private double rotationalTolerance;



  // State variables for execution:
  int targetTagID;
  double targetTagDistance;
  private ChassisSpeeds chassisSpeeds;
  int glitchCounter;
  double yOffset;

  public AlignToAprilTag(DriveSubsystem drivetrain, Bling bling, AprilTagFinder finder, double maxVelocity, double yOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.bling = bling;
    this.maxVelocity = maxVelocity;
    this.finder = finder;
    this.maxAngularVelocity = 0.5;
    this.linearTolerance = 0.05;
    this.rotationalTolerance = 0.05;
    this.yOffset = yOffset;
    // Create these just once and reuse them in execute loops.
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  /**
   * The robot looks for the closest ID, and saves that ID
   * Also the Bling is cleared for later steps
   */
  @Override
  public void initialize() {
    int closestID = finder.getClosestID(); // Get the closest tag ID, will be -1 if there is no tracked tag.
    bling.clearLEDs();
    if(closestID != -1){
      System.out.println(String.format("AlignToAprilTag Initialized to Tag: %d", closestID));
      targetTagID = closestID;
    } else {
      targetTagID = -1;
      System.out.println("AlingToAprilTag Initialize Failed: No Tag In Sight!");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Sets the bling to white as a "headlight" to see the apriltag better
   * @param x - The x component of the translation.
   * @param y - The y component of the translation.
   * @param z - The z component of the translation.
   * @param trasnlation - The translational component of the pose.
   * @param rotation - The rotational component of the pose.
   * @param value - Value to clamp
   * @param low - The lower boundary to which to clamp value.
   * @param hight - The highest boundary to which to clamp value.
   * @return The translational component of the pose.
   * @return buffer length
   * @return The translational component of the pose.
   */
  @Override
  public void execute() {
    bling.setColorRGBAll(255, 255, 255);
    int closestID = finder.getClosestID();    // Closest tag ID from finder.
    Pose3d targetPose = finder.getClosestPose(); // Closest tag pose. (Can be NULL!)
    double currentHeading = drivetrain.getWrappedHeading() * (Math.PI / 180.0);
    //apply offset to target pose
    if (targetPose != null){
    targetPose = new Pose3d(new Translation3d(targetPose.getX(), targetPose.getY() + yOffset, targetPose.getZ()), targetPose.getRotation());
    }

    // If we get a pose and the closestID is the one we were targeting => drive towards alignment left/right.
    if (closestID == targetTagID && targetPose != null) {
      
      // Robot relative movement:
      chassisSpeeds.vxMetersPerSecond = 0.0;
      double rotationSpeed = (Math.PI - currentHeading) * 0.5;
      rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);
      // chassisSpeeds.omegaRadiansPerSecond = targetPose.getRotation().getZ() * 0.5; // Rotate such that Z rotation goes to zero.
      chassisSpeeds.omegaRadiansPerSecond = rotationSpeed;
      chassisSpeeds.vyMetersPerSecond = -targetPose.getTranslation().getY() * 1.0;   // Slide along such that Y offset goes to zero.
      chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -maxVelocity, maxVelocity);
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, 
        Rotation2d.fromDegrees(drivetrain.getHeading()));
      drivetrain.setChassisSpeeds(speeds);
      
      if(Math.abs(targetPose.getTranslation().getY()) < linearTolerance && Math.abs(Math.PI - currentHeading) < rotationalTolerance){
        targetTagID = -1;  // Stop tracking when we're close enough.
      }
      //resets glitch counter
      glitchCounter = 0;

    }

    //increases glitch counter to track 
    else{
      glitchCounter ++;
    }

  }

  // Called once the command ends or is interrupted.
  /**
   * Stops the robot
   * Clears all of the LEDs
   */
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds.vxMetersPerSecond = 0.0;
    chassisSpeeds.vyMetersPerSecond = 0.0;
    chassisSpeeds.omegaRadiansPerSecond = 0.0;
    drivetrain.setChassisSpeeds(chassisSpeeds); // Stop moving.
    bling.clearLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int closestID = finder.getClosestID();      // If closest ID is -1 or not the one we are tracking.
    //de-bouncing glitch "signal"
    //TODO: set glitch counter to 5 instead of 3
    if (targetTagID < 0 || glitchCounter > 3){

      System.out.println("AlignToAprilTag Finished.");
      
      return true;
    } 

    else{
      return false;
    }
  }
}
