// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveToPoint extends CommandBase {
  /** Creates a new DriveToPoint. */
  
  double distanceTolerance = 0.1;
  double angleTolerance = 0.05;
  // double distance;
  double velocity;

  Pose2d robotPose;
  Pose2d targetPose;
  DriveSubsystem drivetrain;
  ChassisSpeeds chassisSpeeds;
  private final static double maximumLinearVelocity = .5;   // Meters/second
  private final static double maximumRotationVelocity = .5; // Radians/second
  private final static double minimumLinearVelocity = -.5;
  private final static double minimumRotationVelocity = -.5;

  public DriveToPoint(DriveSubsystem ds, Pose2d targetPose2d) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = ds;
    this.targetPose = targetPose2d;
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Called every time the scheduler runs while the command is scheduled.
    System.out.println("DriveToPoint");
 }

  @Override
  public void execute() {
    robotPose = drivetrain.getOdometry();
    Transform2d difference = targetPose.minus(robotPose);
    double xVelocity = 0.8 * difference.getX();
    double yVelocity = 0.8 * difference.getY();
    double angularVelocity = 0.8 * difference.getRotation().getRadians();
    if(xVelocity > maximumLinearVelocity){
      xVelocity = maximumLinearVelocity;
    }
    if(xVelocity < minimumLinearVelocity){
      xVelocity = minimumLinearVelocity;
    }
    if(yVelocity > maximumLinearVelocity){
      yVelocity = maximumLinearVelocity;
    }
    if(yVelocity < minimumLinearVelocity){
      yVelocity = minimumLinearVelocity;
    }
    if(angularVelocity > maximumRotationVelocity){
      angularVelocity = maximumRotationVelocity;
    } 
    if(angularVelocity < minimumRotationVelocity){
      angularVelocity = minimumRotationVelocity;
    }
    
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        -xVelocity, -yVelocity, -angularVelocity,
        Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
    drivetrain.setChassisSpeeds(speeds);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setBrakes(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var error = targetPose.minus(robotPose);
    if (error.getTranslation().getNorm()< distanceTolerance && error.getRotation().getRadians() < angleTolerance) {
      System.out.println("DriveToPoint Is Finished");
      return true;
    }
    return false;
  }
}
