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

  Pose2d robotPose;
  Pose2d targetPose;
  DriveSubsystem drivetrain;
  ChassisSpeeds chassisSpeeds;
  private double maxLinearVelocity;   // Meters/second
  private double maxRotationVelocity; // Radians/second

  /**DriveToPoint constuctor.
   * 
   * @param ds  variable for the drive subsystem
   * @param targetPose2d  the target position on the field
   * @param maxSpeed  maximum speed of the robot
   * @param maxRotate maximum rotation of the robot
   */
  public DriveToPoint(DriveSubsystem ds, Pose2d targetPose2d, double maxSpeed, double maxRotate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = ds;
    this.targetPose = targetPose2d;
    maxLinearVelocity = maxSpeed;
    maxRotationVelocity = maxRotate;
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Called every time the scheduler runs while the command is scheduled.
    System.out.println("DriveToPoint");
 }

  //Sets the speed and rotation proportional to the difference in current robot position and target position.
  @Override
  public void execute() {
    robotPose = drivetrain.getOdometry();
    Transform2d difference = targetPose.minus(robotPose);
    double xVelocity = - 0.8 * difference.getX();
    double yVelocity = - 0.8 * difference.getY();
    double angularVelocity = - 0.8 * difference.getRotation().getRadians();
    //tests if velocities are within the maximum and sets them to the max if they exceed
    if(xVelocity > maxLinearVelocity){
      xVelocity = maxLinearVelocity;
    }
    if(xVelocity < - maxLinearVelocity){
      xVelocity = - maxLinearVelocity;
    }
    if(yVelocity > maxLinearVelocity){
      yVelocity = maxLinearVelocity;
    }
    if(yVelocity < -maxLinearVelocity){
      yVelocity = -maxLinearVelocity;
    }
    if(angularVelocity > maxRotationVelocity){
      angularVelocity = maxRotationVelocity;
    } 
    if(angularVelocity < -maxRotationVelocity){
      angularVelocity = -maxRotationVelocity;
    }
    
    //sets chassisSpeeds to the clamped velocity
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xVelocity, yVelocity, angularVelocity,
        Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
    drivetrain.setChassisSpeeds(speeds);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setBrakes(true);
  }

  // Returns true when the command should end.
  // Returns true when robot position is within the tolerance
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
