// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.sql.Savepoint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveToPoint extends CommandBase {
  /** Creates a new DriveToPoint. */
  
  double distanceTolerance = 0.1;
  double distance;
  double velocity;

  double alpha = 1;
  double beta = 1.5;
  double gamma = 1.5;

  Pose2d robotPose;
  Pose2d targetPose;
  DriveSubsystem drivetrain;
  ChassisSpeeds chassisSpeeds;
  private final static double maximumLinearVelocity = 1;   // Meters/second
  private final static double maximumRotationVelocity = 1; // Radians/second

  public DriveToPoint(DriveSubsystem ds, Pose2d targetPose2d, double maximumLinearVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = ds;
    this.targetPose = targetPose2d;
    this.velocity = maximumLinearVelocity;
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Called every time the scheduler runs while the command is scheduled.
    distance = 0;
    double targetAngle = targetPose.getRotation().getRadians();
    System.out.println("DriveToPoint");
 }

  @Override
  public void execute() {
    //getOdometry not complete
    robotPose = drivetrain.getOdometry();
    Transform2d error = targetPose.minus(robotPose);
    double xVelocity = 0.5 * error.getX();
    double yVelocity = 0.5 * error.getY();
    double angularVelocity = 0.5 * error.getRotation().getRadians();
    double velocityMult = maximumLinearVelocity;
    double rotateMult = maximumRotationVelocity;
    
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        yVelocity * velocityMult,
        xVelocity * velocityMult, 
        angularVelocity * rotateMult,
        Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
    drivetrain.setChassisSpeeds(speeds);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var error = targetPose.minus(robotPose);
    if (error.getTranslation().getNorm()< 0.1 && error.getRotation().getRadians()< 0.05) {
      System.out.println("DriveToPoint Is Finished");
      return true;
    }
    return false;
  }
}
