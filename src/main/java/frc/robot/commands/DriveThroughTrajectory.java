// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveThroughTrajectory extends CommandBase {
  /** Creates a new DriveThroughTrajectory. */
  double distanceTolerance = 0.1;
  double angleTolerance = 0.05;

  DriveSubsystem drivetrain;
  Pose2d startPose;
  Pose2d endPose;
  Pose2d robotPose;
  ArrayList<Translation2d> wayPoints;
  ArrayList<Pose2d> posePoints;
  TrajectoryConfig trajectoryCfg;
  Trajectory trajectory;
  double time;
  double maxVelocity;
  double maxAngularVelocity;
  double alpha;
  public DriveThroughTrajectory(DriveSubsystem ds, Pose2d start, ArrayList<Translation2d> wayPointList, Pose2d end, 
  double maxVelocity, double maxAngularVelocity, double maxAcceleration, double alpha) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = ds;
    startPose = start;
    wayPoints = wayPointList;
    endPose = end;
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
    this.alpha = alpha;
    trajectoryCfg = new TrajectoryConfig(maxVelocity, maxAcceleration);
    trajectory = TrajectoryGenerator.generateTrajectory(startPose, wayPoints, endPose, trajectoryCfg);
    addRequirements(ds);
  }

  public DriveThroughTrajectory(DriveSubsystem ds, Pose2d start, ArrayList<Pose2d> posePointList, 
  double maxVelocity, double maxAngularVelocity, double maxAcceleration, double alpha) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = ds;
    startPose = start;
    posePoints = new ArrayList<Pose2d>();
    posePoints.add(start);
    posePoints.addAll(posePointList);
    endPose = posePoints.get(posePoints.size() - 1);
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
    this.alpha = alpha;
    trajectoryCfg = new TrajectoryConfig(maxVelocity, maxAcceleration);
    trajectory = TrajectoryGenerator.generateTrajectory(posePoints, trajectoryCfg);
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = drivetrain.getOdometry();
    Trajectory.State state = trajectory.sample(time);
    Transform2d difference = state.poseMeters.minus(robotPose);
    double xVelocity = alpha * difference.getX();
    double yVelocity = alpha * difference.getY();

    //Transform2d angleDifference = endPose.minus(robotPose);
    //double angularVelocity = 0.4 * angleDifference.getRotation().getRadians();;
    double angularVelocity = 0.4 * difference.getRotation().getRadians();

    SmartDashboard.putNumber("Trajectory X", state.poseMeters.getX());
    SmartDashboard.putNumber("Trajectory Y", state.poseMeters.getY());
    SmartDashboard.putNumber("Difference X", difference.getX());
    SmartDashboard.putNumber("Difference y", difference.getY());
    SmartDashboard.putNumber("Time", time);


    if(xVelocity > maxVelocity){
      xVelocity = maxVelocity;
    }
    if(xVelocity < - maxVelocity){
      xVelocity = - maxVelocity;
    }
    if(yVelocity > maxVelocity){
      yVelocity = maxVelocity;
    }
    if(yVelocity < -maxVelocity){
      yVelocity = -maxVelocity;
    }
    if(angularVelocity > maxAngularVelocity){
      angularVelocity = maxAngularVelocity;
    } 
    if(angularVelocity < -maxAngularVelocity){
      angularVelocity = -maxAngularVelocity;
    }

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, 
    Rotation2d.fromDegrees(drivetrain.getHeading()));
    drivetrain.setChassisSpeeds(chassisSpeeds);
    if(time < trajectory.getTotalTimeSeconds()){
      time += 0.02;
    }
    else{
      time = trajectory.getTotalTimeSeconds();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var error = endPose.minus(robotPose);
    if (error.getTranslation().getNorm()< distanceTolerance && Math.abs(error.getRotation().getRadians()) < angleTolerance) {
      System.out.println("DriveThroughPoint Is Finished");
      return true;
    }
    return false;
  }
}
