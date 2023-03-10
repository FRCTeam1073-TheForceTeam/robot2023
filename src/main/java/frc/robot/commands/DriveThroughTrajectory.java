// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveThroughTrajectory extends CommandBase {
  /** Creates a new DriveThroughTrajectory. */

  double distanceTolerance = 0.2;
  double angleTolerance = 0.1;

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

  /**Constructs a DriveThroughTrajectory
   * 
   * @param ds variable for driveSubsystem
   * @param start start position
   * @param wayPointList list of waypoints the robot should go through
   * @param end end position
   * @param maxVelocity maximum robot velocity
   * @param maxAngularVelocity maximum angular velocity
   * @param maxAcceleration maximum robot acceleration
   * @param alpha speed multiplier
   */
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

  /** Constructs a DriveThroughTrajectory
   * 
   * @param ds variable for driveSubsystem
   * @param start start position
   * @param posePointList list of waypoints the robot should go through
   * @param maxVelocity maximum robot velocity
   * @param maxAngularVelocity maximum angular velocity
   * @param maxAcceleration maximum robot acceleration
   * @param alpha speed multiplier
   */
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
    //trajectoryCfg.addConstraint(new SwerveDriveKinematicsConstraint(drivetrain.getKinematics(), 2.0));
    trajectoryCfg.setKinematics(drivetrain.getKinematics());
    //trajectory = TrajectoryGenerator.generateTrajectory(posePoints, trajectoryCfg);
    trajectory = generateTrajectory(posePointList, trajectoryCfg);
    addRequirements(ds);
  }

  /** Generates trajectory for robot to go through by creating different trajectory states for each waypoints
   * 
   * @param wayPoints list of waypoints the robot should go through
   * @param trajecotryCfg trajectory config of max acceleration and max velocity
   * @return generated trajectory for robot to follow
   */
  public Trajectory generateTrajectory(ArrayList<Pose2d> wayPoints, TrajectoryConfig trajecotryCfg){
    List<Trajectory.State> traj = new ArrayList<Trajectory.State>();
    double trajectoryTime = 0;
    for(int i = 0; i < wayPoints.size(); i++){
      Trajectory.State ts = new Trajectory.State();
      ts.poseMeters = wayPoints.get(i);
      ts.timeSeconds = trajectoryTime;
      ts.velocityMetersPerSecond = trajecotryCfg.getMaxVelocity();
      ts.curvatureRadPerMeter = 0;
      ts.accelerationMetersPerSecondSq = 1;
      traj.add(ts);
      //update time appropriately
      trajectoryTime += 1;
    }

    return new Trajectory(traj);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  //interpolates the trajectory to get the desired pose at a given time and sets speed proportional to the difference
  @Override
  public void execute() {
    robotPose = drivetrain.getOdometry();
    Trajectory.State state = trajectory.sample(time);
    Transform2d difference = new Transform2d(robotPose, state.poseMeters);
    double xVelocity = alpha * difference.getX();
    double yVelocity = alpha * difference.getY();

    double angularVelocity = 0.6 * difference.getRotation().getRadians();
    //double angularVelocity = 0.4 * difference.getRotation().getRadians();
    //double angularVelocity = 0;

    SmartDashboard.putNumber("Trajectory X", state.poseMeters.getX());
    SmartDashboard.putNumber("Trajectory Y", state.poseMeters.getY());
    SmartDashboard.putNumber("Trajectory theta", state.poseMeters.getRotation().getDegrees());
    SmartDashboard.putNumber("Difference X", difference.getX());
    SmartDashboard.putNumber("Difference y", difference.getY());
    SmartDashboard.putNumber("Time", time);

    xVelocity = MathUtil.clamp(xVelocity, -maxVelocity, maxVelocity);
    yVelocity = MathUtil.clamp(yVelocity, -maxVelocity, maxVelocity);
    angularVelocity = MathUtil.clamp(angularVelocity, -maxAngularVelocity, maxAngularVelocity);

//    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, 
//      Rotation2d.fromDegrees(drivetrain.getHeading()));

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);

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
    var error = robotPose.minus(endPose);
     
    if (error.getTranslation().getNorm()< distanceTolerance && Math.abs(error.getRotation().getRadians()) < angleTolerance) {
      System.out.println("DriveThroughPoint Is Finished");
      return true;
    }
    /*
    if(time >= trajectory.getTotalTimeSeconds()){
      return true;
    }*/
    return false;
  }
}
