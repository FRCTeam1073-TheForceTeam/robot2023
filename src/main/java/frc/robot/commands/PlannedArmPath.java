// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PathPlanner;
import frc.robot.subsystems.Arm.JointWaypoints;

public class PlannedArmPath extends CommandBase {
  /** Creates a new PlannedArmPath. */
  Arm arm;
  PathPlanner planner;
  ArrayList<PathPlanner.Node> path;
  ArrayList<Arm.JointWaypoints> waypoints;
  Arm.ArmTrajectory armTraj;
  double maxVelocity;
  double maxAcceleration;
  double time;
  double endTime;
  Arm.JointPositions endPose;
  final double shoulderTolerance = 0.01;
  final double elbowTolerance = 0.01;
  final double wristTolerance = 0.01; 
  
  public PlannedArmPath(Arm arm, PathPlanner planner, int endIdx, double maxAcceleration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.planner = planner;
    this.maxAcceleration = maxAcceleration;
    path = planner.getPathFromClosest(arm.getJointAngles().shoulder , arm.getJointAngles().elbow, arm.getAbsoluteAngles().wrist, endIdx);

    for(int i = 0; i < path.size(); i++){
      waypoints.add(arm.new JointWaypoints(path.get(i).shoulder, path.get(i).elbow, path.get(i).wrist, i * 3));
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmTrajectories(waypoints, arm.new JointVelocities(1, 1, 1), maxAcceleration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.isTrajectoryDone());
  }
}
