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
  Arm.JointVelocities maxVelocity;
  double maxAcceleration;
  double time;
  double endTime;
  int endIdx;
  Arm.JointPositions endPose;
  final double shoulderTolerance = 0.01;
  final double elbowTolerance = 0.01;
  final double wristTolerance = 0.01; 
  
  public PlannedArmPath(Arm arm, PathPlanner planner, int endIdx, Arm.JointVelocities maxVelocities) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.planner = planner;
    this.maxAcceleration = maxAcceleration;
    this.endIdx = endIdx;
    this.maxVelocity = maxVelocities;
    waypoints = new ArrayList<Arm.JointWaypoints>();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    path = planner.getPathFromClosest(arm.getJointAngles().shoulder , arm.getJointAngles().elbow, arm.getJointAngles().wrist, endIdx);
    waypoints.clear();
    for(int i = 0; i < path.size(); i++){
      waypoints.add(arm.new JointWaypoints(path.get(i).shoulder, path.get(i).elbow, path.get(i).wrist, (i + 1) * 3));
    }

    for(int i = 0; i < waypoints.size(); i++){
      System.out.println(waypoints.get(i).time);
      System.out.println(waypoints.get(i).shoulder);
      System.out.println(waypoints.get(i).elbow);
      System.out.println(waypoints.get(i).wrist);
    }
    arm.setArmTrajectories(waypoints, maxVelocity, maxAcceleration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    waypoints.clear();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.isTrajectoryDone());
  }
}
