// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmSplinePosition extends CommandBase {
  /** Creates a new ArmSplinePosition. */
  Arm arm;
  ArrayList<Arm.JointWaypoints> waypoints;
  Arm.ArmTrajectory armTraj;
  double maxVelocity;
  double maxAcceleration;
  double time;
  double endTime;
  final double shoulderTolerance = 0.01;
  final double elbowTolerance = 0.01;
  final double wristTolerance = 0.01;
  
  public ArmSplinePosition(Arm arm, ArrayList<Arm.JointWaypoints> mWaypoints, double maxVelocity, double maxAcceleration) {
    this.arm = arm;
    this.waypoints = mWaypoints;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    addRequirements(arm);
    waypoints.add(0, arm.new JointWaypoints(arm.getJointAngles(), 0));
    armTraj = arm.new ArmTrajectory(waypoints, maxVelocity, maxAcceleration);
    endTime = waypoints.get(waypoints.size() - 1).time;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.JointPositions currentPositions = armTraj.getPositionsAtTime(time);
    time += 0.02;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //double shoulderError = Math.abs(arm.getJointAngles().getShoulderAngle() - shoulderAng);
	  //double elbowError = Math.abs(arm.getJointAngles().getElbowAngle() - elbowAng);

	  //return ((time == endTime) && (shoulderError <= shoulderTolerance && elbowError <= elbowTolerance));
    return false;
  }
}
