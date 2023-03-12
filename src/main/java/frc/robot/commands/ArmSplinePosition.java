// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  Arm.JointPositions endPose;
  final double shoulderTolerance = 0.01;
  final double elbowTolerance = 0.01;
  final double wristTolerance = 0.01;
  
  public ArmSplinePosition(Arm arm, ArrayList<Arm.JointWaypoints> mWaypoints, double maxVelocity, double maxAcceleration) {
    this.arm = arm;
    this.waypoints = mWaypoints;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmTrajectories(waypoints, arm.new JointVelocities(1, 1, 1), maxAcceleration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
	  return (arm.isTrajectoryDone());
  }
}
