// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCorrection extends CommandBase 
{
  Arm arm;
  private double elbowError;
  private double shoulderError;
  private double wristError;
  /** Creates a new ArmCorrection. */
  public ArmCorrection(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    shoulderError = arm.getJointAngles().shoulder - arm.getAbsoluteAngles().shoulder;
    elbowError = arm.getJointAngles().elbow - arm.getAbsoluteAngles().elbow;
    wristError = arm.getJointAngles().wrist - arm.getAbsoluteAngles().wrist;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("shoulder error", shoulderError);
    SmartDashboard.putNumber("elbow error", elbowError);
    SmartDashboard.putNumber("wrist error", wristError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
