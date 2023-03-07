// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * Creates a new ArmSetPosition with tolerances of 0.1 with the absolute value
 * of the shoulder angle
 * 
 * @param arm               The local variable for the arm subsystem
 * @param shoulderAng       The local variable for the shoulder angle
 * @param elbowAng          The local variable for the elbow angle
 * @param shoulderTolerance The variable of 0.1 used to compare to the absolute
 *                          value of the shoulder angle
 * @param elbowTolerance    The variable of 0.1 used to compare to the absolute
 *                          value of the shoulder angle
 * @return Should return true when the command ends
 */


public class ArmSetPosition extends CommandBase {

  private Arm arm;
  private double shoulderAng;
  private double elbowAng;
  private final double shoulderTolerance = 0.1;
  private final double elbowTolerance = 0.1;

  public ArmSetPosition(Arm arm, double shoulderAng, double elbowAng) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.shoulderAng = shoulderAng;
    this.elbowAng = elbowAng;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setTrapezoidTargetAngle(arm.new JointPositions(shoulderAng, elbowAng));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double shoulderError = Math.abs(arm.getJointAngles().getShoulderAngle() - shoulderAng);
    double elbowError = Math.abs(arm.getJointAngles().getElbowAngle() - elbowAng);

    if (shoulderError <= shoulderTolerance && elbowError <= elbowTolerance) {
      return true;
    }
    return false;
  }
}
