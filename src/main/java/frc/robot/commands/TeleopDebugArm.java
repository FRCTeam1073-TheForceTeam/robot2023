// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.OI;

public class TeleopDebugArm extends CommandBase {
  private Arm arm;
  private OI oi;

  /** Creates a new DebugArm. */
  public TeleopDebugArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(false){
      arm.setTargetAngle(arm.new JointPositions(0,0)); //tuck arm in
    }

    arm.setJointVelocities(arm.new JointVelocities(oi.getOperatorLeftX() * .5, oi.getOperatorRightX()*.5));
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
