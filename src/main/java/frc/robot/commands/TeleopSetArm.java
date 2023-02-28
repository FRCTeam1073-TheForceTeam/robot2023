// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.OI;

public class TeleopSetArm extends CommandBase {
  /** Creates a new TeleopSetArm. */
  private Arm arm;
  private OI oi;

  public TeleopSetArm(Arm arm, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.oi = oi;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(oi.getOperatorAButton()){
    //  arm.setTrapezoidTargetAngle(arm.new JointPositions(-3.84, 2.9) );
    //}
    if(oi.getOperatorBButton()){
      arm.setTrapezoidTargetAngle(arm.new JointPositions(-1.5, 3.9));
    }
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
