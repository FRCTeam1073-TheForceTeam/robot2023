// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TeleopOptimizedArm extends CommandBase {
  private Arm arm;

  /** Creates a new OptimizedArm. */
  public TeleopOptimizedArm() {
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
      arm.setTargetAngle(new Arm.JointPositions(0,0)); //tuck arm in
    }

    if(false){
      //move arm up/down
    }

    if(false){
      //move arm forward/backward
    }
    
    //Do we want other specific positions (i.e low/medium/high grid locations)?
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
