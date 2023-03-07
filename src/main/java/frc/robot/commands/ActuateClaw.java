// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ActuateClaw extends CommandBase {
  /** Creates a new ActuateClaw. */
  private Claw claw; 
  //private double time;
  //private double startTime;
  private double actuator1Val;
  private double actuator2Val;

  public ActuateClaw(Claw claw, double actuator1Val, double actuator2Val) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.actuator1Val = actuator1Val;
    this.actuator2Val = actuator2Val;
    //time = timeSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //startTime = System.currentTimeMillis() / 1000.0;
    /* 
    if(open){
      claw.setActuatorDebugPercent(0.5);
    }
    if(!open){
      claw.setActuatorDebugPercent(0.3);
    }
    */
    claw.setActuator1Angle(actuator1Val);
    claw.setActuator2Angle(actuator2Val);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //claw.setActuatorDebugPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return ((System.currentTimeMillis() / 1000.0) - startTime) >= time;
    return true;
  }
}
