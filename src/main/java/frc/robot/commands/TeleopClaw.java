// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.OI;

public class TeleopClaw extends CommandBase {
  private Claw claw;
  private OI oi;

  /** Creates a new TeleopClaw. */
  public TeleopClaw(Claw claw, OI oi) {
    this.claw = claw;
    this.oi = oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(oi.getOperatorLeftBumper()){
      claw.setVacuumSpeed(0);
    }
    if(oi.getOperatorRightBumper()){
      claw.setVacuumSpeed(628.4); //intial: 314.2 rps
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