// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CollectCommand extends CommandBase {
  /** Creates a new VacuumActivateCommand. */
  private Claw claw;
  private boolean isCube;
  private double time;
  private double startTime;
  
  public CollectCommand(Claw claw, boolean isCube, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.time = time;
    this.claw = claw;
    this.isCube = isCube;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    if(isCube){
      claw.setCollectorSpeed(30);
    }
    else{
      claw.setCollectorSpeed(-30);
    }
   

  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setCollectorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - startTime) >= time);
  }
}
