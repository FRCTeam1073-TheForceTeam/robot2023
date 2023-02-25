// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.OI;

public class BlingTeleopCommand extends CommandBase {
  Bling bling;
  OI oi;

  /** Creates a new BlingTeleopComman. */
  public BlingTeleopCommand(Bling bling_, OI oi_) {
    // Use addRequirements() here to declare subsystem dependencies.
    bling = bling_;
    oi = oi_;
    addRequirements(bling);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bling.clearLEDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(oi.getOperatorViewButton()) {
      bling.setSlot(7, 128, 0, 128); //purple for cube
    }
    else if(oi.getOperatorMenuButton()){
      bling.setSlot(7, 255, 255, 0); //yellow for cone
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bling.setSlot(7, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
