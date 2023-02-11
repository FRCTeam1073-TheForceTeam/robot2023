// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Underglow;

public class UnderglowSetCommand extends CommandBase {
  /** Creates a new SetUnderglowCommand. */
  Underglow underglow;
  OI oi;
  public UnderglowSetCommand(Underglow underglow_, OI oi_) {
    // Use addRequirements() here to declare subsystem dependencies.
    underglow = underglow_;
    oi = oi_;
    addRequirements(underglow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    underglow.setLEDIntensity(0, 0.5, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(oi.getOperatorXButton()) {
      underglow.setLEDIntensity(1, 0, 0);
      System.out.println("pressing X button");
    }
    else if(oi.getOperatorAButton()) {
      underglow.setLEDIntensity(0, 0, 1);
    }
    else
    {
      underglow.setLEDIntensity(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    underglow.setLEDIntensity(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
