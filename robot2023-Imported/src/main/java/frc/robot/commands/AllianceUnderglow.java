// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Underglow;

public class AllianceUnderglow extends Command {
  /** Creates a new AllianceUnderglow. */
  private Underglow underglow;
  public AllianceUnderglow(Underglow underglow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.underglow = underglow;
    addRequirements(underglow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double intensity = 0.3; // Default to dim.
    if (DriverStation.isDSAttached()) {
      intensity = 1.0; // Bright if attached.
    }
    // Set lighting to driver station aliance color.
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      underglow.setLEDIntensity(0, 0, intensity);
    }
    else {
      underglow.setLEDIntensity(intensity, 0, 0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
