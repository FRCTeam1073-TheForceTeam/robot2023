// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.OI;

public class BlingSetCommand extends CommandBase {
  Bling bling;
  OI oi;

  /** Creates a new BlingSetCommand. 
  */
  public BlingSetCommand(Bling bling_, OI oi_) {
    // Use addRequirements() here to declare subsystem dependencies.
    bling = bling_;
    oi = oi_;
    addRequirements(bling);
  }

  // Called when the command is initially scheduled.
  /**
   * @param slotNumber
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  @Override
  public void initialize() {
    for(int i = 0; i < 8 ; i++){
      bling.setSlot(i, 177, 204, 157); //sage green
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * @param Button - 
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   * @return The Value of the operator controller's Y button
   * @return The Value of the operator controller's B button
   */
  @Override
  public void execute() {
    if(oi.getOperatorYButton())
    {
      bling.setRGBAll(255, 0, 0);
    }
    else if(oi.getOperatorBButton()){
      bling.setRGBAll(0, 255, 0);
    }
    else
    {
      bling.setRGBAll(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  /**
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  @Override
  public void end(boolean interrupted) {
    bling.setSlot(7, 0, 0, 0);
  }

  // Returns true when the command should end.
  /**
   * @return false
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}