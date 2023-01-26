// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Underglow extends SubsystemBase {
  /** Creates a new Underglow. */
  public Underglow() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // This method sets the color
  public void setColor(Color8Bit color){

  }

  public Color8Bit getCurrentColor(){
    return new Color8Bit(0,0,0);
  }
}
