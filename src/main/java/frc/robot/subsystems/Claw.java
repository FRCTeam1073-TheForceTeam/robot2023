// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private TalonFX vacuumMotor; 

  /** Creates a new Claw. */
  public Claw() {
    vacuumMotor = new TalonFX(19);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVacuumPower(double power){
    vacuumMotor.set(ControlMode.PercentOutput, power);
  }

  // Initialize preferences for this class:
  public static void initPreferences() 
  {
  
  }

  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  // This method will open the claw
  public void openClaw(){

  }

  // This method will close the claw
  public void closeClaw(){

  }

  // This method closes enough to fit a cone
  public void closeOnCone(){

  }

  // This method closes enough to fit a cube
  public void closeOnCube(){

  }


}
