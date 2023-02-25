// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private TalonFX vacuumMotor;
  private VictorSPX actuator;

  /** Creates a new Claw. */
  public Claw() {
    vacuumMotor = new TalonFX(19);
    setUpMotors();
    actuator = new VictorSPX(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVacuumSpeed(double speed){
    vacuumMotor.set(ControlMode.Velocity, speed * 325.94/10); //converted to ticks per second
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

  public void setUpMotors(){
    vacuumMotor.configFactoryDefault();
    //vacuumMotor.setNeutralMode(NeutralMode.Brake);
    // motor.configRemoteFeedbackFilter(encoder, 0);
    // motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    // motor.setSensorPhase(true);
    vacuumMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 17, 0.1));

    vacuumMotor.config_kP(0, 0.2);
    vacuumMotor.config_kI(0, 0);
    vacuumMotor.config_kD(0, 0);
    vacuumMotor.config_kF(0, 0);
    vacuumMotor.configMaxIntegralAccumulator(0, 0);
    vacuumMotor.setIntegralAccumulator(0);
  }

}
