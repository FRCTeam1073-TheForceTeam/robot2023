// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private TalonFX vacuumMotor;
  private TalonSRX actuator;
  private final double closedPosition = 0;
  private final double openedPosition = 0;
  private final double conePosition = 0;
  private final double cubePosition = 0;
  //private final boolean debug = true;

  /** Creates a new Claw. */
  public Claw() {
    vacuumMotor = new TalonFX(19);
    setUpMotors();
    actuator = new TalonSRX(20);
    setUpActuators();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actuator Position", getActuatorPosition());
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
    actuator.set(ControlMode.Position, openedPosition);
  }

  // This method will close the claw
  public void closeClaw(){
    actuator.set(ControlMode.Position, closedPosition);
  }

  // This method closes enough to fit a cone
  public void closeOnCone(){
    actuator.set(ControlMode.Position, conePosition);
  }

  // This method closes enough to fit a cube
  public void closeOnCube(){
    actuator.set(ControlMode.Position, cubePosition);
  }

  public double getActuatorPosition(){
    return actuator.getSelectedSensorPosition();
  }

  public void setActuatorSensorPosition(double position){
    actuator.setSelectedSensorPosition(position);
  }

  public void setActuatorDebugVelocity(double speed){
    actuator.set(ControlMode.Velocity, speed);
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

  public void setUpActuators(){
    actuator.configFactoryDefault();
  }

}
