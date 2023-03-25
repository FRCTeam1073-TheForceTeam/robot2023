// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private final boolean debug = false;
  private TalonFX collectorMotor;
  private SlewRateLimiter collectorRateLimiter;
  private double targetCollectorSpeed;
  private DigitalInput tof1;
  private DigitalInput tof2;
  private DutyCycle tof1DutyCycleInput;
  private DutyCycle tof2DutyCycleInput;
  private double tof1DutyCycle;
  private double tof2DutyCycle;
  private double tof1Freq;
  private double tof2Freq;
  private double tof1Range;
  private double tof2Range;
  private final double tof1ScaleFactor = 100000;
  private final double tof2ScaleFactor = 100000;
  private final double collectorTicksPerMeter = 2048/0.32;
  //private final boolean debug = true;

  /** Creates a new Claw. */
  public Claw() {
    collectorMotor = new TalonFX(19);
    collectorRateLimiter = new SlewRateLimiter(400000.0); //ticks per second per second
    targetCollectorSpeed = 0;
    tof1 = new DigitalInput(0);
    tof2 = new DigitalInput(1);
    tof1DutyCycleInput = new DutyCycle(tof1);
    tof2DutyCycleInput = new DutyCycle(tof2);
    tof1Freq = 0;
    tof2Freq = 0;
    tof1Range = 0;
    tof2Range = 0;
    setUpMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Actuator 1 Position", getActuatorPosition(1));
    //SmartDashboard.putNumber("Actuator 2 Position", getActuatorPosition(2));
    double collectorSpeed = collectorRateLimiter.calculate(targetCollectorSpeed);
    collectorMotor.set(ControlMode.Velocity, collectorSpeed/10);
    tof1Freq = tof1DutyCycleInput.getFrequency();
    tof2Freq = tof2DutyCycleInput.getFrequency();
    tof1DutyCycle = tof1DutyCycleInput.getOutput();
    tof1DutyCycle = tof1DutyCycleInput.getOutput();
    tof1Range = tof1ScaleFactor * (tof1DutyCycle / tof1Freq - 0.001);
    tof2Range = tof2ScaleFactor * (tof2DutyCycle / tof2Freq - 0.001);
    SmartDashboard.putNumber("TOF 1/Range", tof1Range);
    SmartDashboard.putNumber("TOF 2/Range", tof1Range);
    SmartDashboard.putNumber("Collector Speed", collectorSpeed);
    SmartDashboard.putNumber("TOF 1/Frequency", tof1Freq);
    SmartDashboard.putNumber("TOF 2/Frequency", tof2Freq);

    if (debug) {
      SmartDashboard.putNumber("TOF 1/Frequency", tof1Freq);
      SmartDashboard.putNumber("TOF 2/Frequency", tof2Freq);
      SmartDashboard.putNumber("TOF 1/Duty Cycle", tof1DutyCycle);
      SmartDashboard.putNumber("TOF 2/Duty Cycle", tof2DutyCycle);
      SmartDashboard.putNumber("TOF 1/Time", tof1DutyCycle / tof1Freq);  
      SmartDashboard.putNumber("TOF 2/Time", tof2DutyCycle / tof2Freq);  
    }
  }

  public void setCollectorSpeed(double speed){
    targetCollectorSpeed = speed * collectorTicksPerMeter; //converted to ticks per meter
    // collectorMotor.set(ControlMode.Velocity, targetCollectorSpeed);
  }

  // Initialize preferences for this class:
  public static void initPreferences() 
  {
    
  }

  public double getRange1() {
    return tof1Range;
  }

  public double getRange2() {
    return tof2Range;
  }

  public void setUpMotors(){
    collectorMotor.configFactoryDefault();
    //vacuumMotor.setNeutralMode(NeutralMode.Brake);
    // motor.configRemoteFeedbackFilter(encoder, 0);
    // motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    // motor.setSensorPhase(true);
    collectorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 12, 0.1));

    collectorMotor.config_kP(0, 0.1);
    collectorMotor.config_kI(0, 0);
    collectorMotor.config_kD(0, 0.02);
    collectorMotor.config_kF(0, 0);
    collectorMotor.configMaxIntegralAccumulator(0, 0);
    collectorMotor.setIntegralAccumulator(0);
  }

public String getDiagnostics(){
    String result = "";
    Faults faults = new Faults();
    collectorMotor.getFaults(faults);

    if(faults.hasAnyFault()){
      result += faults.toString();
    }
    ErrorCode error = collectorMotor.clearStickyFaults(500);
    if (error != ErrorCode.OK) {
      result += String.format("can't clear collectorMotor faults");
    }

    if(tof1DutyCycleInput.getFrequency()< 2){
      result += String.format("tof1 not working");
    }
    if(tof2DutyCycleInput.getFrequency()< 2){
      result += String.format("tof2 not working");
    }
        
    return result;
}

}
