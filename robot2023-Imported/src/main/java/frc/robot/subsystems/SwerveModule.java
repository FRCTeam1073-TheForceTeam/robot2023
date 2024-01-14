// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. 
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
 * 
 * THIS CODE ONLY WORKS IF YOU SET THE CANCODERS TO BOOT TO ABSOLUTE POSITION
 * 
 * OTHERWISE THE WHEELS WILL NOT INITIALIZE IN THE CORRECT POSITIONS
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
*/

public class SwerveModule 
{
    private SwerveModuleConfig cfg;
    private SwerveModuleIDConfig idcfg;
    private TalonFX steerMotor, driveMotor;
    // private SwerveModuleIDConfig ids;
    private CANcoder steerEncoder;
    public Translation2d position;
    
    /** Constructs a swerve module class. Initializes drive and steer motors
     * 
     * @param cfg swerve module configuration values for this module
     * @param ids Can Ids for this module
     */
    public SwerveModule(SwerveModuleConfig cfg, SwerveModuleIDConfig ids)
    {
        this.position = cfg.position;
        this.cfg = cfg;
        this.idcfg = ids;
        // this.ids = ids;
        steerMotor = new TalonFX(ids.steerMotorID);
        driveMotor = new TalonFX(ids.driveMotorID);
        steerEncoder = new CANcoder(ids.steerEncoderID);
        setUpMotors();
    }

    public static void initPreferences() 
    {
  
    }

    //Returns diagnostics
    public String getDiagnostics() {
        StatusCode error;
        String result = new String();
        // Fault faults = new Faults();
        // driveMotor.getFaults(faults);
        // if (faults.hasAnyFault()) {
        //     result += faults.toString();
        // }
        // steerMotor.getFaults(faults);
        // if(faults.hasAnyFault()){
        //     result += faults.toString();
        // }
        error = steerEncoder.clearStickyFaults(500);
        if (error != StatusCode.OK) {
            result += String.format(" Module %d, steerEncoder %d, error.", cfg.moduleNumber, idcfg.steerEncoderID);
        }
        return result;
      }

    // Populate a SwerveModulePosition object from the state of this module.
    public void updatePosition(SwerveModulePosition position){
        position.angle = Rotation2d.fromRadians(getSteeringAngle());
        position.distanceMeters = getDriveEncoder();
    }

    // Return steering sensor angle in radians. 0 = dead ahead on robot.
    public double getSteeringAngle()
    {
        return (steerEncoder.getPosition().getValue() / (Math.PI * (2.0))) - cfg.steerAngleOffset;
    }

    // Return drive encoder in meters.
    public double getDriveEncoder()
    {
        return -driveMotor.getRotorPosition().getValue() / cfg.tickPerMeter;
    }

    // Return drive velocity in meters/second.
    public double getDriveVelocity(){ 
        // TODO: Find out meters per second
        // TODO:Check ^^
        return -driveMotor.getRotorVelocity().getValue() * cfg.metersPerRotation;
    }

    // Returns the velocity from the motor itself
    public double getDriveRawVelocity() { // TODO: Same thing
        return driveMotor.getRotorVelocity().getValue();
    }
    
    //*Wrapping code from sds example swerve library
    public void setCommand(double steeringAngle, double driveVelocity){
        SmartDashboard.putNumber(String.format(" Steer Angle %d", cfg.moduleNumber), steeringAngle);
        SmartDashboard.putNumber(String.format(" Drive Velocity %d", cfg.moduleNumber), driveVelocity);
        SmartDashboard.putNumber(String.format(" Encoder Angle %d", cfg.moduleNumber), getSteeringAngle());

        // steeringAngle %= (2.0 * Math.PI);
        // if (steeringAngle < -Math.PI) 
        // {
        //     steeringAngle += Math.PI;
        // }
        // if (steeringAngle > Math.PI)
        // {
        //     steeringAngle -= Math.PI;
        // }

        // double difference = steeringAngle - getSteeringAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        // if (difference >= Math.PI) {
        //     steeringAngle -= 2.0 * Math.PI;
        // } else if (difference < -Math.PI) {
        //     steeringAngle += 2.0 * Math.PI;
        // }
        // difference = steeringAngle - getSteeringAngle(); // Recalculate difference

        // SmartDashboard.putNumber(String.format(" Difference %d", ids.steerEncoderID), difference);
        //SmartDashboard.putNumber(String.format(" Steer Angle Result %d", ids.steerEncoderID), steeringAngle);


        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        /*if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steeringAngle += Math.PI;
            driveVelocity *= -1.0;
        }*/

        // Put the target angle back into the range [0, 2pi)
        // steeringAngle %= (2.0 * Math.PI);
        // if (steeringAngle < 0.0) {
        //     steeringAngle += 2.0 * Math.PI;
        // }

        setDriveVelocity(driveVelocity);
        setSteerAngle(steeringAngle);
        // SmartDashboard.putNumber(String.format(" Steer Angle %d", ids.steerEncoderID), steeringAngle);
        // SmartDashboard.putNumber(String.format(" Drive Velocity %d", ids.steerEncoderID), driveVelocity);
        // SmartDashboard.putNumber(String.format(" Difference %d", ids.steerEncoderID), difference);
    }

    //Sets the velocity for the drive motors
    public void setDriveVelocity(double driveVelocity)
    {
        // Velocity commands are ticks per meter in 0.1 seconds... so 1/10th the ticks/second.
        //TODO: cvt driveVelocity to units specified above
        driveMotor.setControl(new VelocityDutyCycle(-driveVelocity * cfg.metersPerRotation));
    }

    //setSteerAngle in radians
    public void setSteerAngle(double steeringAngle)
    {
        //TODO: fix steering angle units and add offset
        //(steeringAngle + cfg.steerAngleOffset) * cfg.tickPerRadian
        steerMotor.setControl(new PositionDutyCycle(steeringAngle));
    }

    /**Sets motors in the module to brake or coast mode
     * 
     * @param brake a boolean to indicate if motors should be in brake mode or not
     */
    public void setBrake(boolean brake){
        if(brake){
            steerMotor.setNeutralMode(NeutralModeValue.Brake);
            driveMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        else{
            steerMotor.setNeutralMode(NeutralModeValue.Coast);
            driveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    // configures motors with PIDF values, if it is inverted or not, current limits
    public void setUpMotors()
    {
        var error = steerMotor.getConfigurator().apply(new TalonFXConfiguration());

        if (error != StatusCode.OK) {
            System.out.print(String.format("Module %d STEER MOTOR ERROR: %s", cfg.moduleNumber, error.toString()));
        }

        error = driveMotor.getConfigurator().apply(new TalonFXConfiguration());

        if (error != StatusCode.OK) {
            System.out.println(String.format("Module %d DRIVE MOTOR ERROR: %s", cfg.moduleNumber, error.toString()));
        }

        // Set control direction of motors:
        steerMotor.setInverted(true);
        driveMotor.setInverted(false);


        // Default to brakes off:
        steerMotor.setNeutralMode(NeutralModeValue.Coast);
        driveMotor.setNeutralMode(NeutralModeValue.Coast);

        //steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, cfg.steerCurrentLimit, cfg.steerCurrentThreshold, cfg.steerCurrentThresholdTime));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(cfg.steerCurrentLimit));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentThreshold(cfg.steerCurrentThreshold));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyTimeThreshold(cfg.steerCurrentThresholdTime));

        //driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, cfg.driveCurrentLimit, cfg.driveCurrentThreshold, cfg.driveCurrentThresholdTime));
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(cfg.driveCurrentLimit));
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentThreshold(cfg.driveCurrentThreshold));
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyTimeThreshold(cfg.driveCurrentThresholdTime));

        //Set up talon for CAN encoder
        // error = steerMotor.configRemoteFeedbackFilter(steerEncoder, 0);
        // if(error != StatusCode.OK)
        // {
        //     System.out.println(String.format("Module %d configRemoteFeedbackFilter failed: %s ", cfg.moduleNumber, error));
        // }
        
        error = steerMotor.getConfigurator().apply(new TalonFXConfiguration().Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
        if(error != StatusCode.OK)
        {
            System.out.println(String.format("Module %d configSelectedFeedbackSensor failed: %s ", cfg.moduleNumber, error));
        }

        //driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.getConfigurator().apply(new TalonFXConfiguration().Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        driveMotor.setPosition(0);
    
        // PID Loop settings for steering position control:
        var steerMotorClosedLoopConfig = new Slot0Configs();

        steerMotorClosedLoopConfig.kP = cfg.steerP;
        steerMotorClosedLoopConfig.kI = cfg.steerI;
        steerMotorClosedLoopConfig.kD = cfg.steerD;
        steerMotorClosedLoopConfig.kV = cfg.steerF;
        
        steerMotor.getConfigurator(). apply(steerMotorClosedLoopConfig, 0.05);

        
        // PID Loop settings for drive velocity control:
        var driveMotorClosedLoopConfig = new Slot0Configs();

        driveMotorClosedLoopConfig.kP = cfg.driveP;
        driveMotorClosedLoopConfig.kI = cfg.driveI;
        driveMotorClosedLoopConfig.kD = cfg.driveD;
        driveMotorClosedLoopConfig.kV = cfg.driveF;

        driveMotor.getConfigurator(). apply(driveMotorClosedLoopConfig, 0.05);

    }

    /**Sets the percent output velocity to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugTranslate(double power)
    {
        driveMotor.setControl(new DutyCycleOut(power));
    }

    /**Sets the percent output velocity of wheel angle to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugRotate(double power)
    {
        steerMotor.setControl(new DutyCycleOut(power));
    }
}