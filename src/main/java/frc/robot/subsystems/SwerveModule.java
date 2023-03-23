// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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
    private CANCoder steerEncoder;
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
        steerEncoder = new CANCoder(ids.steerEncoderID);
        setUpMotors();
    }

    public static void initPreferences() 
    {
  
    }

    //Returns diagnostics
    public String getDiagnostics() {
        ErrorCode error;
        String result = new String();
        Faults faults = new Faults();
        driveMotor.getFaults(faults);
        if(faults.hasAnyFault()){
            result += faults.toString();
        }
        steerMotor.getFaults(faults);
        if(faults.hasAnyFault()){
            result += faults.toString();
        }
        error = steerEncoder.clearStickyFaults(500);
        if (error != ErrorCode.OK) {
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
        return (steerEncoder.getPosition()*Math.PI/180) - cfg.steerAngleOffset;
    }

    // Return drive encoder in meters.
    public double getDriveEncoder()
    {
        return -driveMotor.getSelectedSensorPosition() / cfg.tickPerMeter;
    }

    // Return drive velocity in meters/second.
    public double getDriveVelocity(){
        return -driveMotor.getSelectedSensorVelocity()/cfg.tickPerMeter*10.0;
    }

    // Returns the velocity from the motor itself
    public double getDriveRawVelocity() {
        return driveMotor.getSelectedSensorVelocity();
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
        driveMotor.set(ControlMode.Velocity, -driveVelocity * cfg.tickPerMeter / 10.0);
    }

    //setSteerAngle in radians
    public void setSteerAngle(double steeringAngle)
    {
        steerMotor.set(ControlMode.Position, (steeringAngle + cfg.steerAngleOffset) * cfg.tickPerRadian);
    }

    /**Sets motors in the module to brake or coast mode
     * 
     * @param brake a boolean to indicate if motors should be in brake mode or not
     */
    public void setBrake(boolean brake){
        if(brake){
            steerMotor.setNeutralMode(NeutralMode.Brake);
            driveMotor.setNeutralMode(NeutralMode.Brake);
        }
        else{
            steerMotor.setNeutralMode(NeutralMode.Coast);
            driveMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    // configures motors with PIDF values, if it is inverted or not, current limits
    public void setUpMotors()
    {
        var error = steerMotor.configFactoryDefault();

        if (error != ErrorCode.OK) {
            System.out.print(String.format("Module %d STEER MOTOR ERROR: %s", cfg.moduleNumber, error.toString()));
        }

        error = driveMotor.configFactoryDefault();

        if (error != ErrorCode.OK) {
            System.out.println(String.format("Module %d DRIVE MOTOR ERROR: %s", cfg.moduleNumber, error.toString()));
        }

        // Set control direction of motors:
        steerMotor.setInverted(true);
        driveMotor.setInverted(false);


        // Default to brakes off:
        steerMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setNeutralMode(NeutralMode.Coast);

        steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, cfg.steerCurrentLimit, cfg.steerCurrentThreshold, cfg.steerCurrentThresholdTime));
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, cfg.driveCurrentLimit, cfg.driveCurrentThreshold, cfg.driveCurrentThresholdTime));

        //Set up talon for CAN encoder
        error = steerMotor.configRemoteFeedbackFilter(steerEncoder, 0);
        if(error != ErrorCode.OK)
        {
            System.out.println(String.format("Module %d configRemoteFeedbackFilter failed: %s ", cfg.moduleNumber, error));
        }
        
        error = steerMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        if(error != ErrorCode.OK)
        {
            System.out.println(String.format("Module %d configSelectedFeedbackSensor failed: %s ", cfg.moduleNumber, error));
        }
        
        steerMotor.setSensorPhase(true);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setSelectedSensorPosition(0);

        // PID Loop settings for steering position control:
        steerMotor.config_kP(0, cfg.steerP);
        steerMotor.config_kI(0, cfg.steerI);
        steerMotor.config_kD(0, cfg.steerD);
        steerMotor.config_kF(0, cfg.steerF);
        steerMotor.configMaxIntegralAccumulator(0, cfg.steerMaxIntegrator);
        steerMotor.setIntegralAccumulator(0);

        // PID Loop settings for drive velocity control:
        driveMotor.config_kP(0, cfg.driveP);
        driveMotor.config_kI(0, cfg.driveI);
        driveMotor.config_kD(0, cfg.driveD);
        driveMotor.config_kF(0, cfg.driveF);
        driveMotor.configMaxIntegralAccumulator(0, cfg.driveMaxIntegrator);
        driveMotor.setIntegralAccumulator(0);
    }

    /**Sets the percent output velocity to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugTranslate(double power)
    {
        driveMotor.set(ControlMode.PercentOutput, power);
    }

    /**Sets the percent output velocit of wheel angle to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugRotate(double power)
    {
        steerMotor.set(ControlMode.PercentOutput, power);
    }
}