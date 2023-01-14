// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Translation2d;
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
    private TalonFX steerMotor, driveMotor;
    private SwerveModuleIDConfig ids;
    private CANCoder steerEncoder;
    public Translation2d position;

    public SwerveModule(SwerveModuleConfig cfg, SwerveModuleIDConfig ids)
    {
        this.position = cfg.position;
        this.cfg = cfg;
        this.ids = ids;
        steerMotor = new TalonFX(ids.steerMotorID);
        driveMotor = new TalonFX(ids.driveMotorID);
        steerEncoder = new CANCoder(ids.steerEncoderID);
        setUpMotors();
    }


    public double getSteeringAngle()
    {
        //TODO: do we want it to give us absolute position
        return (steerEncoder.getPosition()*Math.PI/180) - cfg.steerAngleOffset;
    }

    public double getVelocity(){
        return driveMotor.getSelectedSensorVelocity()/cfg.tickPerMeter*10.0;
    }
    
    //*Wrapping code from sds example swerve library
    public void setCommand(double steeringAngle, double driveVelocity){
        SmartDashboard.putNumber(String.format(" Steer Angle %d", cfg.moduleNumber), steeringAngle);
        SmartDashboard.putNumber(String.format(" Drive Velocity %d", cfg.moduleNumber), driveVelocity);
        SmartDashboard.putNumber(String.format(" Encoder Angle %d", cfg.moduleNumber), getSteeringAngle());

        // steeringAngle %= (2.0 * Math.PI);
        // if (steeringAngle < -Math.PI) 
        // {
        //     steeringAngle += 2.0 * Math.PI;
        // }
        // if (steeringAngle > Math.PI)
        // {
        //     steeringAngle -= 2.0 * Math.PI;
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
    public void setDriveVelocity(double driveVelocity)
    {
        driveMotor.set(ControlMode.Velocity, driveVelocity * cfg.tickPerMeter * 0.1);
    }
    public void setSteerAngle(double steeringAngle)
    {
        steerMotor.set(ControlMode.Position, (steeringAngle + cfg.steerAngleOffset) * cfg.tickPerRadian);
    }

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

    public void setUpMotors()
    {
        steerMotor.configFactoryDefault();
        driveMotor.configFactoryDefault();

        steerMotor.setInverted(true);
        driveMotor.setInverted(false);

        //steerMotor.setSafetyEnabled(false);
        //driveMotor.setSafetyEnabled(false);

        steerMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setNeutralMode(NeutralMode.Coast);

        steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, cfg.steerCurrentLimit, cfg.steerCurrentThreshold, cfg.steerCurrentThresholdTime));
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, cfg.driveCurrentLimit, cfg.driveCurrentThreshold, cfg.driveCurrentThresholdTime));

        //Set up talon for CAN encoder
        ErrorCode error = steerMotor.configRemoteFeedbackFilter(steerEncoder, 0);
        if(error != ErrorCode.OK)
        {
            System.out.println("configRemoteFeedbackFilter didn't work: " + error);
        }
        else
        {
            System.out.println("configRemoteFeedbackFilter worked");
        }
        error = steerMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        if(error != ErrorCode.OK)
        {
            System.out.println("configSelectedFeedbackSensor didn't work: " + error);
        }
        else
        {
            System.out.println("configSelectedFeedbackSensor worked");
        }
        steerMotor.setSensorPhase(true);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);



        steerMotor.config_kP(0, cfg.steerP);
        steerMotor.config_kI(0, cfg.steerI);
        steerMotor.config_kD(0, cfg.steerD);
        steerMotor.config_kF(0, cfg.steerF);
        steerMotor.setIntegralAccumulator(0);

        driveMotor.config_kP(0, cfg.driveP);
        driveMotor.config_kI(0, cfg.driveI);
        driveMotor.config_kD(0, cfg.driveD);
        driveMotor.config_kF(0, cfg.driveF);
        driveMotor.setIntegralAccumulator(0);
    }

    public void setDebugTranslate(double power)
    {
        driveMotor.set(ControlMode.PercentOutput, power);
    }

    public void setDebugRotate(double power)
    {
        steerMotor.set(ControlMode.PercentOutput, power);
    }
}