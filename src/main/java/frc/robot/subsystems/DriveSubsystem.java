// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase 
{
  private SwerveDriveKinematics kinematics;
  private SwerveModule[] modules;
  private ChassisSpeeds chassisSpeeds;
  private boolean debug = false;
  private PigeonIMU pigeonIMU;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {
    pigeonIMU = new PigeonIMU(9);
    pigeonIMU.configFactoryDefault();
    pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 50);
    pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 50);
    pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 50);
    pigeonIMU.setFusedHeading(0);

    modules = new SwerveModule[4];
    //front left
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig();
    SwerveModuleIDConfig moduleIDConfig = new SwerveModuleIDConfig();
    moduleIDConfig.driveMotorID = 43;
    moduleIDConfig.steerMotorID = 28;
    moduleIDConfig.steerEncoderID = 14;
    moduleConfig.position = new Translation2d(-0.217, 0.217);
    moduleConfig.tickPerMeter = 52257; // 
    moduleConfig.driveP = 0.1; 
    moduleConfig.driveI = 0; // placeholder value
    moduleConfig.driveD = 0; // placeholder value
    moduleConfig.driveF = 0; // placeholder value

    moduleConfig.tickPerRadian = 4096.0/(2*Math.PI); // 4,096 ticks per rotation, converted to radians
    moduleConfig.steerAngleOffset = 2.879;
    moduleConfig.steerP = 0.8; 
    moduleConfig.steerI = 0; // placeholder value
    moduleConfig.steerD = 0; // placeholder value
    moduleConfig.steerF = 0; // placeholder value

    modules[0] = new SwerveModule(moduleConfig, moduleIDConfig);

    //front right
    moduleConfig = new SwerveModuleConfig();
    moduleIDConfig = new SwerveModuleIDConfig();
    moduleIDConfig.driveMotorID = 38;
    moduleIDConfig.steerMotorID = 33;
    moduleIDConfig.steerEncoderID = 11;
    moduleConfig.position = new Translation2d(-0.217,-0.217);
    moduleConfig.tickPerMeter = -52257; // TODO: figure out why we need to do this
    moduleConfig.driveP = 0.1; 
    moduleConfig.driveI = 0; // placeholder value
    moduleConfig.driveD = 0; // placeholder value
    moduleConfig.driveF = 0; // placeholder value

    moduleConfig.tickPerRadian = 4096.0/(2*Math.PI); // 4,096 ticks per rotation, converted to radians
    moduleConfig.steerAngleOffset = 1.866;
    moduleConfig.steerP = 0.8; 
    moduleConfig.steerI = 0; // placeholder value
    moduleConfig.steerD = 0; // placeholder value
    moduleConfig.steerF = 0; // placeholder value

    modules[1] = new SwerveModule(moduleConfig, moduleIDConfig);

    //back left
    moduleConfig = new SwerveModuleConfig();
    moduleIDConfig = new SwerveModuleIDConfig();
    moduleIDConfig.driveMotorID = 39;
    moduleIDConfig.steerMotorID = 47;
    moduleIDConfig.steerEncoderID = 12;
    moduleConfig.position = new Translation2d(0.217, 0.217);
    moduleConfig.tickPerMeter = 52257;
    moduleConfig.driveP = 0.1; 
    moduleConfig.driveI = 0; // placeholder value
    moduleConfig.driveD = 0; // placeholder value
    moduleConfig.driveF = 0; // placeholder value

    moduleConfig.tickPerRadian = 4096.0/(2*Math.PI); // 4,096 ticks per rotation, converted to radians
    moduleConfig.steerAngleOffset = 2.422; //1.109;
    moduleConfig.steerP = 0.8; 
    moduleConfig.steerI = 0; // placeholder value
    moduleConfig.steerD = 0; // placeholder value
    moduleConfig.steerF = 0; // placeholder value

    modules[2] = new SwerveModule(moduleConfig, moduleIDConfig);

    //back right
    moduleConfig = new SwerveModuleConfig();
    moduleIDConfig = new SwerveModuleIDConfig();
    moduleIDConfig.driveMotorID = 26;
    moduleIDConfig.steerMotorID = 40;
    moduleIDConfig.steerEncoderID = 10;
    moduleConfig.position = new Translation2d(0.217,-0.217);
    moduleConfig.tickPerMeter = -52257; //TODO: same thing as id 11
    moduleConfig.driveP = 0.1; 
    moduleConfig.driveI = 0; // placeholder value
    moduleConfig.driveD = 0; // placeholder value
    moduleConfig.driveF = 0; // placeholder value

    moduleConfig.tickPerRadian = 4096.0/(2*Math.PI); // 4,096 ticks per rotation, converted to radians
    moduleConfig.steerAngleOffset = 1.109; //2.422
    moduleConfig.steerP = 0.8; 
    moduleConfig.steerI = 0; // placeholder value
    moduleConfig.steerD = 0; // placeholder value
    moduleConfig.steerF = 0; // placeholder value

    modules[3] = new SwerveModule(moduleConfig, moduleIDConfig);

    kinematics = new SwerveDriveKinematics(
      modules[0].position,
      modules[1].position,
      modules[2].position,
      modules[3].position
    );
    chassisSpeeds = new ChassisSpeeds(0,0,0);
  }

  //returns heading in degrees
  public double getHeading(){
    return pigeonIMU.getFusedHeading();
  }

  public void zeroHeading(){
    pigeonIMU.setFusedHeading(0);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }

  public ChassisSpeeds getChassisSpeeds(){
    SwerveModuleState[] wheelStates = new SwerveModuleState[4];
    wheelStates[0] = new SwerveModuleState();
    wheelStates[1] = new SwerveModuleState();
    wheelStates[2] = new SwerveModuleState();
    wheelStates[3] = new SwerveModuleState();

    wheelStates[0].speedMetersPerSecond = modules[0].getVelocity();
    wheelStates[1].speedMetersPerSecond = modules[1].getVelocity();
    wheelStates[2].speedMetersPerSecond = modules[2].getVelocity();
    wheelStates[3].speedMetersPerSecond = modules[3].getVelocity();

    wheelStates[0].angle = new Rotation2d(modules[0].getSteeringAngle());
    wheelStates[1].angle = new Rotation2d(modules[1].getSteeringAngle());
    wheelStates[2].angle = new Rotation2d(modules[2].getSteeringAngle());
    wheelStates[3].angle = new Rotation2d(modules[3].getSteeringAngle());

    return kinematics.toChassisSpeeds(wheelStates);
  }

  @Override
  public void periodic() {
    if (! debug){
      // This method will be called once per scheduler run
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);

      modules[0].setCommand(states[0].angle.getRadians(), states[0].speedMetersPerSecond);
      modules[1].setCommand(states[1].angle.getRadians(), states[1].speedMetersPerSecond);
      modules[2].setCommand(states[2].angle.getRadians(), states[2].speedMetersPerSecond);
      modules[3].setCommand(states[3].angle.getRadians(), states[3].speedMetersPerSecond);
    }
    SmartDashboard.putNumber("Module 0 Angle", modules[0].getSteeringAngle());
    SmartDashboard.putNumber("Module 1 Angle", modules[1].getSteeringAngle());
    SmartDashboard.putNumber("Module 2 Angle", modules[2].getSteeringAngle());
    SmartDashboard.putNumber("Module 3 Angle", modules[3].getSteeringAngle());
    SmartDashboard.putNumber("Module 0 Velocity", modules[0].getVelocity());
    SmartDashboard.putNumber("Module 1 Velocity", modules[1].getVelocity());
    SmartDashboard.putNumber("Module 2 Velocity", modules[2].getVelocity());
    SmartDashboard.putNumber("Module 3 Velocity", modules[3].getVelocity());
    SmartDashboard.putNumber("Heading", getHeading());
  }
  public void setDebugSpeed(double speed){
    modules[0].setDriveVelocity(speed);
    modules[1].setDriveVelocity(speed);
    modules[2].setDriveVelocity(speed);
    modules[3].setDriveVelocity(speed);
  }
  public void setDebugAngle(double angle){
    SmartDashboard.putNumber("Debug Angle", angle);

    modules[0].setSteerAngle(angle);
    modules[1].setSteerAngle(angle);
    modules[2].setSteerAngle(angle);
    modules[3].setSteerAngle(angle);
  }
}