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
import edu.wpi.first.wpilibj.Preferences;

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

    // Make space for four swerve modules:
    modules = new SwerveModule[4];

    //front left
    SwerveModuleIDConfig moduleIDConfig = new SwerveModuleIDConfig(43, 28, 14);
    // moduleIDConfig.driveMotorID = 43; // moduleIDConfig.steerMotorID = 28; // moduleIDConfig.steerEncoderID = 14;

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 0;
    moduleConfig.position = new Translation2d(0.217, 0.217);
    moduleConfig.steerAngleOffset = Preferences.getDouble("Drive.Module0.SteerAngleOffset", 2.879); //2.879;

    modules[0] = new SwerveModule(moduleConfig, moduleIDConfig);

    //front right
    moduleIDConfig = new SwerveModuleIDConfig(38, 33, 11);
    // moduleIDConfig.driveMotorID = 38; // moduleIDConfig.steerMotorID = 33; // moduleIDConfig.steerEncoderID = 11;

    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 1;
    moduleConfig.position = new Translation2d(0.217,-0.217);
    moduleConfig.steerAngleOffset = Preferences.getDouble("Drive.Module1.SteerAngleOffset", 1.866); // 1.866;

    modules[1] = new SwerveModule(moduleConfig, moduleIDConfig);

    //back left
    moduleIDConfig = new SwerveModuleIDConfig(39, 47, 12);
    // moduleIDConfig.driveMotorID = 39; // moduleIDConfig.steerMotorID = 47; // moduleIDConfig.steerEncoderID = 12;

    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 2;
    moduleConfig.position = new Translation2d(-0.217, 0.217);
    moduleConfig.steerAngleOffset = Preferences.getDouble("Drive.Module2.SteerAngleOffset", 2.422); // 2.422;

    modules[2] = new SwerveModule(moduleConfig, moduleIDConfig);

    //back right
    moduleIDConfig = new SwerveModuleIDConfig(26, 40, 10);
    // moduleIDConfig.driveMotorID = 26; // moduleIDConfig.steerMotorID = 40; // moduleIDConfig.steerEncoderID = 10;
    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 3;
    moduleConfig.position = new Translation2d(-0.217,-0.217);
    moduleConfig.steerAngleOffset = Preferences.getDouble("Drive.Module3.SteerAngleOffset", 1.109); //1.109;

    modules[3] = new SwerveModule(moduleConfig, moduleIDConfig);

    // Create our kinematics class
    kinematics = new SwerveDriveKinematics(
      modules[0].position,
      modules[1].position,
      modules[2].position,
      modules[3].position
    );

    // Initial chassis speeds are zero:
    chassisSpeeds = new ChassisSpeeds(0,0,0);
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
    Preferences.initDouble("Drive.Module0.SteerAngleOffset", 2.879);
    Preferences.initDouble("Drive.Module1.SteerAngleOffset", 1.866);
    Preferences.initDouble("Drive.Module2.SteerAngleOffset", 2.422);
    Preferences.initDouble("Drive.Module3.SteerAngleOffset", 1.109);
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

  public void setBrakes(boolean brakeOn){
    modules[0].setBrake(brakeOn);
    modules[1].setBrake(brakeOn);
    modules[2].setBrake(brakeOn);
    modules[3].setBrake(brakeOn);
  }

  @Override
  public void periodic() {
    if (! debug){
      // This method will be called once per scheduler run
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);
      states[0] = SwerveModuleState.optimize(states[0], new Rotation2d(modules[0].getSteeringAngle()));
      states[1] = SwerveModuleState.optimize(states[1], new Rotation2d(modules[1].getSteeringAngle()));
      states[2] = SwerveModuleState.optimize(states[2], new Rotation2d(modules[2].getSteeringAngle()));
      states[3] = SwerveModuleState.optimize(states[3], new Rotation2d(modules[3].getSteeringAngle()));

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