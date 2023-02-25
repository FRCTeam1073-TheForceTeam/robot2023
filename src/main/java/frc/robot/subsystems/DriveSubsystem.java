// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase 
{
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private SwerveModule[] modules;
  private ChassisSpeeds chassisSpeeds;
  private boolean debug = false;
  private Pigeon2 pigeon2;
  private SwerveModulePosition[] modulePositions;
  private double maximumLinearSpeed = 1.0;
  private boolean parkingBrakeOn = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem()
  {
    pigeon2 = new Pigeon2(13);
    var error = pigeon2.configFactoryDefault();
    if (error != ErrorCode.OK) {
      System.out.println(String.format("PIGEON IMU ERROR: %s", error.toString()));
    }
    error = pigeon2.setYaw(180);

    // Make space for four swerve modules:
    modules = new SwerveModule[4];
    modulePositions = new SwerveModulePosition[4];

    //front left
    SwerveModuleIDConfig moduleIDConfig = new SwerveModuleIDConfig(9, 5, 1);
    // moduleIDConfig.driveMotorID = 9; // moduleIDConfig.steerMotorID = 5; // moduleIDConfig.steerEncoderID = 1;

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 0;
    moduleConfig.position = new Translation2d(Preferences.getDouble("Drive.ModulePositions", 0.5017), Preferences.getDouble("Drive.ModulePositions", 0.5017));
    moduleConfig.steerAngleOffset = Preferences.getDouble("Drive.Module0.SteerAngleOffset", 2.879); //2.879;

    modules[0] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[0] = new SwerveModulePosition();

    //front right
    moduleIDConfig = new SwerveModuleIDConfig(10, 6, 2);
    // moduleIDConfig.driveMotorID = 10; // moduleIDConfig.steerMotorID = 6; // moduleIDConfig.steerEncoderID = 2;

    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 1;
    moduleConfig.position = new Translation2d(Preferences.getDouble("Drive.ModulePositions", 0.5017), -Preferences.getDouble("Drive.ModulePositions", 0.5017));
    moduleConfig.steerAngleOffset = Preferences.getDouble("Drive.Module1.SteerAngleOffset", 1.866); // 1.866;

    modules[1] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[1] = new SwerveModulePosition();

    //back left
    moduleIDConfig = new SwerveModuleIDConfig(11, 7, 3);
    // moduleIDConfig.driveMotorID = 11; // moduleIDConfig.steerMotorID = 7; // moduleIDConfig.steerEncoderID = 3;

    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 2;
    moduleConfig.position = new Translation2d(-Preferences.getDouble("Drive.ModulePositions", 0.5017), Preferences.getDouble("Drive.ModulePositions", 0.5017));
    moduleConfig.steerAngleOffset = Preferences.getDouble("Drive.Module2.SteerAngleOffset", 2.422); // 2.422;

    modules[2] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[2] = new SwerveModulePosition();

    //back right
    moduleIDConfig = new SwerveModuleIDConfig(12, 8, 4);
    // moduleIDConfig.driveMotorID = 12; // moduleIDConfig.steerMotorID = 8; // moduleIDConfig.steerEncoderID = 4;
    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 3;
    moduleConfig.position = new Translation2d(-Preferences.getDouble("Drive.ModulePositions", 0.5017), -Preferences.getDouble("Drive.ModulePositions", 0.5017));
    moduleConfig.steerAngleOffset = Preferences.getDouble("Drive.Module3.SteerAngleOffset", 1.109); //1.109;

    modules[3] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[3] = new SwerveModulePosition();

    // Create our kinematics class
    kinematics = new SwerveDriveKinematics(
      modules[0].position,
      modules[1].position,
      modules[2].position,
      modules[3].position
    );

    // Create odometry:
    modules[0].updatePosition(modulePositions[0]);
    modules[1].updatePosition(modulePositions[1]);
    modules[2].updatePosition(modulePositions[2]);
    modules[3].updatePosition(modulePositions[3]);
    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), modulePositions, new Pose2d(0,0,new Rotation2d(Math.PI)));

    // Configure maximum linear speed for limiting:
    maximumLinearSpeed = Preferences.getDouble("Drive.MaximumLinearSpeed", 3.5);
    // Initial chassis speeds are zero:
    chassisSpeeds = new ChassisSpeeds(0,0,0);
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
    Preferences.initDouble("Drive.Module0.SteerAngleOffset", 2.879); // Radians.
    Preferences.initDouble("Drive.Module1.SteerAngleOffset", 1.866);
    Preferences.initDouble("Drive.Module2.SteerAngleOffset", 2.422);
    Preferences.initDouble("Drive.Module3.SteerAngleOffset", 1.109);
    Preferences.initDouble("Drive.MaximumLinearSpeed", 3.5); // Meters/second
    Preferences.initDouble("Drive.ModulePositions", 0.5017);
  }

  public String getDiagnostics() 
  {
    String result = modules[0].getDiagnostics();
    result += modules[1].getDiagnostics();
    result += modules[2].getDiagnostics();
    result += modules[3].getDiagnostics();
    //Check errors for all hardware
    return result;
  }

  public void setDebugMode(boolean debug) {
    this.debug = debug;
  }

  //Returns IMU heading in degrees
  public double getHeading() {
    return pigeon2.getYaw();
  }

  public double getPitch(){
    return pigeon2.getPitch();
  }

  public double getRoll(){
    return pigeon2.getRoll();
  }

  public double getPitchRate()
  {
    return 0.0;
  }

  public double getRollRate()
  {
    return 0.0;
  }

  // Reset IMU heading to zero degrees
  public void zeroHeading() {
    pigeon2.setYaw(0);
  }

  // Set the commanded chassis speeds for the drive subsystem.
  public void setChassisSpeeds(ChassisSpeeds speeds){
    SmartDashboard.putNumber("ChassisSpeed x", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed y", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed rotation", speeds.omegaRadiansPerSecond);
    chassisSpeeds = speeds;
  }

  // Return the measured chassis speeds for the drive subsystem.
  public ChassisSpeeds getChassisSpeeds(){
    SwerveModuleState[] wheelStates = new SwerveModuleState[4];
    wheelStates[0] = new SwerveModuleState();
    wheelStates[1] = new SwerveModuleState();
    wheelStates[2] = new SwerveModuleState();
    wheelStates[3] = new SwerveModuleState();

    wheelStates[0].speedMetersPerSecond = modules[0].getDriveVelocity();
    wheelStates[1].speedMetersPerSecond = modules[1].getDriveVelocity();
    wheelStates[2].speedMetersPerSecond = modules[2].getDriveVelocity();
    wheelStates[3].speedMetersPerSecond = modules[3].getDriveVelocity();

    wheelStates[0].angle = new Rotation2d(modules[0].getSteeringAngle());
    wheelStates[1].angle = new Rotation2d(modules[1].getSteeringAngle());
    wheelStates[2].angle = new Rotation2d(modules[2].getSteeringAngle());
    wheelStates[3].angle = new Rotation2d(modules[3].getSteeringAngle());

    return kinematics.toChassisSpeeds(wheelStates);
  }

  public void setBrakes(boolean brakeOn)
  {
    modules[0].setBrake(brakeOn);
    modules[1].setBrake(brakeOn);
    modules[2].setBrake(brakeOn);
    modules[3].setBrake(brakeOn);
  }

  public SwerveDriveKinematics getKinematics(){
    return kinematics;
  }

  public void updateOdometry() {
    modules[0].updatePosition(modulePositions[0]);
    modules[1].updatePosition(modulePositions[1]);
    modules[2].updatePosition(modulePositions[2]);
    modules[3].updatePosition(modulePositions[3]);
    odometry.update(Rotation2d.fromDegrees(getHeading()), modulePositions);
  }

  public void resetOdometry(Pose2d where){
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), modulePositions, where);
  }

  public Pose2d getOdometry(){
    return new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), Rotation2d.fromDegrees(getHeading()));
  }

  public double getOdometryX()
  {
    return odometry.getPoseMeters().getX();
  }

  public double getOdometryY()
  {
    return odometry.getPoseMeters().getX();
  }

  public Pose3d get3dOdometry(){
    // return odometry position as a pose 3d
    Pose2d odo = getOdometry();
    //TODO: use internal roll and pitch methods later
    return new Pose3d(odo.getX(), odo.getY(), 0.0, new Rotation3d(pigeon2.getRoll(), pigeon2.getPitch(), getHeading()));
  }
  
  @Override
  public void periodic(){
    if (!debug && !parkingBrakeOn)
    {
      // This method will be called once per scheduler run
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, maximumLinearSpeed);
      states[0] = optimizeB(states[0], new Rotation2d(modules[0].getSteeringAngle()));
      states[1] = optimizeB(states[1], new Rotation2d(modules[1].getSteeringAngle()));
      states[2] = optimizeB(states[2], new Rotation2d(modules[2].getSteeringAngle()));
      states[3] = optimizeB(states[3], new Rotation2d(modules[3].getSteeringAngle()));

      modules[0].setCommand(states[0].angle.getRadians(), states[0].speedMetersPerSecond);
      modules[1].setCommand(states[1].angle.getRadians(), states[1].speedMetersPerSecond);
      modules[2].setCommand(states[2].angle.getRadians(), states[2].speedMetersPerSecond);
      modules[3].setCommand(states[3].angle.getRadians(), states[3].speedMetersPerSecond);
    }
    else if(!parkingBrakeOn)
    { //in debug mode
      SmartDashboard.putNumber("Module 0 Velocity", modules[0].getDriveRawVelocity());
    }
    updateOdometry();
    SmartDashboard.putNumber("Odometry.X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry.Y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry.Heading", this.getHeading());

    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
  }

  public void parkingBrake(boolean parkingBrakeOn)
  {
    this.parkingBrakeOn = parkingBrakeOn;
    if (parkingBrakeOn)
    {
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
      states[0] = optimizeB(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), new Rotation2d(modules[0].getSteeringAngle()));
      states[1] = optimizeB(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), new Rotation2d(modules[1].getSteeringAngle()));
      states[2] = optimizeB(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), new Rotation2d(modules[2].getSteeringAngle()));
      states[3] = optimizeB(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), new Rotation2d(modules[3].getSteeringAngle()));

      modules[0].setCommand(states[0].angle.getRadians(), states[0].speedMetersPerSecond);
      modules[1].setCommand(states[1].angle.getRadians(), states[1].speedMetersPerSecond);
      modules[2].setCommand(states[2].angle.getRadians(), states[2].speedMetersPerSecond);
      modules[3].setCommand(states[3].angle.getRadians(), states[3].speedMetersPerSecond);
    }
  }

  public boolean getParkingBrake(){
    return parkingBrakeOn;
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

  public void setDebugDrivePower(double power) {
    modules[0].setDebugTranslate(power);
    modules[1].setDebugTranslate(power);
    modules[2].setDebugTranslate(power);
    modules[3].setDebugTranslate(power);
  }

  public static SwerveModuleState optimizeB(SwerveModuleState desiredState, Rotation2d currentAngle){
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0){
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else{
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound){
          newAngle += 360;
      }
      while (newAngle > upperBound){
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180){
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }
}