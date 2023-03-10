// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
  private TalonFX shoulderMotor, elbowMotor;
  private CANCoder shoulderEncoder, elbowEncoder;
  //private SlewRateLimiter elbowLimiter;
  //private SlewRateLimiter shoulderLimiter;
  private boolean isElbowMagnetHealthy;
  private boolean isShoulderMagnetHealthy;
  //set variables below to correct lengths
  public final double upperArmLength = 25.0;
  public final double forearmLength = 27.5;
  public final double shoulderOffset = 0.0;
  public final double shoulderAbsoluteOffset = 2.42;
  public final double elbowOffset = 0.0;
  public final double elbowAbsoluteOffset = 1.73;
  public final double shoulderTicksPerRadian = 26931.24;
  public final double elbowTicksPerRadian = 13465.62;
  public final double maxShoulderVel = 1.5;
  public final double maxElbowVel = 2.1;
  public final double maxShoulderAcc = 1.5;
  public final double maxElbowAcc = 2.5;
  public JointPositions minAngles;
  public JointPositions currentJointPositions = new JointPositions();
  public JointPositions targetPositions;
  public JointVelocities currentJointVelocities = new JointVelocities();
  public TrapezoidProfile shoulderProfile;
  public TrapezoidProfile elbowProfile;
  public TrapezoidProfile.State currentShoulderState;
  public TrapezoidProfile.State currentElbowState;
  public double profileStartTime;
  Bling bling;


  public class JointPositions{
   public double shoulder;
   public double elbow;

    public JointPositions(double shoulderAng, double elbowAng){
      //showing angles in relation to the flat robot chassis and not the angle between two arm segments
      shoulder = shoulderAng;
      elbow = elbowAng;
    }

    public JointPositions(){
      //showing angles in relation to the flat robot chassis and not the angle between two arm segments
       shoulder = 0.0;
      elbow = 0.0;
    }

    public double getShoulderAngle(){
      return shoulder;
    }

    public double getElbowAngle(){
      return elbow;
    }
  }

  public class JointWaypoints{
    double shoulder;
    double elbow;
    double time;

    public JointWaypoints(double shoulder, double elbow, double time){
      this.shoulder = shoulder;
      this.elbow = elbow;
      this.time = time;
    }

    public JointWaypoints(JointPositions positions, double time){
      shoulder = positions.shoulder;
      elbow = positions.elbow;
      this.time = time;
    }
  }

  public class JointVelocities{
    public double shoulder;
    public double elbow;

    public JointVelocities(double shoulderVel, double elbowVel){
      shoulder = shoulderVel;
      elbow = elbowVel;
    }

    public JointVelocities(){
      shoulder = 0.0;
      elbow = 0.0;
    }
  }

  public class CartesianPosition{
    double x;
    double z;
    double pitch;

    public double getCartesianX(){
      return x;
    }

    public double getCartesianZ(){
      return z;
    }

    public double getCartesianPitch(){
      return pitch;
    }
  }


  public class ArmTrajectory{

    private ArrayList<JointWaypoints> waypoints;
    private TrajectoryConfig trajectoryConfig;
    private PolynomialSplineFunction elbowSplines;
    private PolynomialSplineFunction shoulderSplines;
    double[] elbowPositions;
    double[] shoulderPositions;
    double[] times;


    public ArmTrajectory(ArrayList<JointWaypoints> waypoints, double maxVelocity, double maxAcceleration){
      this.waypoints = waypoints;
      trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration);
      var interpolator = new SplineInterpolator();
      elbowPositions = new double[waypoints.size()];
      shoulderPositions = new double[waypoints.size()];
      times = new double[waypoints.size()];

      for(int i = 0; i < waypoints.size(); i++){
        elbowPositions[i] = waypoints.get(i).elbow;
        shoulderPositions[i] = waypoints.get(i).shoulder;
        times[i] = waypoints.get(i).time;
      }

      elbowSplines = interpolator.interpolate(elbowPositions, times);
      shoulderSplines = interpolator.interpolate(shoulderPositions, times);
    }
 /* 
    public double getTimeForMotion(double[] singularJoint){
      double difference = Math.abs(singularJoint[0] - singularJoint[shoulderPositions.length]);
      double time = trajectoryConfig.getMaxVelocity()/trajectoryConfig.getMaxAcceleration();
      time += difference/trajectoryConfig.getMaxVelocity();
      time += trajectoryConfig.getMaxVelocity()/trajectoryConfig.getMaxAcceleration();
      return time;
    }
/* 
    public double timeBetweenPoints(JointPositions firstPoint, JointPositions secondPoint){

    }
*/
    public JointVelocities getVelocityAtTime(double time){
      //get derivative of elbow spline
      double elbowVelocity = elbowSplines.derivative().value(time);
      //get derivative of shoulder spline
      double shoulderVelocity = shoulderSplines.derivative().value(time);
      return new JointVelocities(shoulderVelocity, elbowVelocity);
    }

    public JointPositions getTrajectoryAtTime(double time){
      //placeholder for joint angles
      return new JointPositions(shoulderSplines.value(time), elbowSplines.value(time));
    }
  }

  /** Creates a new Arm. */
  //Set height limiter
  public Arm() {
    shoulderMotor = new TalonFX(16);
    elbowMotor = new TalonFX(18);
    shoulderEncoder = new CANCoder(15);
    elbowEncoder = new CANCoder(17);

    //elbowLimiter = new SlewRateLimiter(0.5);
    //shoulderLimiter = new SlewRateLimiter(0.5);

    setUpMotor(shoulderMotor, shoulderEncoder);
    setUpMotor(elbowMotor, elbowEncoder);
    shoulderMotor.setSensorPhase(true);
    shoulderMotor.setInverted(true);

    shoulderMotor.config_kP(0, 0.2);
    shoulderMotor.config_kI(0, 0);
    shoulderMotor.config_kD(0, 0);
    shoulderMotor.config_kF(0, 0);
    shoulderMotor.configMaxIntegralAccumulator(0, 0);
    shoulderMotor.setIntegralAccumulator(0);

    elbowMotor.config_kP(0, 0.2);
    elbowMotor.config_kI(0, 0);
    elbowMotor.config_kD(0, 0);
    elbowMotor.config_kF(0, 0);
    elbowMotor.configMaxIntegralAccumulator(0, 0);
    elbowMotor.setIntegralAccumulator(0);

    ErrorCode errorElbow = elbowMotor.setSelectedSensorPosition(getAbsoluteAngles().elbow * elbowTicksPerRadian, 0, 400);
    ErrorCode errorShoulder = shoulderMotor.setSelectedSensorPosition(-3.84 * shoulderTicksPerRadian, 0, 400);

    SmartDashboard.putBoolean("Is errorElbow returned", errorElbow != null);
    SmartDashboard.putBoolean("Is errorShoulder returned", errorShoulder != null);

    SmartDashboard.putNumber("Shoulder Angle on init", getJointAngles().shoulder);
    SmartDashboard.putNumber("Elbow Angle on init", getJointAngles().elbow);

    SmartDashboard.putNumber("Shoulder Absolute Angle on init", getAbsoluteAngles().shoulder);
    SmartDashboard.putNumber("Elbow Absolute Angle on init", getAbsoluteAngles().elbow);
    //Trapezoid trajectory for angles
    
    currentShoulderState = new TrapezoidProfile.State(getAbsoluteAngles().shoulder, 0.0);
    currentElbowState = new TrapezoidProfile.State(getAbsoluteAngles().elbow, 0.0);
    //targetPositions = new JointPositions(getJointAngles().shoulder, getJointAngles().elbow);
     
    shoulderProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxShoulderVel, maxShoulderAcc), currentShoulderState, currentShoulderState);
    elbowProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxElbowVel, maxElbowAcc), currentElbowState, currentElbowState);
    profileStartTime = System.currentTimeMillis() / 1000.0;
    
    SmartDashboard.putNumber("Shoulder State on init", currentShoulderState.position);
    SmartDashboard.putNumber("Elbow State on int", currentElbowState.position);
    //minAngles = getAbsoluteAngles();
  }

  @Override
  public void periodic(){
    double trajectoryTime = ((double)System.currentTimeMillis() / 1000.0) - profileStartTime;
    //setting angles with trapezoid trajectories
    //SmartDashboard.putNumber("Shoulder State", currentShoulderState.position);
    //SmartDashboard.putNumber("Elbow State", currentElbowState.position);
    if(!elbowProfile.isFinished(trajectoryTime)){
      currentElbowState = elbowProfile.calculate(trajectoryTime);
      elbowMotor.set(ControlMode.Position, currentElbowState.position * elbowTicksPerRadian);
      //SmartDashboard.putNumber("Elbow State", currentElbowState.position);
    }
    if(!shoulderProfile.isFinished(trajectoryTime)){
      currentShoulderState = shoulderProfile.calculate(trajectoryTime);
      shoulderMotor.set(ControlMode.Position, currentShoulderState.position * shoulderTicksPerRadian);
    }
    //SmartDashboard.putNumber("Shoulder State", currentShoulderState.position);
    SmartDashboard.putNumber("Shoulder State", currentShoulderState.position);
    SmartDashboard.putNumber("Elbow State", currentElbowState.position);

    // This method will be called once per scheduler run
    currentJointPositions = getAbsoluteAngles();
    SmartDashboard.putNumber("Shoulder Angle", getJointAngles().shoulder);
    SmartDashboard.putNumber("Elbow Angle", getJointAngles().elbow);
    SmartDashboard.putNumber("Shoulder Motor Angle", shoulderMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbow Motor Angle", elbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shoulder Absolute Angle", currentJointPositions.shoulder);
    SmartDashboard.putNumber("Elbow Absolute Angle", currentJointPositions.elbow);
    SmartDashboard.putNumber("Shoulder Velocitiy", currentJointVelocities.shoulder);
    SmartDashboard.putNumber("Elbow Velocitiy", currentJointVelocities.elbow);
    SmartDashboard.putBoolean("Is Elbow Magnet Healthy", isElbowMagnetHealthy);
    SmartDashboard.putBoolean("Is Shoulder Magnet Healthy", isShoulderMagnetHealthy);
    
    if(elbowEncoder.getMagnetFieldStrength().equals(MagnetFieldStrength.BadRange_RedLED)){
      isElbowMagnetHealthy = false;
    }
    else{
      isElbowMagnetHealthy = true;
    }

    if(shoulderEncoder.getMagnetFieldStrength().equals(MagnetFieldStrength.BadRange_RedLED)){
      isShoulderMagnetHealthy = false;
    }
    else{
      isShoulderMagnetHealthy = true;
    }

  }

  // Initialize preferences for this class:
  public static void initPreferences(){
  
  }

  public void setUpMotor(TalonFX motor, CANCoder encoder){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    // motor.configRemoteFeedbackFilter(encoder, 0);
    // motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    // motor.setSensorPhase(true);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1));
  }
  
  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  // This methods returns the angle of each joint
  public JointPositions getJointAngles(){
    //sensor angles should be divided by the appropriate ticks per radian
    double shoulderRawAngle = (shoulderMotor.getSelectedSensorPosition()/shoulderTicksPerRadian);
    double elbowRawAngle = (elbowMotor.getSelectedSensorPosition()/elbowTicksPerRadian); 
    //return new JointPositions(shoulderRawAngle + shoulderOffset, elbowRawAngle + elbowOffset - shoulderRawAngle);
    return new JointPositions(shoulderRawAngle, elbowRawAngle);
  }

  public void initializeShoulder(){
    ErrorCode errorShoulder = shoulderMotor.setSelectedSensorPosition(getAbsoluteAngles().shoulder * shoulderTicksPerRadian, 0, 200);
  }

  public JointPositions getAbsoluteAngles(){
    double shoulderAngle = -(shoulderEncoder.getPosition() * Math.PI / 180 - shoulderAbsoluteOffset);
    double elbowAngle = elbowEncoder.getPosition() * Math.PI / 180 - elbowAbsoluteOffset;
    return new JointPositions(shoulderAngle, elbowAngle);
  }

  // This method returns the maximum angles of joints
  public JointPositions getMaxAngles(){
    return new JointPositions(3.37, 0.08);
  }

  // This method returns the minimum angles of joints
  public JointPositions getMinAngles(){ 
    return new JointPositions(-3.37, 2.98);
  }

  // This method sets a target angle for joints
  public void setTargetAngle(JointPositions target){
    //elbowLimiter.calculate(target.elbow);
    //shoulderLimiter.calculate(target.shoulder);
  //  shoulderMotor.set(ControlMode.Position, target.shoulder);
  //  elbowMotor.set(ControlMode.Position, target.elbow);
  }

  public void setTrapezoidTargetAngle(JointPositions target){
    targetPositions = target;
    profileStartTime = ((double)System.currentTimeMillis() / 1000.0);
    elbowProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxElbowVel, maxElbowAcc), 
      new TrapezoidProfile.State(targetPositions.elbow, 0),
      new TrapezoidProfile.State(getJointAngles().elbow, 0)
    );

    shoulderProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxShoulderVel, maxShoulderAcc), 
      new TrapezoidProfile.State(targetPositions.shoulder, 0),
      new TrapezoidProfile.State(getJointAngles().shoulder, 0)
    );   
  }

  // This method sets a target speed for joints
  public void setJointVelocities(JointVelocities speed){
    if(currentJointPositions.shoulder < -3.77){ //3.79
      if(speed.shoulder > 0){
        speed.shoulder = 0;
      }
    }
    if(currentJointPositions.shoulder > 0.98){ //1.0
      if(speed.shoulder < 0){
        speed.shoulder = 0;
      }
    }

    if(currentJointPositions.elbow < -2.82){ //-2.82
      if(speed.elbow < 0){
        speed.elbow = 0;
      }
    }
    if(currentJointPositions.elbow > 2.99){
      if(speed.elbow > 0){
        speed.elbow = 0;
      }
    }

    //shoulderMotor.set(ControlMode.Velocity, -speed.shoulder * 26075.9 / 10);
    //elbowMotor.set(ControlMode.Velocity, speed.elbow * 13037.95 / 10);
    currentJointVelocities.shoulder = speed.shoulder;
    currentJointVelocities.elbow = speed.elbow;
  }

  // This method returns the position of claw given different joint angles
  public CartesianPosition getCartesianPosition(JointPositions positions){
    CartesianPosition cartesianPosition = new CartesianPosition();
    cartesianPosition.x = (Math.cos(positions.elbow) * forearmLength) + (Math.cos(positions.shoulder) * upperArmLength);
    cartesianPosition.z = (Math.sin(positions.elbow) * forearmLength) + (Math.sin(positions.shoulder) * upperArmLength);
    cartesianPosition.pitch = getJointAngles().elbow + getJointAngles().shoulder;
    return cartesianPosition;
  }

  // This method returns the joint angles given a position of the claw
  public JointPositions getInverseKinematics(CartesianPosition clawPose){
    double x = clawPose.x;
    double z = clawPose.z;
    //JointPositions poses = new JointPositions(0, 0);
    double elbowAngle = Math.acos((Math.pow(x, 2) + Math.pow(z, 2) - Math.pow(upperArmLength, 2) - Math.pow(forearmLength, 2))/
      (2 * upperArmLength * forearmLength));
    double shoulderAngle = Math.atan(z/x) - Math.atan((forearmLength * Math.sin(elbowAngle))/
      (upperArmLength + forearmLength * Math.cos(elbowAngle)));
      elbowAngle -= shoulderAngle;

    return new JointPositions(shoulderAngle, elbowAngle + shoulderAngle);
  }

  public JointPositions getInverseKinematicsForElbowUp(CartesianPosition clawPose){
    double x = clawPose.x;
    double z = clawPose.z;
    //JointPositions poses = new JointPositions(0, 0);
    double elbowAngle = - Math.acos((Math.pow(x, 2) + Math.pow(z, 2) - Math.pow(upperArmLength, 2) - Math.pow(forearmLength, 2))/
      (2 * upperArmLength * forearmLength));
    double shoulderAngle = Math.atan(z/x) + Math.atan((forearmLength * Math.sin(elbowAngle))/
      (upperArmLength + forearmLength * Math.cos(elbowAngle)));

    return new JointPositions(shoulderAngle, elbowAngle + shoulderAngle);
  }
}