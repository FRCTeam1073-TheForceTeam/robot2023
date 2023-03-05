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

public class Wrist extends SubsystemBase{
  private TalonFX wristMotor;
  public final double wristOffset = 0.0;
  public final double wristAbsoluteOffset = 0.0;
  public final double shoulderTicksPerRadian = 2;//TODO correct ticks/radian when gear ratio finalized
  public final double maxwristVel = 1.5;
  public final double maxwristAcc = 1.5;
  public JointPositions minAngles;
  public JointPositions currentJointPositions = new JointPositions();
  public JointPositions targetPositions;
  public JointVelocities currentJointVelocities = new JointVelocities();
  public TrapezoidProfile wristProfile;
  public TrapezoidProfile.State currentWristState;
  public double profileStartTime;
  Bling bling;


  public class JointPositions{
   public double wrist;

    public JointPositions(double wristAng){
      //showing angles in relation to the flat robot chassis and not the angle between two arm segments
      wrist = wristAng;
    }

    public JointPositions(){
      //showing angles in relation to the flat robot chassis and not the angle between two arm segments
       wrist = 0.0;
    }

    public double getWristAngle(){
      return wrist;
    }
  }

  public class JointWaypoints{
    double wrist;
    double time;

    public JointWaypoints(double wrist, double time){
      this.wrist = wrist;
      this.time = time;
    }

    public JointWaypoints(JointPositions positions, double time){
      wrist = positions.wrist;
      this.time = time;
    }
  }

  public class JointVelocities{
    public double wrist;

    public JointVelocities(double wristVel){
      wrist = wristVel;
    }

    public JointVelocities(){
      wrist = 0.0;
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


  public class WristTrajectory{

    private ArrayList<JointWaypoints> waypoints;
    private TrajectoryConfig trajectoryConfig;
    private PolynomialSplineFunction wristSplines;
    double[] wristPositions;
    double[] times;


    public WristTrajectory(ArrayList<JointWaypoints> waypoints, double maxVelocity, double maxAcceleration){
      this.waypoints = waypoints;
      trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration);
      var interpolator = new SplineInterpolator();
      wristPositions = new double[waypoints.size()];
      times = new double[waypoints.size()];

      for(int i = 0; i < waypoints.size(); i++){
        wristPositions[i] = waypoints.get(i).wrist;
        times[i] = waypoints.get(i).time;
      }

      wristSplines = interpolator.interpolate(wristPositions, times);
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
      //get derivative of shoulder spline
      double shoulderVelocity = wristSplines.derivative().value(time);
      return new JointVelocities(wristVelocity);
    }

    public JointPositions getTrajectoryAtTime(double time){
      //placeholder for joint angles
      return new JointPositions(wristSplines.value(time));
    }
  }

  /** Creates a new Arm. */
  //Set height limiter
  public Wrist() {
    wristMotor = new TalonFX(18);

    //elbowLimiter = new SlewRateLimiter(0.5);
    //shoulderLimiter = new SlewRateLimiter(0.5);

    setUpMotor(wristMotor);
    wristMotor.setSensorPhase(true);
    wristMotor.setInverted(true);

    wristMotor.config_kP(0, 0.2);
    wristMotor.config_kI(0, 0);
    wristMotor.config_kD(0, 0);
    wristMotor.config_kF(0, 0);
    wristMotor.configMaxIntegralAccumulator(0, 0);
    wristMotor.setIntegralAccumulator(0);

    ErrorCode errorWrist = wristMotor.setSelectedSensorPosition(-3.84 * wristTicksPerRadian, 0, 400);

    SmartDashboard.putBoolean("Is errorWrist returned", errorWrist != null);

    SmartDashboard.putNumber("Wrist Angle on init", getJointAngles().wrist);

    SmartDashboard.putNumber("Wrist Absolute Angle on init", getAbsoluteAngles().wrist);
    //Trapezoid trajectory for angles
    
    currentWristState = new TrapezoidProfile.State(getAbsoluteAngles().wrist, 0.0);
    //targetPositions = new JointPositions(getJointAngles().shoulder, getJointAngles().elbow);
     
    wristProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxWristVel, maxWristAcc), currentWristState, currentWristState);
    profileStartTime = System.currentTimeMillis() / 1000.0;
    
    SmartDashboard.putNumber("Wrist State on init", currentWristState.position);
    //minAngles = getAbsoluteAngles();
  }

  @Override
  public void periodic(){
    double trajectoryTime = ((double)System.currentTimeMillis() / 1000.0) - profileStartTime;
    //setting angles with trapezoid trajectories
    //SmartDashboard.putNumber("Shoulder State", currentShoulderState.position);
    //SmartDashboard.putNumber("Elbow State", currentElbowState.position);
    if(!wristProfile.isFinished(trajectoryTime)){
      currentWristState = wristProfile.calculate(trajectoryTime);
      wristMotor.set(ControlMode.Position, currentwristState.position * wristTicksPerRadian);
      //SmartDashboard.putNumber("Elbow State", curren3tElbowState.position);
    }
   
    //SmartDashboard.putNumber("Shoulder State", currentShoulderState.position);
    SmartDashboard.putNumber("Wrist State", currentWristState.position);

    // This method will be called once per scheduler run
    currentJointPositions = getAbsoluteAngles();
    SmartDashboard.putNumber("Wrist Angle", getJointAngles().wrist);
    SmartDashboard.putNumber("Wrist Motor Angle", wristMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Wrist Absolute Angle", currentJointPositions.wrist);
    SmartDashboard.putNumber("Wrist Velocitiy", currentJointVelocities.wrist);
  }

  // Initialize preferences for this class:
  public static void initPreferences(){
  
  }

  public void setUpMotor(TalonFX motor){
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
    double shoulderRawAngle = (wristMotor.getSelectedSensorPosition()/wristTicksPerRadian);
    //return new JointPositions(shoulderRawAngle + shoulderOffset, elbowRawAngle + elbowOffset - shoulderRawAngle);
    return new JointPositions(wristRawAngle);
  }

  public void initializeShoulder(){
    ErrorCode wrist = wristMotor.setSelectedSensorPosition(getAbsoluteAngles().wrist * wristTicksPerRadian, 0, 200);
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