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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
  private TalonFX shoulderMotor, elbowMotor, wristMotor;
  private CANCoder shoulderEncoder, elbowEncoder;

  private boolean isElbowMagnetHealthy;
  private boolean isShoulderMagnetHealthy;
  private boolean isShoulderInitialized;
  private boolean isTrajectoryDone;

  //set variables below to correct lengths
  public final double upperArmLength = 25.0;
  public final double forearmLength = 27.5;
  public final double shoulderOffset = 0.0;
  public final double shoulderAbsoluteOffset = 4.3828;
  public final double elbowOffset = 0.0;
  public final double elbowAbsoluteOffset = 3.25;
  public final double wristOffset = 0.0;
  public final double wristAbsoluteOffset = -1.21;
  public final double shoulderTicksPerRadian = -26931.24;
  public final double elbowTicksPerRadian = 13465.62;
  public final double wristTicksPerRadian = -1303.7; // Should check if accurate
  public final double maxShoulderVel = 1.5;
  public final double maxElbowVel = 2.1;
  public final double maxWristVel = 1;
  public final double maxShoulderAcc = 1.5;
  public final double maxElbowAcc = 2.5;
  public final double maxWristAcc = 1;
  public JointPositions minAngles;
  public JointPositions maxAngles;
  public JointPositions currentJointPositions = new JointPositions();
  public JointVelocities currentJointVelocities = new JointVelocities();
  public JointPositions referencePositions = new JointPositions();
  public JointVelocities referenceVelocities = new JointVelocities();
  public ArmTrajectory armTrajectory;

  public JointPositions targetPositions;
  public TrapezoidProfile shoulderProfile;
  public TrapezoidProfile elbowProfile;
  public TrapezoidProfile wristProfile;
  public TrapezoidProfile.State currentShoulderReference;
  public TrapezoidProfile.State currentElbowReference;
  public TrapezoidProfile.State currentWristReference;
  public double profileStartTime;
  Bling bling;


  public class JointPositions{
   public double shoulder;
   public double elbow;
   public double wrist;

    public JointPositions(double shoulderAng, double elbowAng, double wristAng){
      //showing angles in relation to the flat robot chassis and not the angle between two arm segments
      shoulder = shoulderAng;
      elbow = elbowAng;
      wrist = wristAng;
    }

    public JointPositions(){
      //showing angles in relation to the flat robot chassis and not the angle between two arm segments
       shoulder = 0.0;
      elbow = 0.0;
      wrist = 0.0;
    }

    public double getShoulderAngle(){
      return shoulder;
    }

    public double getElbowAngle(){
      return elbow;
    }
    public double getWristAngle(){
      return wrist;
    }

    
  }

  // Clamp joint positions between minimum and maximum positions for safety on commands:
  public static void clamp(JointPositions positions, JointPositions min, JointPositions max) {
    positions.shoulder = MathUtil.clamp(positions.shoulder, min.shoulder, max.shoulder);
    positions.elbow = MathUtil.clamp(positions.elbow, min.elbow, max.elbow);
    positions.wrist = MathUtil.clamp(positions.wrist, min.wrist, max.wrist);
  }


  public class JointWaypoints{
    public double shoulder;
    public double elbow;
    public double wrist;
    public double time;

    public JointWaypoints(double shoulder, double elbow, double wrist, double time){
      this.shoulder = shoulder;
      this.elbow = elbow;
      this.wrist = wrist;
      this.time = time;
    }

    public JointWaypoints(JointPositions positions, double time){
      shoulder = positions.shoulder;
      elbow = positions.elbow;
      wrist = positions.wrist;
      this.time = time;
    }
  }

  public class JointVelocities{
    public double shoulder;
    public double elbow;
    public double wrist;

    public JointVelocities(double shoulderVel, double elbowVel, double wristVel){
      shoulder = shoulderVel;
      elbow = elbowVel;
      wrist = wristVel;
    }

    public JointVelocities(){
      shoulder = 0.0;
      elbow = 0.0;
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


  public class ArmTrajectory{

    private ArrayList<JointWaypoints> waypoints;
    private TrajectoryConfig trajectoryConfig;
    private PolynomialSplineFunction elbowSplines;
    private PolynomialSplineFunction shoulderSplines;
    private PolynomialSplineFunction wristSplines;
    double[] elbowPositions;
    double[] shoulderPositions;
    double[] wristPositions;
    double[] times;
    double finalTime;
    double startTime;


    public ArmTrajectory(ArrayList<JointWaypoints> waypoints, JointVelocities maxVelocity, double maxAcceleration){
      this.waypoints = waypoints;
      //trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration);
      var interpolator = new SplineInterpolator();
      elbowPositions = new double[waypoints.size()];
      shoulderPositions = new double[waypoints.size()];
      wristPositions = new double[waypoints.size()];
      times = new double[waypoints.size()];

      finalTime = 0.0;
      startTime = ((double)System.currentTimeMillis() / 1000.0);
      for(int i = 0; i < waypoints.size(); i++){
        elbowPositions[i] = waypoints.get(i).elbow;
        shoulderPositions[i] = waypoints.get(i).shoulder;
        wristPositions[i] = waypoints.get(i).wrist;
        times[i] = waypoints.get(i).time;
      }

      if (waypoints.size() > 0)
        finalTime = startTime + times[waypoints.size()-1];  // The last time.


      elbowSplines = interpolator.interpolate(times, elbowPositions);
      shoulderSplines = interpolator.interpolate(times, shoulderPositions);
      wristSplines = interpolator.interpolate(times, wristPositions);
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
    public double getFinalTime(){
      return finalTime;
    }

    public double getStartTime(){
      return startTime;
    }

    public JointVelocities getVelocitiesAtTime(double time) {
      double t = time - startTime;
      if (time <= finalTime) {
      //get derivative of elbow spline
      double elbowVelocity = elbowSplines.derivative().value(t);
      //get derivative of shoulder spline
      double shoulderVelocity = shoulderSplines.derivative().value(t);
      double wristVelocity = wristSplines.derivative().value(t);
      return new JointVelocities(shoulderVelocity, elbowVelocity, wristVelocity);
      } else {
        return new JointVelocities(0.0, 0.0, 0.0);
      }
    }

    public JointPositions getPositionsAtTime(double time){
      double t = time - startTime;
      if (time <= finalTime) {
        // Placeholder for joint angles
        return new JointPositions(shoulderSplines.value(t), elbowSplines.value(t), wristSplines.value(t));
      } else {
        // Return the terminal point forever after the time is over:
        return new JointPositions(shoulderPositions[waypoints.size()-1], elbowPositions[waypoints.size()-1], wristPositions[waypoints.size()-1]);
      }
    }
  }

  /** Creates a new Arm. */
  //Set height limiter
  public Arm() {

    shoulderMotor = new TalonFX(16);
    elbowMotor = new TalonFX(18);
    wristMotor = new TalonFX(20); 
    shoulderEncoder = new CANCoder(15);
    elbowEncoder = new CANCoder(17);

    // Called to set up motors:
    setUpMotors();

    // Trapezoid trajectory for angles
    currentShoulderReference = new TrapezoidProfile.State(getAbsoluteAngles().shoulder, 0.0);
    currentElbowReference = new TrapezoidProfile.State(getAbsoluteAngles().elbow, 0.0);
    currentWristReference = new TrapezoidProfile.State(getAbsoluteAngles().wrist, 0.0);
     
    // At the start we "go to where we already are":
    shoulderProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxShoulderVel, maxShoulderAcc), currentShoulderReference, currentShoulderReference);
    elbowProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxElbowVel, maxElbowAcc), currentElbowReference, currentElbowReference);
    wristProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxWristVel, maxElbowAcc),currentWristReference, currentWristReference);
    profileStartTime = System.currentTimeMillis() / 1000.0;
    
    //SmartDashboard.putNumber("Shoulder State on init", currentShoulderState.position);
    //SmartDashboard.putNumber("Elbow State on int", currentElbowState.position);
    //SmartDashboard.putNumber("Wrist state on init", currentWristState.position);
    //minAngles = getAbsoluteAngles();

    // Set our physical angle Limits:
    maxAngles = new JointPositions(3.37, 0.08, 1.5);
    minAngles = new JointPositions(-3.37, 2.98, -1.21);
  }

  @Override
  public void periodic(){
  
    if(!isShoulderInitialized){
      if (initializeShoulder()) {
        System.out.println("Shoulder Initialized!");
        isShoulderInitialized = true;
      } else {
        System.out.println("Shoulder initialization failed!");
      }
    }
     
    double trajectoryTime = ((double)System.currentTimeMillis() / 1000.0);

    /*
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
    
    if(!wristProfile.isFinished(trajectoryTime)){
      currentWristState = wristProfile.calculate(trajectoryTime);
      wristMotor.set(ControlMode.Position, currentWristState.position * wristTicksPerRadian);
    }*/
  
    if(armTrajectory != null){
      referencePositions = armTrajectory.getPositionsAtTime(trajectoryTime);
      referenceVelocities = armTrajectory.getVelocitiesAtTime(trajectoryTime);
      SmartDashboard.putNumber("Shoulder Reference", referencePositions.shoulder);
      SmartDashboard.putNumber("Elbow Reference", referencePositions.elbow);
      SmartDashboard.putNumber("Wrist Reference", referencePositions.wrist);

      shoulderMotor.set(ControlMode.Position, referencePositions.shoulder * shoulderTicksPerRadian);
      elbowMotor.set(ControlMode.Position, referencePositions.elbow * elbowTicksPerRadian);
      wristMotor.set(ControlMode.Position, referencePositions.wrist * wristTicksPerRadian);
    }

    //SmartDashboard.putNumber("Shoulder State", currentShoulderState.position);

    // This method will be called once per scheduler run
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

    //if(((double)System.currentTimeMillis() / 1000.0) -  >= ArmTrajectory.getFinalTime())

    currentJointPositions = getAbsoluteAngles();
    SmartDashboard.putNumber("Shoulder Angle", getJointAngles().shoulder);
    SmartDashboard.putNumber("Elbow Angle", getJointAngles().elbow);
    SmartDashboard.putNumber("Wrist Angle", getJointAngles().wrist);
    //SmartDashboard.putNumber("Shoulder Motor Angle", shoulderMotor.getSelectedSensorPosition());
    //SmartDashboard.putNumber("Elbow Motor Angle", elbowMotor.getSelectedSensorPosition());
    //SmartDashboard.putNumber("Wrist Motor ANgle", wristMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shoulder Absolute Angle", currentJointPositions.shoulder);
    SmartDashboard.putNumber("Elbow Absolute Angle", currentJointPositions.elbow);
    SmartDashboard.putNumber("Wrist Absolute Anglge", currentJointPositions.wrist);
    // SmartDashboard.putNumber("Shoulder Velocitiy", currentJointVelocities.shoulder);
    // SmartDashboard.putNumber("Elbow Velocitiy", currentJointVelocities.elbow);
    // SmartDashboard.putNumber("Wrist Velocity", currentJointVelocities.wrist);
    SmartDashboard.putBoolean("Is Elbow Magnet Healthy", isElbowMagnetHealthy);
    SmartDashboard.putBoolean("Is Shoulder Magnet Healthy", isShoulderMagnetHealthy);
   
  }

  // Initialize preferences for this class:
  public static void initPreferences(){
  
  }

  public void setUpMotors(){
    ErrorCode shoulderError = shoulderMotor.configFactoryDefault();
    ErrorCode elbowError = elbowMotor.configFactoryDefault();
    ErrorCode wristError = wristMotor.configFactoryDefault();

    // Shoulder Motor Setup:
    shoulderMotor.setNeutralMode(NeutralMode.Brake);
    shoulderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1));
    shoulderMotor.setSensorPhase(true);
    //shoulderMotor.setInverted(true);

    shoulderMotor.config_kP(0, 0.2, 100);
    shoulderMotor.config_kI(0, 0, 100);
    shoulderMotor.config_kD(0, 0, 100);
    shoulderMotor.config_kF(0, 0, 100);
    shoulderMotor.configMaxIntegralAccumulator(0, 0, 100);
    shoulderMotor.setIntegralAccumulator(0, 0, 100);
    isShoulderInitialized = false;

    ErrorCode errorShoulder = shoulderMotor.setSelectedSensorPosition(-3.84 * shoulderTicksPerRadian, 0, 400);
    if (errorShoulder != ErrorCode.OK) {
      System.out.println("Shoulder: set selected sensor position failed.");
    }

    // Elbow Motor Setup:
    elbowMotor.setNeutralMode(NeutralMode.Brake);
    elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1));
    elbowMotor.config_kP(0, 0.2);
    elbowMotor.config_kI(0, 0);
    elbowMotor.config_kD(0, 0);
    elbowMotor.config_kF(0, 0);
    elbowMotor.configMaxIntegralAccumulator(0, 0);
    elbowMotor.setIntegralAccumulator(0);

    ErrorCode errorElbow = elbowMotor.setSelectedSensorPosition(getAbsoluteAngles().elbow * elbowTicksPerRadian, 0, 400);
    if (errorElbow != ErrorCode.OK) {
      System.out.println("Elbow: set selected sensor position failed,");
    }


    // Wrist Motor Setup:
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 0.1));

    wristMotor.config_kP(0, 0.2);
    wristMotor.config_kI(0, 0);
    wristMotor.config_kD(0, 0);
    wristMotor.config_kF(0, 0);
    wristMotor.configMaxIntegralAccumulator(0, 0);
    wristMotor.setIntegralAccumulator(0);

    ErrorCode errorWrist = wristMotor.setSelectedSensorPosition(wristAbsoluteOffset * wristTicksPerRadian, 0, 400);
    if (errorWrist != ErrorCode.OK) {
      System.out.println("Wrist: set selected sensor position failed.");
    }


    SmartDashboard.putBoolean("Is errorElbow returned", errorElbow != ErrorCode.OK);
    SmartDashboard.putBoolean("Is errorShoulder returned", errorShoulder != ErrorCode.OK);
    SmartDashboard.putBoolean("Is errorWrsit returned", errorWrist != ErrorCode.OK);

    SmartDashboard.putNumber("Shoulder Angle on init", getJointAngles().shoulder);
    SmartDashboard.putNumber("Elbow Angle on init", getJointAngles().elbow);
    SmartDashboard.putNumber("Wrist Angle on Init", getJointAngles().wrist);

    SmartDashboard.putNumber("Shoulder Absolute Angle on init", getAbsoluteAngles().shoulder);
    SmartDashboard.putNumber("Elbow Absolute Angle on init", getAbsoluteAngles().elbow);
    SmartDashboard.putNumber("Wrist Absolute Angle on Init", getAbsoluteAngles().wrist);
  }
  
  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  // This methods returns the angle of each joint
  public JointPositions getJointAngles() {
    // Sensor angles should be divided by the appropriate ticks per radian
    double shoulderRawAngle = (shoulderMotor.getSelectedSensorPosition()/shoulderTicksPerRadian);
    double elbowRawAngle = (elbowMotor.getSelectedSensorPosition()/elbowTicksPerRadian); 
    double wristRawAngle = (wristMotor.getSelectedSensorPosition()/wristTicksPerRadian);

    //return new JointPositions(shoulderRawAngle + shoulderOffset, elbowRawAngle + elbowOffset - shoulderRawAngle);
    return new JointPositions(shoulderRawAngle, elbowRawAngle, wristRawAngle);
  }

  public boolean initializeShoulder() {
    ErrorCode errorShoulder = shoulderMotor.setSelectedSensorPosition(getAbsoluteAngles().shoulder * shoulderTicksPerRadian, 0, 200);
    if (errorShoulder != ErrorCode.OK) {
      return false;
    } else {
      return true;
    }
  }

  public JointPositions getAbsoluteAngles(){
    double shoulderAngle = (shoulderEncoder.getPosition() * Math.PI / 180 - shoulderAbsoluteOffset);
    double elbowAngle = elbowEncoder.getPosition() * Math.PI / 180 - elbowAbsoluteOffset;
    // No separate absolute sensor for wrist:
    double wristAngle = wristMotor.getSelectedSensorPosition()/wristTicksPerRadian - wristAbsoluteOffset;
    return new JointPositions(shoulderAngle, elbowAngle, wristAngle);
  }

  // This method returns the maximum angles of joints
  public JointPositions getMaxAngles(){
    return maxAngles;
  }

  // This method returns the minimum angles of joints
  public JointPositions getMinAngles(){ 
    return minAngles;
  }

  // This method sets a target angle for joints
  public void setTargetAngles(JointPositions target){
    // elbowLimiter.calculate(target.elbow);
    // shoulderLimiter.calculate(target.shoulder);
    //  shoulderMotor.set(ControlMode.Position, target.shoulder);
    //  elbowMotor.set(ControlMode.Position, target.elbow);
    targetPositions = target;
    //shoulderMotor.set(ControlMode.Position, target.shoulder);
    //elbowMotor.set(ControlMode.Position, target.elbow);
    //wristMotor.set(ControlMode.Position, target.wrist);
    clamp(targetPositions, minAngles, maxAngles);
  }

  public void setTrapezoidTargetAngles(JointPositions target){
    targetPositions = target;

    // Enforce joint target position limits:
    clamp(targetPositions, minAngles, maxAngles);

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
    wristProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxWristVel, maxWristAcc),
      new TrapezoidProfile.State(targetPositions.wrist, 0),
      new TrapezoidProfile.State(getJointAngles().wrist, 0)
      );

  }

  public void setArmTrajectories(ArrayList<JointWaypoints> waypoints, JointVelocities maxVelocities, double maxAcceleration){
    waypoints.add(0, new JointWaypoints(getJointAngles(), 0));
    armTrajectory = new ArmTrajectory(waypoints, maxVelocities, maxAcceleration);
    //endTime = waypoints.get(waypoints.size() - 1).time;
    //endPose = arm.new JointPositions(waypoints.get(waypoints.size() - 1).shoulder, waypoints.get(waypoints.size() - 1).elbow, waypoints.get(waypoints.size() - 1).wrist);
  }

  public boolean isTrajectoryDone(){
    double trajectoryTime = ((double)System.currentTimeMillis() / 1000.0) - armTrajectory.getStartTime();
    if(armTrajectory != null){
      if(armTrajectory.getFinalTime() < trajectoryTime){
        return false;
      }
    }
    return true;
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

    //TODO add limits on wrist rotation

    //shoulderMotor.set(ControlMode.Velocity, -speed.shoulder * 26075.9 / 10);
    //elbowMotor.set(ControlMode.Velocity, speed.elbow * 13037.95 / 10);

    currentJointVelocities.shoulder = speed.shoulder;
    currentJointVelocities.elbow = speed.elbow;
    currentJointVelocities.wrist = speed.wrist;
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

    return new JointPositions(shoulderAngle, elbowAngle + shoulderAngle, 0);
  }

  public JointPositions getInverseKinematicsForElbowUp(CartesianPosition clawPose){
    double x = clawPose.x;
    double z = clawPose.z;
    //JointPositions poses = new JointPositions(0, 0);
    double elbowAngle = - Math.acos((Math.pow(x, 2) + Math.pow(z, 2) - Math.pow(upperArmLength, 2) - Math.pow(forearmLength, 2))/
      (2 * upperArmLength * forearmLength));
    double shoulderAngle = Math.atan(z/x) + Math.atan((forearmLength * Math.sin(elbowAngle))/
      (upperArmLength + forearmLength * Math.cos(elbowAngle)));

    //TODO add wrist calculations
    return new JointPositions(shoulderAngle, elbowAngle + shoulderAngle, 0);
  }
}