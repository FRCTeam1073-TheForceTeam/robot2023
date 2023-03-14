// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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

  // Set variables below to correct lengths:
  public final double upperArmLength = 0.635; // ~25.0"
  public final double forearmLength = 0.6858; // ~27.5";
  public final double wristLength = 0.15; // ~6"
  // Approximage masses of arm segments for gravity compensation:
  public final double upperArmMass = 1.0; // 2.2lbs
  public final double foreArmMass = 0.5; // 1.1lbs
  public final double wristMass = 3.85; // ~8.5lbs
  public final double gravityCompensationGain = 1.25;            // Increase this to increase the overall amount of gravity compensation.
  public final double gravityCompensationShoulderGain = 0.0125; // 1/80 gear ratio
  public final double gravityCompensationElbowGain = 0.025;     // 1/40 gear ratio
  public final double gravityCompensationWristGain = 0.05;      // 1/20 gear ratio


  public final double shoulderOffset = 0.0;
  public final double shoulderAbsoluteOffset = 4.3828;
  public final double elbowOffset = 0.0;
  public final double elbowAbsoluteOffset = 3.25;
  public final double wristOffset = 0.0;
  public final double wristAbsoluteOffset = -1.21;
  public final double shoulderTicksPerRadian = -26931.24;
  public final double elbowTicksPerRadian = 13465.62;
  public final double wristTicksPerRadian = -6518.5; // New 20:1 ratio.
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
  public JointPositions absoluteJointPositions = new JointPositions();
  public JointPositions referencePositions = new JointPositions();
  public JointVelocities referenceVelocities = new JointVelocities();
  public ArmTrajectory armTrajectory;

  public double profileStartTime;
  public boolean motorsEnabled = false;

  public double[] gravityCompensation;   // Feed forward gravity compensation forces for shoulder, elbow, wrist in order.

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
      elbowPositions = new double[waypoints.size() + 1];
      shoulderPositions = new double[waypoints.size() + 1];
      wristPositions = new double[waypoints.size() + 1];
      times = new double[waypoints.size() + 1];

      finalTime = 0.0;
      startTime = ((double)System.currentTimeMillis() / 1000.0);
      JointPositions pos = getJointAngles();
      elbowPositions[0] = pos.elbow;
      shoulderPositions[0] = pos.shoulder;
      wristPositions[0] = pos.wrist;
      times[0] = 0;
      for(int i = 0; i < waypoints.size(); i++){
        elbowPositions[i + 1] = waypoints.get(i).elbow;
        shoulderPositions[i + 1] = waypoints.get(i).shoulder;
        wristPositions[i + 1] = waypoints.get(i).wrist;
        times[i + 1] = waypoints.get(i).time;
      }

      if (waypoints.size() > 0)
        finalTime = startTime + times[times.length - 1];  // The last time.
      else{
        finalTime = startTime;
      }

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

    // Fills in velocities with trajectory velocity at time:
    public void getVelocitiesAtTime(double time, JointVelocities velocities) {
      double t = time - startTime;
      if (time <= finalTime) {
        velocities.shoulder = shoulderSplines.derivative().value(t);
        velocities.elbow = elbowSplines.derivative().value(t);
        velocities.wrist = wristSplines.derivative().value(t);
      } else {
        velocities.shoulder = 0.0;
        velocities.elbow = 0.0;
        velocities.wrist = 0.0;
      }
    }

    // Fills in positions with trajectory position at time:
    public void getPositionsAtTime(double time, JointPositions positions){
      double t = time - startTime;
      if (time <= finalTime) {
        // Placeholder for joint angles
        positions.shoulder = shoulderSplines.value(t);
        positions.elbow = elbowSplines.value(t);
        positions.wrist = wristSplines.value(t);
      } else {
        positions.shoulder = shoulderPositions[shoulderPositions.length - 1];
        positions.elbow = elbowPositions[elbowPositions.length - 1];
        positions.wrist = wristPositions[wristPositions.length - 1];
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

    // Set our physical angle Limits:
    maxAngles = new JointPositions(3.37, 0.08, 1.5);
    minAngles = new JointPositions(-3.37, 2.98, -1.21);

    // Allocate space for gravity compensation:
    gravityCompensation = new double[3]; // Shoulder, elbow, wrist gravity compensation.
    gravityCompensation[0] = 0.0; // Shoulder.
    gravityCompensation[1] = 0.0; // Elbow.
    gravityCompensation[2] = 0.0; // Wrist.

    motorsEnabled = false;
  }

  // Runs in periodic:
  private void updateCurrentPositions() {
    // Sensor angles should be divided by the appropriate ticks per radian
    currentJointPositions.shoulder = (shoulderMotor.getSelectedSensorPosition()/shoulderTicksPerRadian);
    currentJointPositions.elbow = (elbowMotor.getSelectedSensorPosition()/elbowTicksPerRadian); 
    currentJointPositions.wrist = (wristMotor.getSelectedSensorPosition()/wristTicksPerRadian);
  }

  // Runs in periodic:
  private void updateAbsolutePositions() {
    absoluteJointPositions.shoulder = (shoulderEncoder.getPosition() * Math.PI / 180 - shoulderAbsoluteOffset);
    absoluteJointPositions.elbow = elbowEncoder.getPosition() * Math.PI / 180 - elbowAbsoluteOffset;
    // No separate absolute sensor for wrist:
    absoluteJointPositions.wrist = wristMotor.getSelectedSensorPosition()/wristTicksPerRadian - wristAbsoluteOffset;
  }

  public void disableMotors() {
    motorsEnabled = false;
    armTrajectory = null;
  }

  public void enableMotors() {
    motorsEnabled = true;
  }


  public boolean areMotorsEnabled() {
    return motorsEnabled;
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


    // Update all of the positions from sensors: Must call this every time through this looop.
    updateCurrentPositions();
    updateAbsolutePositions();
    updateGravityCompensation(); // Must be called every time after updates of positions.
    
    
    double trajectoryTime = ((double)System.currentTimeMillis() / 1000.0);
  
    // If we have a trajectory, then get references from the trajectory:
    if (armTrajectory != null) {
      // Update reference positions and velocities:
      armTrajectory.getPositionsAtTime(trajectoryTime, referencePositions);
      armTrajectory.getVelocitiesAtTime(trajectoryTime, referenceVelocities);
    }
    else{
      //referencePositions.shoulder += referenceVelocities.shoulder * 0.02;
      //referencePositions.elbow += referenceVelocities.elbow * 0.02;
      //referencePositions.wrist += referenceVelocities.wrist * 0.02;
    }

    

    if (motorsEnabled == true)  {
      // Send motor command if motors are enabled, otherwise send zero power.
      shoulderMotor.set(ControlMode.Position, referencePositions.shoulder * shoulderTicksPerRadian, DemandType.ArbitraryFeedForward, gravityCompensation[0]);
      elbowMotor.set(ControlMode.Position, referencePositions.elbow * elbowTicksPerRadian, DemandType.ArbitraryFeedForward, gravityCompensation[1]);
      wristMotor.set(ControlMode.Position, referencePositions.wrist * wristTicksPerRadian, DemandType.ArbitraryFeedForward, gravityCompensation[2]);
    } else {
      shoulderMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, gravityCompensation[0]);
      elbowMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, gravityCompensation[1]);
      wristMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, gravityCompensation[2]); 
    }

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

    // Output angles:
    SmartDashboard.putNumber("Shoulder Angle", currentJointPositions.shoulder);
    SmartDashboard.putNumber("Elbow Angle", currentJointPositions.elbow);
    SmartDashboard.putNumber("Wrist Angle", currentJointPositions.wrist);
    SmartDashboard.putNumber("Shoulder Gravity", gravityCompensation[0]);
    SmartDashboard.putNumber("Elbow Gravity", gravityCompensation[1]);
    SmartDashboard.putNumber("Wrist Gravity", gravityCompensation[2]);

    SmartDashboard.putNumber("Shoulder Absolute Angle", absoluteJointPositions.shoulder);
    SmartDashboard.putNumber("Elbow Absolute Angle", absoluteJointPositions.elbow);
    SmartDashboard.putNumber("Wrist Absolute Anglge", absoluteJointPositions.wrist);

    SmartDashboard.putBoolean("Is Elbow Magnet Healthy", isElbowMagnetHealthy);
    SmartDashboard.putBoolean("Is Shoulder Magnet Healthy", isShoulderMagnetHealthy);

    SmartDashboard.putNumber("Shoulder Reference", referencePositions.shoulder);
    SmartDashboard.putNumber("Elbow Reference", referencePositions.elbow);
    SmartDashboard.putNumber("Wrist Reference", referencePositions.wrist);
   
    SmartDashboard.putNumber("Reference Shoulder velocity", referenceVelocities.shoulder);
    SmartDashboard.putNumber("Reference Elbow velocity", referenceVelocities.elbow);
    SmartDashboard.putNumber("Reference wrist velocity", referenceVelocities.wrist);
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
  
  }

  public void setUpMotors(){
    ErrorCode shoulderError = shoulderMotor.configFactoryDefault();
    ErrorCode elbowError = elbowMotor.configFactoryDefault();
    ErrorCode wristError = wristMotor.configFactoryDefault();

    updateAbsolutePositions();

    // Shoulder Motor Setup:
    shoulderMotor.setNeutralMode(NeutralMode.Brake);
    shoulderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 28, 0.1));
    shoulderMotor.setSensorPhase(true);
    //shoulderMotor.setInverted(true);

    shoulderMotor.config_kP(0, 0.5, 100);
    shoulderMotor.config_kI(0, 0.1, 100);
    //shoulderMotor.config_kP(0, 0, 100);
    //shoulderMotor.config_kI(0, 0, 100);
    shoulderMotor.config_kD(0, 0.04, 100);
    shoulderMotor.config_kF(0, 0, 100);
    shoulderMotor.configMaxIntegralAccumulator(0, 500, 100);
    shoulderMotor.setIntegralAccumulator(0, 0, 100);
    isShoulderInitialized = false;

    ErrorCode errorShoulder = shoulderMotor.setSelectedSensorPosition(absoluteJointPositions.shoulder * shoulderTicksPerRadian, 0, 400);
    if (errorShoulder != ErrorCode.OK) {
      System.out.println("Shoulder: set selected sensor position failed.");
    }

    // Elbow Motor Setup:
    elbowMotor.setNeutralMode(NeutralMode.Brake);
    elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 28, 0.1));
    elbowMotor.config_kP(0, 0.5, 100);
    elbowMotor.config_kI(0, 0.1, 100);
    //elbowMotor.config_kP(0, 0, 100);
    //elbowMotor.config_kI(0, 0, 100);
    elbowMotor.config_kD(0, 0.04, 100);
    elbowMotor.config_kF(0, 0, 100);
    elbowMotor.configMaxIntegralAccumulator(0, 500, 100);
    elbowMotor.setIntegralAccumulator(0);

    ErrorCode errorElbow = elbowMotor.setSelectedSensorPosition(absoluteJointPositions.elbow * elbowTicksPerRadian, 0, 400);
    if (errorElbow != ErrorCode.OK) {
      System.out.println("Elbow: set selected sensor position failed,");
    }


    // Wrist Motor Setup:
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 0.1));

    wristMotor.config_kP(0, 0.35, 100);
    wristMotor.config_kI(0, 0.0, 100);
    //wristMotor.config_kP(0, 0, 100);
    //wristMotor.config_kI(0, 0, 100);
    wristMotor.config_kD(0, 0.01, 100);
    wristMotor.config_kF(0, 0, 100);
    wristMotor.configMaxIntegralAccumulator(0, 500, 100);
    wristMotor.setIntegralAccumulator(0);

    ErrorCode errorWrist = wristMotor.setSelectedSensorPosition(wristAbsoluteOffset * wristTicksPerRadian, 0, 400);
    if (errorWrist != ErrorCode.OK) {
      System.out.println("Wrist: set selected sensor position failed.");
    }

    // First time update:
    updateCurrentPositions();
    updateAbsolutePositions();

    SmartDashboard.putBoolean("Is errorElbow returned", errorElbow != ErrorCode.OK);
    SmartDashboard.putBoolean("Is errorShoulder returned", errorShoulder != ErrorCode.OK);
    SmartDashboard.putBoolean("Is errorWrsit returned", errorWrist != ErrorCode.OK);

    SmartDashboard.putNumber("Shoulder Angle on init", currentJointPositions.shoulder);
    SmartDashboard.putNumber("Elbow Angle on init", currentJointPositions.elbow);
    SmartDashboard.putNumber("Wrist Angle on Init", currentJointPositions.wrist);

    SmartDashboard.putNumber("Shoulder Absolute Angle on init", absoluteJointPositions.shoulder);
    SmartDashboard.putNumber("Elbow Absolute Angle on init", absoluteJointPositions.elbow);
    SmartDashboard.putNumber("Wrist Absolute Angle on Init", absoluteJointPositions.wrist);

  }
  
  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  // This methods returns the angle of each joint
  public JointPositions getJointAngles() {
    //return new JointPositions(shoulderRawAngle + shoulderOffset, elbowRawAngle + elbowOffset - shoulderRawAngle);
    return new JointPositions(currentJointPositions.shoulder, currentJointPositions.elbow, currentJointPositions.wrist);
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
    return new JointPositions(absoluteJointPositions.shoulder, absoluteJointPositions.elbow, absoluteJointPositions.wrist);
  }

  // This method returns the maximum angles of joints
  public JointPositions getMaxAngles(){
    return maxAngles;
  }

  // This method returns the minimum angles of joints
  public JointPositions getMinAngles(){ 
    return minAngles;
  }

  public void setArmTrajectories(ArrayList<JointWaypoints> waypoints, JointVelocities maxVelocities, double maxAcceleration){
    armTrajectory = new ArmTrajectory(waypoints, maxVelocities, maxAcceleration);
    motorsEnabled = true; // We're moving when given a trajectory.
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
      if(speed.shoulder < 0){
        speed.shoulder = 0;
      }
    }
    if(currentJointPositions.shoulder > 0.98){ //1.0
      if(speed.shoulder > 0){
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
    if(currentJointPositions.wrist < -1.2){ //-2.82
      if(speed.wrist < 0){
        speed.wrist = 0;
      }
    }
    if(currentJointPositions.wrist > 1.49){
      if(speed.wrist > 0){
        speed.wrist = 0;
      }
    }


    //TODO add limits on wrist rotation
    
    referenceVelocities.shoulder = speed.shoulder;
    referenceVelocities.elbow = speed.elbow;
    referenceVelocities.wrist = speed.wrist;
    if(!motorsEnabled || armTrajectory != null){
      updateCurrentPositions();
      System.out.println("Positions updated");
      referencePositions = currentJointPositions;
    }
    armTrajectory = null;
    motorsEnabled = true;
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

  // Use states to compute gravity compensation values:
  private void updateGravityCompensation() {
    double upperLinkX = Math.cos(absoluteJointPositions.shoulder) * upperArmLength;
    double forearmX = Math.cos(absoluteJointPositions.elbow + absoluteJointPositions.shoulder) * forearmLength;
    double wristX = Math.cos(absoluteJointPositions.wrist + absoluteJointPositions.elbow + absoluteJointPositions.shoulder) * wristLength;

    // Compute torques baed on X offset from previous axis * mass and then scale by a per-joint gain and an overal gain for gravity compensation.
    gravityCompensation[2] = -gravityCompensationGain * gravityCompensationWristGain * (wristX * wristMass); // Wrist compensation.
    gravityCompensation[1] = -gravityCompensationGain * gravityCompensationElbowGain * ((wristX + forearmX) * wristMass + forearmX * foreArmMass); // Elbow compensation.
    gravityCompensation[0] = gravityCompensationGain * gravityCompensationShoulderGain * ((wristX + forearmX + upperLinkX) * wristMass + (forearmX + upperLinkX)* foreArmMass + upperLinkX * upperArmMass); // Shoulder compensation
  }
}