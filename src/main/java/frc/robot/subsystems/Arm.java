// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
  private TalonFX shoulderMotor, elbowMotor;
  private CANCoder shoulderEncoder, elbowEncoder;
  //set variables below to correct lengths
  public final double upperArm = 0;
  public final double foreArm = 0;

  public class JointPositions{
    double shoulder;
    double elbow;

    public JointPositions(double shoulderAng, double elbowAng){
      shoulder = shoulderAng;
      elbow = elbowAng;
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
    double shoulder;
    double elbow;

    public JointVelocities(double shoulderVel, double elbowVel){
      shoulder = shoulderVel;
      elbow = elbowVel;
    }
  }

  public class CartesianPosition{
    double x;
    double z;
    double pitch;
  }


  public class ArmTrajectory{

    private ArrayList<JointWaypoints> waypoints;
    private TrajectoryConfig trajectoryConfig;
    private PolynomialSplineFunction elbowSplines;
    private PolynomialSplineFunction shoulderSplines;

    public ArmTrajectory(ArrayList<JointWaypoints> waypoints, double maxVelocity, double maxAcceleration){
      this.waypoints = waypoints;
      trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration);
      var interpolator = new SplineInterpolator();
      double[] elbowPositions = new double[waypoints.size()];
      double[] shoulderPositions = new double[waypoints.size()];
      double[] times = new double[waypoints.size()];

      for(int i = 0; i < waypoints.size(); i++){
        elbowPositions[i] = waypoints.get(i).elbow;
        shoulderPositions[i] = waypoints.get(i).shoulder;
        times[i] = waypoints.get(i).time;
      }

      elbowSplines = interpolator.interpolate(elbowPositions, times);
      shoulderSplines = interpolator.interpolate(shoulderPositions, times);
    }
/* 
    public double getTimeForMotion(){
      
    }

    public double timeBetweenPoints(JointPositions firstPoint, JointPositions secondPoint){

    }
*/
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
    setUpMotor(shoulderMotor, shoulderEncoder);
    setUpMotor(elbowMotor, elbowEncoder);

    shoulderMotor.config_kP(0, 0);
    shoulderMotor.config_kI(0, 0);
    shoulderMotor.config_kD(0, 0);
    shoulderMotor.config_kF(0, 0);
    shoulderMotor.configMaxIntegralAccumulator(0, 0);
    shoulderMotor.setIntegralAccumulator(0);

    elbowMotor.config_kP(0, 0);
    elbowMotor.config_kI(0, 0);
    elbowMotor.config_kD(0, 0);
    elbowMotor.config_kF(0, 0);
    elbowMotor.configMaxIntegralAccumulator(0, 0);
    elbowMotor.setIntegralAccumulator(0);
  }

  @Override
  public void periodic(){
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Angle", getJointAngles().shoulder);
    SmartDashboard.putNumber("Elbow Angle", getJointAngles().elbow);
  }

  // Initialize preferences for this class:
  public static void initPreferences(){
  
  }

  public void setUpMotor(TalonFX motor, CANCoder encoder){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configRemoteFeedbackFilter(encoder, 0);
    motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    motor.setSensorPhase(true);
  }
  
  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  // This methods returns the angle of each joint
  public JointPositions getJointAngles(){
    return new JointPositions(shoulderMotor.getSelectedSensorPosition()/1000, elbowMotor.getSelectedSensorPosition()/1000);
  }

  // This method returns the maximum angles of joints
  public JointPositions getMaxAngles(){
    return null;
  }

  // This method returns the minimum angles of joints
  public JointPositions getMinAngles(){
    return null;
  }

  // This method sets a target angle for joints
  public void setTargetAngle(JointPositions target){
    shoulderMotor.set(ControlMode.Position, target.shoulder);
    elbowMotor.set(ControlMode.Position, target.elbow);
  }

  // This method sets a target speed for joints
  public void setJointVelocities(JointVelocities speed){
    shoulderMotor.set(ControlMode.Velocity, speed.shoulder);
    elbowMotor.set(ControlMode.Velocity, speed.elbow);
  }

  // This method returns the position of claw given different joint angles
  public CartesianPosition getCartesianPosition(JointPositions positions){
    return null;
  }

  // This method returns the joint angles given a position of the claw
  public JointPositions getInverseKinematics(CartesianPosition clawPose){
    return null;
  }
}
