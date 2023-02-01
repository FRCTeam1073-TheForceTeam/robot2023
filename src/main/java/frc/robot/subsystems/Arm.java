// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
  private TalonFX shoulderMotor, elbowMotor;
  private CANCoder shoulderEncoder, elbowEncoder;

  public class JointPositions{
    double shoulder;
    double elbow;

    public JointPositions(double shoulderAng, double elbowAng){
      shoulder = shoulderAng;
      elbow = elbowAng;
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

  public class Spline{
    public Spline(){

    }
  }

  public class ArmTrajectory{

    private JointPositions startPosition;
    private JointPositions endPosition;
    private ArrayList<JointPositions> waypoints;
    private TrajectoryConfig trajectoryConfig;
    private Spline shoulderTrajectory;
    private Spline elbowTrajectory;

    public ArmTrajectory(JointPositions startPosition, JointPositions endPosition, ArrayList<JointPositions> waypoints,
    double maxVelocity, double maxAcceleration){
      this.startPosition = startPosition;
      this.endPosition = endPosition;
      this.waypoints = waypoints;
      trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration);
      Spline shoulderTrajectory = new Spline();
      Spline elbowTrajectory = new Spline();
    }

    

    public JointPositions getTrajectoryAtTime(double time){
      //placeholder for joint angles
      JointPositions position = new JointPositions(0, 0);
      //getAtX doesn't exist for spline yet
      //position.shoulder = shoulderTrajectory.getAtX(time);
      //position.elbow = elbowTrajectory.getAtX(time);
      return position;
    }
  }

  /** Creates a new Arm. */
  //Set height limiter
  public Arm(){
    shoulderMotor = new TalonFX(0);
    elbowMotor = new TalonFX(0);
    shoulderEncoder = new CANCoder(0);
    elbowEncoder = new CANCoder(0);
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
