// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public class JointPositions{
    double shoulder;
    double elbow;
  }

  public class JointVelocities{
    double shoulder;
    double elbow;
  }

  public class CartesianPosition{
    double x;
    double z;
    double pitch;
  }

  /** Creates a new Arm. */
  //Set height limiter
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
  
  }

  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  // This methods returns the angle of each joint
  public JointPositions getJointAngles(){
    return null;
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

  }

  // This method sets a target speed for joints
  public void setJointVelocities(JointVelocities speed){

  }

  // This method returns the position of clawgiven different joint angles
  public CartesianPosition getCartesianPosition(JointPositions positions){
    return null;
  }

  // This method returns the joint angles given a position of the claw
  public JointPositions getInverseKinematics(CartesianPosition clawPose){
    return null;
  }
}
