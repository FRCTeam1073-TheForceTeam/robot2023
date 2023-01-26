// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTag extends SubsystemBase {
  /** Creates a new AprilTag. */
  public AprilTag() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // This method returns the position of the robot based on information from AprilTags
  public Pose2d getFieldPose(){
    return new Pose2d();
  }

  // This method returns the position of the passed in AprilTag id relative to the Robot
  public Transform2d getRelativePose(int id){
    return new Transform2d();
  }

  //This method returns an array of all the visible AprilTag ids
  public int[] getVisibleTags(){
    return null;
  }

}
