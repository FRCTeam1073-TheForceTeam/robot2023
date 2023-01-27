// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagFinder extends SubsystemBase {
  /** Creates a new AprilTag. */
  public AprilTagFinder() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
  
  }

  // This method returns the position of the robot based on information from AprilTags
  public Pose2d getFieldPose(){
    return new Pose2d();
  }

  // This method returns the position of the passed in AprilTag id relative to the Robot
  public Pose3d getRelativePose(int id){
    //getTagPose method in AprilTagFieldLayout
    return new Pose3d();
  }

  //This method returns an array of all the visible AprilTag ids
  public AprilTag[] getVisibleTags(){
    return null;
  }

}
