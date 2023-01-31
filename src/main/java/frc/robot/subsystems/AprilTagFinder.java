// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;

public class AprilTagFinder extends SubsystemBase {
  
  //Creates
  private ArrayList<AprilTagDetection> detections;
  private NetworkTable apriltagNetwork;
  private NetworkTableEntry apriltagEntry;

  /** Creates a new AprilTag. */
  public AprilTagFinder() {
    detections = new ArrayList<AprilTagDetection>() ;
    apriltagNetwork = NetworkTable.getInstance().getTable("Vision");
    apriltagEntry = apriltagNetwork.getEntry("Tags1");
  }
   
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Number[]tagData = apriltagEntry.getNumberArray();
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
  public ArrayList<AprilTag> getVisibleTags()
  {
    return null;
  }
  



}
