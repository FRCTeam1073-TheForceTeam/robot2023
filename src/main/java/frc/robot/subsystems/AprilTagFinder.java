// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  private AprilTagPoseEstimator poseEstimator;
  private ArrayList<AprilTag> tags;
  private Transform3d cameraTransform;
  private DriveSubsystem driveSubsystem;

  /** Creates a new AprilTag. */
  public AprilTagFinder(DriveSubsystem ds) {
    driveSubsystem = ds;
    detections = new ArrayList<AprilTagDetection>();
    apriltagNetwork = NetworkTableInstance.getDefault().getTable("Vision");
    apriltagEntry = apriltagNetwork.getEntry("Tags1");
    AprilTagPoseEstimator.Config config = new AprilTagPoseEstimator.Config(0.1524, 1385/1.75, 1385/1.75, 320, 240);
    poseEstimator = new AprilTagPoseEstimator(config);
    tags = new ArrayList<AprilTag>();
    cameraTransform = new Transform3d(new Translation3d(0.1143, -0.1397, 0.51435), new Rotation3d(0, 0, 0)); //location of the camera in robot cordinates
    //cameraTransform = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, Math.PI/2.0, 0)); //location of the camera in robot cordinates
    //cameraTransform = new Transform3d();
  }
   
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Number[] tagData = apriltagEntry.getNumberArray(new Number[0]);
    detections.clear();
    tags.clear();
    int numTags = tagData.length/23;
    double closestDistance = 9999.0;
    int closestTag = -1;
    for (int i = 0; i < numTags; i = i +1){
      double[] homography = new double[9];
      double[] corners = new double[8];
      homography[0] = tagData[i*23 + 4].doubleValue();
      homography[1] = tagData[i*23 + 5].doubleValue();
      homography[2] = tagData[i*23 + 6].doubleValue();
      homography[3] = tagData[i*23 + 7].doubleValue();
      homography[4] = tagData[i*23 + 8].doubleValue();
      homography[5] = tagData[i*23 + 9].doubleValue();
      homography[6] = tagData[i*23 + 10].doubleValue();
      homography[7] = tagData[i*23 + 11].doubleValue();
      homography[8] = tagData[i*23 + 12].doubleValue();
      /*homography[0] = tagData[i*23 + 4].doubleValue();
      homography[1] = tagData[i*23 + 7].doubleValue();
      homography[2] = tagData[i*23 + 10].doubleValue();
      homography[3] = tagData[i*23 + 5].doubleValue();
      homography[4] = tagData[i*23 + 8].doubleValue();
      homography[5] = tagData[i*23 + 11].doubleValue();
      homography[6] = tagData[i*23 + 6].doubleValue();
      homography[7] = tagData[i*23 + 9].doubleValue();
      homography[8] = tagData[i*23 + 12].doubleValue();*/
      corners[0] = tagData[i*23 + 15].doubleValue();
      corners[1] = tagData[i*23 + 16].doubleValue();
      corners[2] = tagData[i*23 + 17].doubleValue();
      corners[3] = tagData[i*23 + 18].doubleValue();
      corners[4] = tagData[i*23 + 19].doubleValue();
      corners[5] = tagData[i*23 + 20].doubleValue();
      corners[6] = tagData[i*23 + 21].doubleValue();
      corners[7] = tagData[i*23 + 22].doubleValue();
      AprilTagDetection detection = new AprilTagDetection("16h5", tagData[i*23 + 0].intValue(),
      tagData[i*23 + 1].intValue(), tagData[i*23 + 2].floatValue(), homography, 
      tagData[i*23 + 13].doubleValue(), tagData[i*23 + 14].doubleValue(), corners);
      detections.add(detection);
      //Transform3d transform = poseEstimator.estimate(detection);
      //AprilTagPoseEstimate poseEstimate = poseEstimator.estimateOrthogonalIteration(detection, 50);
      Transform3d transform = poseEstimator.estimate(detection);
      if (transform.getZ() < 0.0){
        transform = new Transform3d(new Translation3d(-transform.getZ(), transform.getX(), transform.getY()), 
        new Rotation3d(-transform.getRotation().getZ(), transform.getRotation().getX(), transform.getRotation().getY()));

      }else{
        transform = new Transform3d(new Translation3d(transform.getZ(), -transform.getX(), -transform.getY()), 
        new Rotation3d(transform.getRotation().getZ(), -transform.getRotation().getX(), -transform.getRotation().getY()));
      }
      //Keep track of the closest tag
      if (transform.getTranslation().getNorm() < closestDistance) {
        closestDistance = transform.getTranslation().getNorm();
        closestTag = i;
      }
      //Pose3d pose = new Pose3d();
      //Pose3d pose = driveSubsystem.get3dOdometry();
      Pose3d tagPose = new Pose3d(cameraTransform.getTranslation(), cameraTransform.getRotation());
      //Pose3d tagPose = new Pose3d(transform.getTranslation(), transform.getRotation());
      //Pose3d pose = new Pose3d();
      //Transform3d odoTransform = new Transform3d(driveSubsystem.get3dOdometry().getTranslation(), driveSubsystem.get3dOdometry().getRotation());
      
      tagPose = tagPose.plus(transform);
      //tagPose = tagPose.plus(cameraTransform);
      //pose = pose.plus(odoTransform);

      //pose = pose.plus(transform);
      tags.add(new AprilTag(detection.getId(), tagPose));

    }
    //End of tag proccesing loop
    SmartDashboard.putNumber("AprilTag.numTags", numTags);
    if (closestTag >= 0){
      SmartDashboard.putNumber("Closest ID", tags.get(closestTag).ID);
      SmartDashboard.putNumber("Closest Distance", closestDistance);
      SmartDashboard.putNumber("Closest X", tags.get(closestTag).pose.getX());
      SmartDashboard.putNumber("Closest Y", tags.get(closestTag).pose.getY());
      SmartDashboard.putNumber("Closest Z", tags.get(closestTag).pose.getZ());
      SmartDashboard.putNumber("Tag Rotation X", tags.get(closestTag).pose.getRotation().getX());
      SmartDashboard.putNumber("Tag Rotation Y", tags.get(closestTag).pose.getRotation().getY());
      SmartDashboard.putNumber("Tag Rotation Z", tags.get(closestTag).pose.getRotation().getZ());
    }
    
  }


  public ArrayList<AprilTag> getTags(){
    return tags;
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
  public ArrayList<AprilTagDetection> getVisibleTags()
  {
    return detections;
  }
  



}
