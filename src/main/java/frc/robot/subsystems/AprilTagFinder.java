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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

public class AprilTagFinder extends SubsystemBase {
  
  //Creates
  private ArrayList<AprilTagDetection> detections;
  private NetworkTable apriltagNetwork;
  private NetworkTableEntry apriltagEntry;
  private AprilTagPoseEstimator poseEstimator;
  private ArrayList<AprilTag> tags;
  private Transform3d cameraTransform;
  private DriveSubsystem driveSubsystem;
  private String tableName;
  private int closestID; // ID of closest tag or -1.
  private double closestDistance;
  private Pose3d closestPose; // Pose of closest tag or null;

  /** Creates a new AprilTag. */
  public AprilTagFinder(DriveSubsystem ds, String tableName, Transform3d cameraTransform) {
    driveSubsystem = ds;
    this.tableName = tableName;
    this.cameraTransform = cameraTransform;
    detections = new ArrayList<AprilTagDetection>();
    //apriltagNetwork = NetworkTableInstance.getDefault().getTable("Vision");
    apriltagNetwork = NetworkTableInstance.getDefault().getTable(tableName);
    apriltagEntry = apriltagNetwork.getEntry("Tags1");
    // This depends on the camera type, lens focal length and resolution used on vision co-processor.
    AprilTagPoseEstimator.Config config = new AprilTagPoseEstimator.Config(0.1524, 333.3, 333.3, 320, 180);
    poseEstimator = new AprilTagPoseEstimator(config);
    tags = new ArrayList<AprilTag>();

    // Initialize closest values:
    closestID = -1; // None.
    closestDistance = 99999.0; // Far.
    closestPose = null; // None.
  }
   
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Number[] tagData = apriltagEntry.getNumberArray(new Number[0]);
    detections.clear();
    tags.clear();
    int numTags = tagData.length/23;
    // Reset search variables for closest to empty:
    closestDistance = 9999.0;
    closestID = -1;
    closestPose = null; 
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
        new Rotation3d(transform.getRotation().getX(), 1 + transform.getRotation().getZ(), -transform.getRotation().getY()));

      }else{
        transform = new Transform3d(new Translation3d(transform.getZ(), -transform.getX(), -transform.getY()), 
        new Rotation3d(-transform.getRotation().getX(), -transform.getRotation().getZ(), Math.PI + transform.getRotation().getY()));
      }

      //Keep track of the closest tag while we're running through the list.
      if (transform.getTranslation().getNorm() < closestDistance) {
        closestDistance = transform.getTranslation().getNorm();
        closestPose = new Pose3d(transform.getTranslation(), transform.getRotation());
        closestID = detection.getId();
      }
      
      Pose3d tagPose = new Pose3d(cameraTransform.getTranslation(), cameraTransform.getRotation());
      
      tagPose = tagPose.plus(transform);

      tags.add(new AprilTag(detection.getId(), tagPose));
    }

    //End of tag proccesing loop
    SmartDashboard.putNumber(String.format("%s/NumTags", tableName), numTags);
    if (closestID >= 0 && closestPose != null) {
      SmartDashboard.putNumber(String.format("%s/Closest ID", tableName), closestID);
      SmartDashboard.putNumber(String.format("%s/Closest Distance", tableName), closestDistance);
      SmartDashboard.putNumber(String.format("%s/Closest X", tableName), closestPose.getX());
      SmartDashboard.putNumber(String.format("%s/Closest Y", tableName), closestPose.getY());
      SmartDashboard.putNumber(String.format("%s/Closest Z", tableName), closestPose.getZ());
      SmartDashboard.putNumber(String.format("%s/Closest Rotation X", tableName), closestPose.getRotation().getX());
      SmartDashboard.putNumber(String.format("%s/Closest Rotation Y", tableName), closestPose.getRotation().getY());
      SmartDashboard.putNumber(String.format("%s/Closest Rotation Z", tableName), closestPose.getRotation().getZ());
    } else {
      SmartDashboard.putNumber(String.format("%s/Closest ID", tableName), -1);
      SmartDashboard.putNumber(String.format("%s/Closest Distance", tableName), 99999.0);
      SmartDashboard.putNumber(String.format("%s/Closest X", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/Closest Y", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/Closest Z", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/Closest Rotation X", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/Closest Rotation Y", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/Closest Rotation Z", tableName), 0.0);
    }
    
  }

  public ArrayList<AprilTag> getTags(){
    return tags;
  }

  // Return april tag with given ID: returns null if ID not currently found/tracked.
  public AprilTag getTagByID(int ID) {
    for (int i = 0; i < tags.size(); i++) {
      if (tags.get(i).ID == ID)
        return tags.get(i);
    }
    return null; // Fell through loop did not find the ID we were looking for.
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
  
  }

  // This method returns the position of the robot based on information from AprilTags, null if no april tags are seen.
  public Pose2d getFieldPose() {
    if (tags.size() > 0) {
    // TODO: Implement this method when needed, requires map data of tag location by ID, etc.
      return null; // Until we implement it.
    } else {
      return null;
    }
  }

  // This method returns the position of the passed in AprilTag id relative to the Robot or null if tag is not tracked.
  public Pose3d getRelativePose(int id) {
    // getTagPose method in AprilTagFieldLayout
    for (int i = 0; i < tags.size(); i++) {
      if (tags.get(i).ID == id) {
        return tags.get(i).pose;
      }
    }
    return null;
  }

  // This will return -1 if there is no tag detected.
  public int getClosestID() {
    return closestID;
  }

  // This will return 9999.0 if there is no tag detected.
  public double getClosestDistance() {
    return closestDistance; 
  }

  // THIS WILL RETURN NULL if there is no tag detected.
  public Pose3d getClosestPose() {
    return closestPose;
  }

  //This method returns an array of all the visible AprilTag ids
  public ArrayList<AprilTagDetection> getDetections()
  {
    return detections;
  }
  
}
