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
  private double cameraOffset;

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
    int numTags = tagData.length/22;
    // Reset search variables for closest to empty:
    closestID = -1;
    closestDistance = 9999.0;
    closestPose = null; 

    for (int i = 0; i < numTags; i = i +1){

      // Checks to see if the tag ID is between 1-8 (by excluding 0 and 9) to reduce false positives.
      // If not, it won't send information to the tagData array.

      if(tagData[i*22].intValue() > 0 && tagData[i*22].intValue() < 9){

      double[] homography = new double[9];
      double[] corners = new double[8];
      //Doesn't use homography anymore
      homography[0] = tagData[i*22 + 13].doubleValue();
      homography[1] = tagData[i*22 + 14].doubleValue();
      homography[2] = tagData[i*22 + 15].doubleValue();
      homography[3] = tagData[i*22 + 16].doubleValue();
      homography[4] = tagData[i*22 + 17].doubleValue();
      homography[5] = tagData[i*22 + 18].doubleValue();
      homography[6] = tagData[i*22 + 19].doubleValue();
      homography[7] = tagData[i*22 + 20].doubleValue();
      homography[8] = tagData[i*22 + 21].doubleValue();
      corners[0] = tagData[i*22 + 5].doubleValue();
      corners[1] = tagData[i*22 + 6].doubleValue();
      corners[2] = tagData[i*22 + 7].doubleValue();
      corners[3] = tagData[i*22 + 8].doubleValue();
      corners[4] = tagData[i*22 + 9].doubleValue();
      corners[5] = tagData[i*22 + 10].doubleValue();
      corners[6] = tagData[i*22 + 11].doubleValue();
      corners[7] = tagData[i*22 + 12].doubleValue();
      AprilTagDetection detection = new AprilTagDetection("16h5", tagData[i*22 + 0].intValue(),
      tagData[i*22 + 1].intValue(), tagData[i*22 + 2].floatValue(), homography, 
      tagData[i*22 + 3].doubleValue(), tagData[i*22 + 4].doubleValue(), corners);
      detections.add(detection);
      //Transform3d transform = poseEstimator.estimate(detection);
      //AprilTagPoseEstimate poseEstimate = poseEstimator.estimateOrthogonalIteration(detection, 50);
      //Transform3d transform = poseEstimator.estimate(detection);
      Transform3d transform = poseEstimator.estimateHomography(detection);

      // Set rotation to 0, lied to robot. 
      // TODO: Go back and do transform calibration after Week 1

      // if (transform.getZ() < 0.0){
      //   // transform = new Transform3d(new Translation3d(-transform.getZ(), transform.getX(), transform.getY()), 
      //   // new Rotation3d(transform.getRotation().getX(), 1 + transform.getRotation().getZ(), -transform.getRotation().getY()));

      //   transform = new Transform3d(new Translation3d(-transform.getZ(), transform.getX(), transform.getY()), 
      //   new Rotation3d(0, 0,0));

      // }else{
      //   // transform = new Transform3d(new Translation3d(transform.getZ(), -transform.getX(), -transform.getY()), 
      //   // new Rotation3d(-transform.getRotation().getX(), -transform.getRotation().getZ(), Math.PI + transform.getRotation().getY()));

      //   transform = new Transform3d(new Translation3d(-transform.getZ(), transform.getX(), transform.getY()), 
      //   new Rotation3d(0, 0, 0));
      // }

      // Location of tag in camera coords rotated into robot orientation as a pose3d. Tranform3d and Pose3d are compatiable with certain conditions

      
      //set transform to 0

      transform = new Transform3d(new Translation3d(transform.getZ(), -transform.getX(), -transform.getY()), 
      new Rotation3d(0,0,0));
      //new Rotation3d(transform.getRotation().getZ(), -transform.getRotation().getX(), -transform.getRotation().getY()));

      Pose3d tagPose = new Pose3d(transform.getTranslation(), transform.getRotation());
      //Pose3d cameraPose = new Pose3d(cameraTransform.getTranslation(), cameraTransform.getRotation());
      //tagPose = tagPose.plus(transform);

      // Swapping order of tranformation due to ambiguous docs
      tagPose = tagPose.plus(cameraTransform);

      //Keep track of the closest tag while we're running through the list.
      if (tagPose.getTranslation().getNorm() < closestDistance) {
        closestDistance = tagPose.getTranslation().getNorm();
        
        //took out transform to test rotation
        //closestPose = new Pose3d(transform.getTranslation(), transform.getRotation());
        closestPose = tagPose;
        closestID = detection.getId();
      }
      
      tags.add(new AprilTag(detection.getId(), tagPose));

      }
      
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

  // Return ArrayList of tags
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


public String getDiagnostics() {
    return "";
}
  
}
