// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  public ArrayList<AprilTag> getVisibleTags()
  {
    return null;
  }
  
   /**
   * Gets the decoded ID of the tag.
   *
   * @return Decoded ID
   */

  public int getId() {
    return m_id;
  }

  /**
   * Gets how many error bits were corrected. Note: accepting large numbers of corrected errors
   * leads to greatly increased false positive rates. NOTE: As of this implementation, the detector
   * cannot detect tags with a hamming distance greater than 2.
   *
   * @return Hamming distance (number of corrected error bits)
   */
  public int getHamming() {
    return m_hamming;
  }

  /**
   * Gets a measure of the quality of the binary decoding process: the average difference between
   * the intensity of a data bit versus the decision threshold. Higher numbers roughly indicate
   * better decodes. This is a reasonable measure of detection accuracy only for very small tags--
   * not effective for larger tags (where we could have sampled anywhere within a bit cell and still
   * gotten a good detection.)
   *
   * @return Decision margin
   */
  public float getDecisionMargin() {
    return m_decisionMargin;
  }

  /**
   * Gets the 3x3 homography matrix describing the projection from an "ideal" tag (with corners at
   * (-1,1), (1,1), (1,-1), and (-1, -1)) to pixels in the image.
   *
   * @return Homography matrix data
   */
  @SuppressWarnings("PMD.MethodReturnsInternalArray")
  public double[] getHomography() {
    return m_homography;
}
