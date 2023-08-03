// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AprilTagFinder;

public class AprilTagOdometry extends SubsystemBase {

  public class AprilTagPosition {
    double x;
    double y;
    double z;
    int id;
  }

  /** Creates a new AprilTagOdometry. */
  public AprilTagOdometry() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
