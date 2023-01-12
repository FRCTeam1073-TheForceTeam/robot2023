// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class SwerveModuleConfig 
{
    public Translation2d position = new Translation2d(0,0);
    public double tickPerMeter = 1000;
    public double tickPerRadian = 1000;
    public double steerAngleOffset = 0;
    public double steerCurrentLimit = 10;
    public double driveCurrentLimit = 20;
    public double steerCurrentThreshold = 12;
    public double driveCurrentThreshold = 22;
    public double steerCurrentThresholdTime = 0.1;
    public double driveCurrentThresholdTime = 0.25;
    public double steerP = 0;
    public double steerI = 0;
    public double steerD = 0;
    public double steerF = 0;
    public double driveP = 0;
    public double driveI = 0;
    public double driveD = 0;
    public double driveF = 0;    

    public SwerveModuleConfig()
    {

    }
}
