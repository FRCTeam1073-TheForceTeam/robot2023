// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OpenMV;

public class AlignToGamePiece extends CommandBase {
  /** Creates a new AlignToAprilTag. */
  private ChassisSpeeds chassisSpeeds;
  private OpenMV openMV;
  private DriveSubsystem drivetrain;
  private String targetType;
  private String selected_type;
  private int selected_x;
  private int selected_y;
  private double selected_area;
  private double selected_confidence;
  private ArrayList<OpenMV.Target> targets;
  private OpenMV.Target target;
  private double distance;
  private double angle;
  private int steerPhase;
  private Pose2d robotPose;
  private Pose2d targetPose;



  public AlignToGamePiece(DriveSubsystem driveSubsystem, OpenMV openMV, String TargetType) {
    this.drivetrain = driveSubsystem;
    this.openMV = openMV;
    this.targetType = targetType;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassisSpeeds = new ChassisSpeeds();
    drivetrain.setChassisSpeeds(chassisSpeeds);
    targets = openMV.getTargets();
    selected_area = 0;
    steerPhase = 0; //first turn to the right angle

    for(int i = 0; i < targets.size(); i++) {
      target = targets.get(i);
      if(target.type == targetType){  //if this is the right type of game piece
        if(target.area > selected_area){ //looking for the biggest area
          selected_type = target.type;
          selected_area = target.area;
          selected_x = target.imagex;
          selected_y = target.imagey;
          selected_confidence = target.confidence;
        } 
      }
    }

    robotPose = drivetrain.getOdometry();
    // calculate angle: tan^-1 (x/y)
    // calculate targetPose, add x, y, and angle to robotPose
    // factor in how far away from target we should be to collect
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = drivetrain.getOdometry();
    if(selected_area == 0) {
      // set ChassisSpeeds to 0 
      steerPhase = 2;
    }
    else if(steerPhase == 0) { 
      //turn towards target
      //set ChassisSpeeds omega to ___
      //if(robotPose angle is >= targetPose angle) {
        // set ChassisSpeeds omega to 0 and x to ___
        steerPhase = 1;
      }
      
    else {
      //drive toward target
      // if really close to targer pose, then set ChassisSpeeds all to 0
      steerPhase = 2;
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set ChassisSpeeds to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return steerPhase == 2;
  }
}
