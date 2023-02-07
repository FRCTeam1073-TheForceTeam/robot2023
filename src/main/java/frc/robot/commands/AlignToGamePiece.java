// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OpenMV;

public class AlignToGamePiece extends CommandBase {
  /** Creates a new AlignToGamePiece. */
  private ChassisSpeeds chassisSpeeds;
  private OpenMV openMV;
  private DriveSubsystem drivetrain;
  private String targetType;
  private double selected_x;
  private double selected_y;
  private double selected_area;
  private double selected_confidence;
  private ArrayList<OpenMV.Target> targets;
  private OpenMV.Target target;
  private double travelDistance;
  private double armOffset;
  private double targetAngle;
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
          selected_area = target.area;
          selected_x = (double)target.targetx;
          selected_y = (double)target.targety;
          selected_confidence = target.confidence;
        } 
      }
    }

    robotPose = drivetrain.getOdometry();

    // factor in how far away from target we should be to collect
    travelDistance = Math.sqrt(Math.pow(selected_x, 2) + Math.pow(selected_y, 2)) - armOffset;

    // calculate angle: tan^-1 (x/y)
    targetAngle = robotPose.getRotation().getRadians() + Math.atan(selected_y/selected_x);

    // calculate new x and y with armOffset
    selected_x = travelDistance * Math.cos(targetAngle);
    selected_y = travelDistance * Math.sin(targetAngle);

    // calculate targetPose, add x, y, and angle to robotPose
    targetPose = new Pose2d(robotPose.getX() + selected_x, robotPose.getY() + selected_y, new Rotation2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = drivetrain.getOdometry();
    if(selected_area == 0) {
      // set ChassisSpeeds to 0 
      chassisSpeeds = new ChassisSpeeds(0, 0, 0);
      drivetrain.setChassisSpeeds(chassisSpeeds);
      steerPhase = 2;
    }
    else if(steerPhase == 0) { 
      //turn towards target
      //set ChassisSpeeds omega to ___
      chassisSpeeds = new ChassisSpeeds(0, 0, (selected_y)/(Math.abs(selected_y))); // TODO left or right in 
      drivetrain.setChassisSpeeds(chassisSpeeds);

      if(Math.abs(robotPose.getRotation().getRadians() - targetAngle) < 0.1) {
        steerPhase = 1;
      }
    }
    else if(steerPhase == 1) {
      //drive toward target
      // set ChassisSpeeds omega to 0 and x to ___
      chassisSpeeds = new ChassisSpeeds(1, 0, 0);
      drivetrain.setChassisSpeeds(chassisSpeeds);
      if(Math.abs(robotPose.getX() - targetPose.getX()) < 0.1 ) {
        steerPhase = 2;
      }
    }
    else {
      // if really close to targer pose, then set ChassisSpeeds all to 0        
      chassisSpeeds = new ChassisSpeeds(0, 0, 0);
      drivetrain.setChassisSpeeds(chassisSpeeds);
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set ChassisSpeeds to 0
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return steerPhase == 2;
  }
}
