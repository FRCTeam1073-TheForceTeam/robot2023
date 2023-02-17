// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OI;

public class TeleopDrive extends CommandBase 
{
  double angleTolerance = 0.05;
  ChassisSpeeds chassisSpeeds;
  Pose2d targetRotation;
  Pose2d robotRotation;
  DriveSubsystem m_driveSubsystem;
  OI m_OI;
  private boolean fieldCentric;
  private boolean parked;
  ChassisSpeeds speeds;

  // Teleop drive velocity scaling:
  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second

  /** Creates a new Teleop. */
  public TeleopDrive(DriveSubsystem ds, OI oi){
    super.setName("Teleop Drive");
    m_driveSubsystem = ds;
    m_OI = oi;
    fieldCentric = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    System.out.println("TeleopDrive: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    double mult1 = 1.0 + (m_OI.getDriverLeftTrigger() * 5);
    double mult2 = 1.0 + m_OI.getDriverRightTrigger();

    //double mult1 = m_OI.getDriverLeftTrigger() * maximumLinearVelocity * 0.475;
    //double mult2 = m_OI.getDriverRightTrigger() * maximumLinearVelocity * 0.475;

    // if (m_OI.getLeftBumper()){
    //   velocityMult *= 0.5; // 50% maximum speed
    //   rotateMult *= 0.5;
    // }
    // else if (m_OI.getRightBumper()){
    //   velocityMult *= 1.0; // Maximum speed
    //   rotateMult *= 1.0;
    // } else {
    //   velocityMult *= 0.1;  // 10% maximum speed.
    //   rotateMult *= 0.1;
    // }

    // Allow driver to zero the drive subsystem heading for field-centric control.
    if(m_OI.getMenuButton()){
      m_driveSubsystem.zeroHeading();
    }

    if(m_OI.getAButton()){
      Rotation2d zeroRotate = new Rotation2d();
      Pose2d zero = new Pose2d(0.0, 0.0, zeroRotate);
      m_driveSubsystem.resetOdometry(zero);
    }

    double leftY = m_OI.getDriverLeftY() * maximumLinearVelocity * 0.1;
    double leftX = m_OI.getDriverLeftX() * maximumLinearVelocity * 0.1;
    double rightX = m_OI.getDriverRightX() * maximumRotationVelocity * 0.1;
    if (Math.abs(leftY) < .35) {leftY = 0;}
    if (Math.abs(leftX) < .35) {leftX = 0;}
    if (Math.abs(rightX) < .35) {rightX = 0;}

    if(m_OI.getXButton()){
      parked = !parked;
    }

    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(leftY * 0.5, leftX * 0.5, rightX); //debug
    if (m_OI.getFieldCentricToggle()){
      fieldCentric = !fieldCentric;
    }

    SmartDashboard.putBoolean("Field Centric", fieldCentric);

    if(parked){
      m_driveSubsystem.parkingBrake();
    }
    else if (fieldCentric){
      //Snap to cardinal directions
      double currentAngle = m_driveSubsystem.getOdometry().getRotation().getRadians() % (2 * Math.PI);
      double cardinalError;
      SmartDashboard.putNumber("Current Angle within 2 pi", currentAngle);

      if(m_OI.getDPad() == 0){
        //cardinalError = Math.abs()
        if(currentAngle == 0){
          rightX = 0;
        }
        if((currentAngle > 0 && currentAngle <= Math.PI) || (currentAngle < -Math.PI)){
          rightX = -.5;
        }
        else if(currentAngle > Math.PI || (currentAngle < 0 && currentAngle >= -Math.PI)){
          rightX = .5;
        }
      }

      if(m_OI.getDPad() == 90){ //90 is 270 on the robot
        if(currentAngle == 3/2 * Math.PI){
          rightX = 0;
        }
        if((Math.abs(currentAngle) > 0 && Math.abs(currentAngle) <= Math.PI / 2) ||
          (Math.abs(currentAngle) > 3/2 * Math.PI && Math.abs(currentAngle) < 2 * Math.PI)){
          rightX = -.5;
        }
        else if(Math.abs(currentAngle) > Math.PI / 2 && Math.abs(currentAngle) <= Math.PI){
          rightX = .5;
        }
      }

      if(m_OI.getDPad() == 180){
        if(currentAngle == Math.PI){
          rightX = 0;
        }
        if(currentAngle > Math.PI || (currentAngle < 0 && currentAngle >= -Math.PI)){
          rightX = -.5;
        }
        else if((currentAngle > 0 && currentAngle <= Math.PI) || (currentAngle < -Math.PI)){
          rightX = .5;
        }
      }

      if(m_OI.getDPad() == 270){ //270 is 90 on the robot
        if(currentAngle == Math.PI / 2){
          rightX = 0;
        }
        if(Math.abs(currentAngle) > Math.PI / 2 && Math.abs(currentAngle) <= Math.PI){
          rightX = -.5;
        }
        else if((Math.abs(currentAngle) > 0 && Math.abs(currentAngle) <= Math.PI / 2) ||
                (Math.abs(currentAngle) > 3/2 * Math.PI && Math.abs(currentAngle) < 2 * Math.PI)){
          rightX = .5;
        }
      }

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        -leftY * mult1 * mult2 * 0.5,
        -leftX * mult1 * mult2 * 0.5,
        -rightX * mult1 * mult2 * 0.5,
        Rotation2d.fromDegrees(m_driveSubsystem.getHeading())); // get fused heading
        m_driveSubsystem.setChassisSpeeds(speeds);
    }
    
    else{
      // Robot centric driving.
      speeds = new ChassisSpeeds();
      speeds.vxMetersPerSecond = -leftY * mult1 * mult2 * 0.5; 
      speeds.vyMetersPerSecond = -leftX * mult1 * mult2 * 0.5; 
      speeds.omegaRadiansPerSecond = -rightX * mult1 * mult2 * 0.5;
      m_driveSubsystem.setChassisSpeeds(speeds); 
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    if (interrupted) {
      System.out.println("TeleopDrive: Interrupted!");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
