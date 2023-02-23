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
  private boolean parked = false;
  ChassisSpeeds speeds;

  // Teleop drive velocity scaling:
  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second

  //Snap to position thresholds
  private final static double STOP_THRESHOLD = 30;
  private final static double SLOW_THRESHOLD = 20;

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
    //multiples the angle by a number from 1 to the square root of 20:
    double mult1 = 1.0 + (m_OI.getDriverLeftTrigger() * (Math.sqrt(40) - 1));
    double mult2 = 1.0 + (m_OI.getDriverRightTrigger() * (Math.sqrt(40) - 1));

    double leftY = m_OI.getDriverLeftY();
    double leftX = m_OI.getDriverLeftX();
    double rightX = m_OI.getDriverRightX();
    //sets deadzones on the controller to extend to .05:
    if(Math.abs(leftY) < .05) {leftY = 0;}
    if (Math.abs(leftX) < .05) {leftX = 0;}
    if (Math.abs(rightX) < .05) {rightX = 0;}

    //sets the velocity to a number from 0 to 1/20th of the max:
    leftY *= maximumLinearVelocity * .025;
    leftX *= maximumLinearVelocity * .025;
    rightX *= maximumRotationVelocity * .025;

    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(leftY * 0.5, leftX * 0.5, rightX); //debug
    if (m_OI.getFieldCentricToggle()){
      fieldCentric = !fieldCentric;
    }
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    if(m_OI.getXButton()){
      parked = !parked;
    }
    if(parked && !m_driveSubsystem.getParkingBrake()){
      m_driveSubsystem.parkingBrake(true);
    }
    if(!parked && m_driveSubsystem.getParkingBrake()){
      m_driveSubsystem.parkingBrake(false);
    }
    else if (fieldCentric){
      //Snap to cardinal directions
      double currentAngle = m_driveSubsystem.getOdometry().getRotation().getDegrees() % (2 * Math.PI);
      SmartDashboard.putNumber("Current Angle within 360 degrees", currentAngle);
      SmartDashboard.putNumber("DPad input", 360 - m_OI.getDPad());
      rightX = snapToHeading(currentAngle, (360 - m_OI.getDPad()));

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        -leftY * mult1 * mult2,
        -leftX * mult1 * mult2,
        -rightX * mult1 * mult2,
        Rotation2d.fromDegrees(m_driveSubsystem.getHeading())); // get fused heading
        m_driveSubsystem.setChassisSpeeds(speeds);
    }
    else{
      // Robot centric driving.
      speeds = new ChassisSpeeds();
      speeds.vxMetersPerSecond = -leftY * mult1 * mult2; 
      speeds.vyMetersPerSecond = -leftX * mult1 * mult2; 
      speeds.omegaRadiansPerSecond = -rightX * mult1 * mult2;
      m_driveSubsystem.setChassisSpeeds(speeds); 
    }
    
    // Allow driver to zero the drive subsystem heading for field-centric control.
    if(m_OI.getMenuButton()){
      m_driveSubsystem.zeroHeading();
    }

    if(m_OI.getAButton()){
      Rotation2d zeroRotate = new Rotation2d();
      Pose2d zero = new Pose2d(0.0, 0.0, zeroRotate);
      m_driveSubsystem.resetOdometry(zero);
    }
  }

  public double snapToHeading(double currentAngle, double targetAngle){
    /* 
    while(angle < 0){
      angle += 2 * Math.PI;
    }
    if(m_OI.getDPad() == 0){ //moves battery toward 0
      error = Math.abs(Math.PI - angle);
      if(error <= .1){
        rotation = 0;
      }
      else if((angle <= 0 && angle >= -Math.PI) || (angle >= 0 && angle <= Math.PI)){
        rotation = -angle;
      }
      else if(angle >= Math.PI){
        rotation = -2 * Math.PI - angle;
      }
      else if(angle <= -Math.PI){
        rotation = 2 * Math.PI + angle;
      }
    }

    if(m_OI.getDPad() == 90){ //moves battery toward pi/2
      error = Math.abs(Math.PI / 2 - angle);
      if(error <= .1){
        rotation = 0;
      }
      else if(angle >= 0 && angle <= Math.PI / 2){
        rotation = Math.PI / 2 - angle;
      }
      else if(angle >= Math.PI / 2){
        rotation = -angle - (Math.PI / 2);
      }
      else if(angle <= 0 && angle >= -3 * Math.PI / 2){
        rotation = 3 * Math.PI / 2 + angle;
      }
      else if(angle <= -3 * Math.PI / 2){
        rotation = -angle - 3 * Math.PI/2;
      }
    }

    if(m_OI.getDPad() == 180){ //moves battery toward pi
      error = Math.abs(0 - angle);
      if(error <= .1){
        rotation = 0;
      }
      else if((angle >= 0 && angle <= Math.PI) || (angle <= 0 && angle >= -Math.PI)){
        rotation = -angle;
      }
      else if(angle <= -Math.PI){
        rotation = -2 * Math.PI + angle;
      }
      else if(angle >= Math.PI){
        rotation = 2 * Math.PI - angle;
      }
    }

    if(m_OI.getDPad() == 270){ //moves battery toward 3pi/2
      error = Math.abs(3/2 * Math.PI - angle);
      if(error <= .1){
        rotation = 0;
      }
      else if(angle >= 0 && angle <= 3 * Math.PI / 2){
        rotation = 3 * Math.PI / 2 - angle;
      }
      else if(angle >= 3 * Math.PI / 2){
        rotation = angle - 3 * Math.PI / 2;
      }
      else if(angle <= 0 && angle >= -Math.PI / 2){
        rotation = -angle;
      }
      else if(angle <= -Math.PI / 2){
        rotation = -angle - Math.PI / 2;
      }
    }
    */
    double error = currentAngle - targetAngle;
    while(error < -180){error += 360;}
    while(error > 180){error -= 360;}
    SmartDashboard.putNumber("Angle Error", error);
    if(Math.abs(error) < STOP_THRESHOLD){
      return 0;
    }
    if(Math.abs(error) < SLOW_THRESHOLD){
      return error/SLOW_THRESHOLD;
    }
    return error;
    
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
