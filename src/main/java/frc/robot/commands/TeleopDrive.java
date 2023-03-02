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
import frc.robot.subsystems.Bling;
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
  Bling m_bling;
  private boolean fieldCentric;
  private boolean parked = false;
  ChassisSpeeds speeds;

  // Teleop drive velocity scaling:
  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second

  //Snap to position thresholds
  private final static double STOP_THRESHOLD = 5;
  private final static double SLOW_THRESHOLD = 30;

  /** Creates a new Teleop. */
  public TeleopDrive(DriveSubsystem ds, OI oi, Bling bling){
    super.setName("Teleop Drive");
    m_driveSubsystem = ds;
    m_OI = oi;
    m_bling = bling;
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
    if(m_OI.getLeftBumper()){
      parked = !parked;
    }
    if(parked && !m_driveSubsystem.getParkingBrake()){
      m_driveSubsystem.parkingBrake(true);
      m_bling.clearLEDs();
      m_bling.setSlot(1, 255, 0, 0);
      m_bling.setSlot(2, 255, 160, 0);
      m_bling.setSlot(3, 255, 255, 0);
      m_bling.setSlot(4, 0, 255, 0);
      m_bling.setSlot(5, 0, 0, 255);
      m_bling.setSlot(6, 255, 0, 255);
      m_bling.setSlot(7, 255, 255, 255);
    }
    if(!parked && m_driveSubsystem.getParkingBrake()){
      m_driveSubsystem.parkingBrake(false);
      m_bling.clearLEDs();
    }
    else if (fieldCentric){
      //Snap to cardinal directions
      double currentAngle = m_driveSubsystem.getOdometry().getRotation().getDegrees();
      rightX = snapToHeading(currentAngle, 360 - m_OI.getDPad(), rightX);

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

    //PAST snap-to vars
    // double leftY = m_OI.getDriverLeftY();
    // double leftX = m_OI.getDriverLeftX();
    // double rightX = m_OI.getDriverRightX();
    // if (Math.abs(leftY) < .1) {leftY = 0;}
    // if (Math.abs(leftX) < .1) {leftX = 0;}
    // if (Math.abs(rightX) < .1) {rightX = 0;}

    // if(m_OI.getXButton()){
    //   parked = !parked;

  public double snapToHeading(double currentAngle, double targetAngle, double defaultVelocity){
    if(targetAngle == 361){
      return defaultVelocity;
    }
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
    return error * .7;
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