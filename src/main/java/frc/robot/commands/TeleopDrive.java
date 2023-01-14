// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OI;

public class TeleopDrive extends CommandBase 
{
  DriveSubsystem m_driveSubsystem;
  OI m_OI;
  private double velocityMult;
  private double rotateMult;
  private boolean fieldCentric;

  /** Creates a new Teleop. */
  public TeleopDrive(DriveSubsystem ds, OI oi) {
    addRequirements(ds);
    m_driveSubsystem = ds;
    m_OI = oi;
    fieldCentric = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (m_OI.getLeftBumper()){
      velocityMult = 0.15;
      rotateMult = 0.25;
    }
    else if (m_OI.getRightBumper()){
      velocityMult = 0.8;
      rotateMult = 1;
    }
    else{
      velocityMult = 0.35;
      rotateMult = 0.5;
    }

    if(m_OI.getMenuButton()){
      m_driveSubsystem.zeroHeading();
    }

    //Vector<N2> vector = new Vector<N2>(Nat.N2()); //(m_OI.getDriverLeftX() * 0.5, m_OI.getDriverLeftY() * 0.5);
    //vector.rotate(m_driveSubsystem.getHeading()); //rotates by degrees
    //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vector.x, vector.y, m_OI.getDriverRightX()); //rotation-oriented
    double leftY = m_OI.getDriverLeftY();
    double leftX = m_OI.getDriverLeftX();
    double rightX = m_OI.getDriverRightX();
    if(Math.abs(leftY) < .5){leftY = 0;}
    if(Math.abs(leftX) < .5){leftX = 0;}
    if(Math.abs(rightX) < .5){rightX = 0;}
    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(leftY * 0.5, leftX * 0.5, rightX); //debug
    if (m_OI.getFieldCentricToggle())
    {
      fieldCentric = !fieldCentric;
    }

    SmartDashboard.putBoolean("Field Centric", fieldCentric);

    if (fieldCentric)
    {
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        leftY * velocityMult,
        leftX * velocityMult, 
        rightX * rotateMult,
        Rotation2d.fromDegrees(m_driveSubsystem.getHeading())); // get fused heading
      m_driveSubsystem.setChassisSpeeds(speeds);
    }
    else
    {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
      chassisSpeeds.vxMetersPerSecond = leftY * velocityMult; 
      chassisSpeeds.vyMetersPerSecond = leftX * velocityMult; 
      chassisSpeeds.omegaRadiansPerSecond = rightX * rotateMult;
      m_driveSubsystem.setChassisSpeeds(chassisSpeeds); 
    }
    
    //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    
    //m_driveSubsystem.setDebugSpeed(m_OI.getDriverLeftX());
    //m_driveSubsystem.setDebugAngle(m_OI.getDriverRightY() * 3); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
