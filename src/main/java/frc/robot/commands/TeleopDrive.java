// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
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
    if (m_OI.getLeftBumper())
    {
      velocityMult = 0.15;
      rotateMult = 0.25;
    }
    else if (m_OI.getRightBumper())
    {
      velocityMult = 0.8;
      rotateMult = 1;
    }
    else
    {
      velocityMult = 0.35;
      rotateMult = 0.5;
    }

    if (m_OI.getFieldCentricToggle())
    {
      fieldCentric = !fieldCentric;
    }

    SmartDashboard.putBoolean("Field Centric", fieldCentric);

    if (fieldCentric)
    {
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        m_OI.getDriverLeftY() * velocityMult,
        m_OI.getDriverLeftX() * velocityMult, 
        m_OI.getDriverRightX() * rotateMult,
        Rotation2d.fromDegrees(0)); // get fused heading
      m_driveSubsystem.setChassisSpeeds(speeds);
    }
    else
    {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
      chassisSpeeds.vxMetersPerSecond = m_OI.getDriverLeftY() * velocityMult; 
      chassisSpeeds.vyMetersPerSecond = m_OI.getDriverLeftX() * velocityMult; 
      chassisSpeeds.omegaRadiansPerSecond = m_OI.getDriverRightX() * rotateMult;
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
