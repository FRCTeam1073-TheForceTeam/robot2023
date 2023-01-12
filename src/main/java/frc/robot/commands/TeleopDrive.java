// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OI;

public class TeleopDrive extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  OI m_OI;
  /** Creates a new Teleop. */
  public TeleopDrive(DriveSubsystem ds, OI oi) {
    addRequirements(ds);
    m_driveSubsystem = ds;
    m_OI = oi;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Vector<N2> vector = new Vector<N2>(Nat.N2()); //(m_OI.getDriverLeftX() * 0.5, m_OI.getDriverLeftY() * 0.5);
    //vector.rotate(m_driveSubsystem.getHeading()); //rotates by degrees
    //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vector.x, vector.y, m_OI.getDriverRightX()); //rotation-oriented
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(m_OI.getDriverLeftX() * 0.5, m_OI.getDriverLeftY() * 0.5, m_OI.getDriverRightX()); //debug
    //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    m_driveSubsystem.setChassisSpeeds(chassisSpeeds);
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
