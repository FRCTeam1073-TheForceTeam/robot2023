// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GamePieceFinder;

public class AlignToGamePiece extends CommandBase {
  /** Creates a new AlignToGamePiece. */
  private DriveSubsystem drivetrain;
  private Bling bling;
  private GamePieceFinder finder;
  private double maxVelocity;
  private double maxAnguarVelocity;
  private double tolerance;


  // State variables for execution:
  int tagetGamePiece;
  double targetGamePieceDistance;
  private ChassisSpeeds chassisSpeeds;
  int glitchCounter;


  public AlignToGamePiece(DriveSubsystem drivetrain, Bling bling, GamePieceFinder finder, double maxVelosity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.bling = bling;
    this.finder = finder;
    this.maxAnguarVelocity = 0.2;
    this.tolerance = 0.05;
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String closestGamePiece = finder.getClosestGamePiece();
    bling.clearLEDs();
    if(closestGamePiece != "None"){
      System.out.println(String.format("AlignToGamePiece Initialized to %s", closestGamePiece));
    }  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
