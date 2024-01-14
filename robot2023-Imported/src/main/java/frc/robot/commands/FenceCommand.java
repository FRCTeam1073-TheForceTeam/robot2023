// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class FenceCommand extends Command {
  /** Creates a new FenceCommand. */
  private DriveSubsystem ds;
  private double minX;
  private double minY;
  private double maxX;
  private double maxY;
  private Command cmd;
  private boolean finish = false;

  public FenceCommand(DriveSubsystem ds, double minX, double minY, double maxX, double maxY, Command cmd) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ds = ds;
    this.minX = minX;
    this.minY = minY;
    this.maxX = maxX;
    this.maxY = maxY;
    this.cmd = cmd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd.initialize();
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cmd.execute();
    Pose2d odometry = ds.getOdometry();
    if(odometry.getX() < minX || odometry.getY() < minY || odometry.getX() > maxX || odometry.getY() > maxY){
      finish = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmd.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish || cmd.isFinished();
  }
}
