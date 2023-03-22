// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.OI;

public class TeleopClaw extends CommandBase {
  private Claw claw;
  private OI oi;
  private double clawPower;

  /** Creates a new TeleopClaw. */
  public TeleopClaw(Claw claw, OI oi) {
    this.claw = claw;
    this.oi = oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawPower = 0;
    claw.setCollectorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //right bumper is in for cube, left is out
    //reversed for cube
    if(oi.getOperatorLeftBumper()){
      if(oi.isCubeMode() == true) {
        claw.setCollectorSpeed(5);
      }
      if(oi.isCubeMode() == false){
        claw.setCollectorSpeed(-5);
      }
      System.out.println("Operator Left Bumper");
    }
    else if(oi.getOperatorRightBumper()){
      if(oi.isCubeMode() == true) {
        claw.setCollectorSpeed(-5);
      }
      if(oi.isCubeMode() == false){
        claw.setCollectorSpeed(5);
      }
      System.out.println("Operator Right Bumper");
    }
    else {
      claw.setCollectorSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setCollectorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
