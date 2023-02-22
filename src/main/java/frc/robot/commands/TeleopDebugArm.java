// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.OI;

public class TeleopDebugArm extends CommandBase {
  private Arm arm;
  private OI oi;
  //private final double maximumX = 48;
  //private final double maximumZ = 78;
  private final double maxShoulderVel = 3;
  private final double maxElbowVel = 3;

  /** Creates a new DebugArm. */
  public TeleopDebugArm(Arm arm, OI oi) {
    this.arm = arm;
    this.oi = oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(false){
      arm.setTargetAngle(arm.new JointPositions(0,0)); //tuck arm in
    }
    //code change for week zero (increased speed) 
    double leftY = -oi.getOperatorLeftY() * 1.2;
    double rightY = oi.getOperatorRightY() * 1.2;
    /* 
    if(leftX > 0){
      if(arm.getCartesianPosition(arm.getJointAngles()).getCartesianX() >= maximumX ){
        leftX *= -1;
      }

      if(arm.getCartesianPosition(arm.getJointAngles()).getCartesianX() >= maximumX ){
        leftX *= -1;
      }
    }
    else{
      if(arm.getCartesianPosition(arm.getJointAngles()).getCartesianX() >= maximumX ){
        leftX = -rightX;      
      }

      if(arm.getCartesianPosition(arm.getJointAngles()).getCartesianX() >= maximumX ){
        leftX = -rightX;      
      }
    }
*/
    //if(leftX < 0 || rightX < 0){
    //  if(arm.getJointAngles().shoulder <= arm.getMinAngles().shoulder )
    //}
    // if(arm.getAbsoluteAngles().getShoulderAngle() >= arm.getMinAngles().getShoulderAngle()){
    //   leftX = Math.max(0, leftX);
    // }

    // if(arm.getAbsoluteAngles().getElbowAngle() <= arm.getMinAngles().getElbowAngle()){
    //   rightX = Math.max(0, rightX);
    // }
/* 
    if(arm.getAbsoluteAngles().getShoulderAngle() >= arm.getMaxAngles().getShoulderAngle()){
      leftX = Math.max(0, leftX);
    }

    if(arm.getAbsoluteAngles().getElbowAngle() >= arm.getMaxAngles().getElbowAngle()){
      rightX = Math.max(0, rightX);
    }
*/
    arm.setJointVelocities(arm.new JointVelocities(leftY * maxShoulderVel, rightY * maxElbowVel));

    if(oi.getAButton()){
      arm.setTargetAngle(arm.new JointPositions(-2.0, 4.7));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setJointVelocities(arm.new JointVelocities(0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}