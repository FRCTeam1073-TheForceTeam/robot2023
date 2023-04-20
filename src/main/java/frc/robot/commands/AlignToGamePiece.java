// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GamePieceFinder;
import frc.robot.subsystems.OI;

public class AlignToGamePiece extends CommandBase {
  /** Creates a new AlignToGamePiece. */
  private DriveSubsystem drivetrain;
  private Bling bling;
  private GamePieceFinder finder;
  private double maxVelocity;
  private double YTolerance;
  private double XTolerance;
  private double targetConeX;
  private double targetConeY;
  private double targetCubeX;
  private double targetCubeY;
  private double targetX;// current targetX
  private double targetY;//current target Y
  private double gamePieceX;//current game piece position
  private double gamePieceY;// current game piece position
  private boolean cubeMode;
  private OI oi;



  // State variables for execution:
  Boolean targetGamePiece;
  double targetGamePieceDistance;
  private ChassisSpeeds chassisSpeeds;
  int glitchCounter;
  double yOffset;


  public AlignToGamePiece(DriveSubsystem drivetrain, Bling bling, GamePieceFinder finder, double maxVelocity, boolean cubeMode, double XTolerance, double YTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.bling = bling;
    this.finder = finder;
    this.XTolerance = XTolerance;
    this.YTolerance = YTolerance;
    this.cubeMode = cubeMode;
    this.maxVelocity = maxVelocity;
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    targetCubeX = 434;
    targetCubeY = 345;
    targetConeX = 460;
    targetConeY = 324.5;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bling.clearLEDs();
    bling.setColorRGBAll(50, 100, 50);


    if (cubeMode == true) {
      targetX = targetCubeX;
      targetY = targetCubeY;
    }else{
      targetX = targetConeX;
      targetY = targetConeY;
    }
    System.out.println("Align To Game Piece Started");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (cubeMode) {
     if (finder.getClosestCube() == -1) {
       return;
     }
    }else{
      if (finder.getClosestCone() == -1){
        return;
      }
    }

    if (cubeMode == true) {
      gamePieceX = finder.closestCubeX;
      gamePieceY = finder.closestCubeY;
      gamePieceX = MathUtil.clamp(gamePieceX, 0, 10000);
      gamePieceY = MathUtil.clamp(gamePieceY, 0, 10000);
    }else{
      gamePieceX = finder.closestConeX;
      gamePieceY = finder.closestConeY;
      gamePieceX = MathUtil.clamp(gamePieceX, 0, 10000);
      gamePieceY = MathUtil.clamp(gamePieceY, 0, 10000);
    }

    double deltaX = targetX - gamePieceX;
    double deltaY = targetY - gamePieceY;

    //ends command if game piece is close (x,y) to robot

    chassisSpeeds.vyMetersPerSecond = (deltaX) * 0.0025;
    chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -maxVelocity, maxVelocity);

    chassisSpeeds.vxMetersPerSecond = (deltaY) * 0.0025;
    chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, 0, maxVelocity);

    chassisSpeeds.omegaRadiansPerSecond = 0;

    drivetrain.setChassisSpeeds(chassisSpeeds);


    SmartDashboard.putNumber("VY Meters pre second", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("VX Meters pre second", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Game Piece Max Velocity", maxVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds.vyMetersPerSecond = 0;
    chassisSpeeds.vxMetersPerSecond = 0;
    chassisSpeeds.omegaRadiansPerSecond = 0;
    drivetrain.setChassisSpeeds(chassisSpeeds);

    bling.clearLEDs();
    System.out.println("Align To Game Piece Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cubeMode) {
      if (finder.getClosestCube() == -1) {
        System.out.println("No Game Piece To Be Found");
        return true;
      }
     }else{
       if (finder.getClosestCone() == -1){
        System.out.println("No Game Piece To Be Found");
         return true;
       }
     }

    if (Math.abs(targetX - gamePieceX) < XTolerance && Math.abs(targetY - gamePieceY) < YTolerance) {
      System.out.println("Aligned to Game Piece Finished");
      return true;
    }else{
      return false;
    }

  }
}
