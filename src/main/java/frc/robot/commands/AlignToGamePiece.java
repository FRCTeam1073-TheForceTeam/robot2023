// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private double YTolerance;
  private double XTolerance;
  private double rotationalTolerance;
  private double coneX;
  private double coneY;
  private double cubeX;
  private double cubeY;
  private double rotationSpeed;
  private double targetX;
  private double targetY;
  private boolean coneMode;



  // State variables for execution:
  Boolean targetGamePiece;
  double targetGamePieceDistance;
  private ChassisSpeeds chassisSpeeds;
  int glitchCounter;
  double yOffset;


  public AlignToGamePiece(DriveSubsystem drivetrain, Bling bling, GamePieceFinder finder, double maxVelosity, boolean coneMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.bling = bling;
    this.finder = finder;
    this.maxAnguarVelocity = 0.2;
    this.YTolerance = 0.05;
    this.XTolerance = 0.05;
    this.rotationalTolerance = 0.05;
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    this.coneMode = coneMode;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    bling.clearLEDs();
    
    
    
    //outdated
    // if(finder.closestGamePiece != false){
    //   System.out.println(String.format("AlignToGamePiece Initialized to %s", finder.closestGamePiece));
    //   targetGamePiece = finder.closestGamePiece;
    // } 
    // else {
    //   targetGamePiece = false;
    //   System.out.println("AlignToGamePiece Initialize Failed: No Game Piece in Signt!");
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bling.setColorRGBAll(255, 255, 255);
    double gamePieceX;
    double gamePieceY;


    double currentHeading = drivetrain.getWrappedHeading();

    if (coneMode == true) {
      gamePieceX = finder.closestConeX;
      gamePieceY = finder.closestConeY;
      gamePieceX = MathUtil.clamp(gamePieceX, 0, 10000);
      gamePieceY = MathUtil.clamp(gamePieceY, 0, 10000);
    }else{
      gamePieceX = finder.closestCubeX;
      gamePieceY = finder.closestCubeY;
      gamePieceX = MathUtil.clamp(gamePieceX, 0, 10000);
      gamePieceY = MathUtil.clamp(gamePieceY, 0, 10000);
    }

    //ends command if game piece is close (x,y) to robot
    if (gamePieceY > YTolerance && gamePieceX > XTolerance) {
      chassisSpeeds.vxMetersPerSecond = (targetY - gamePieceY) * 0.5;
      chassisSpeeds.vyMetersPerSecond = (targetX - gamePieceX) * 0.5;
      chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -1, 1);
      chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, 0, 1);
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0, 
        Rotation2d.fromDegrees(drivetrain.getHeading()));
    }else if (gamePieceY < YTolerance && gamePieceX < XTolerance) {
      chassisSpeeds.vxMetersPerSecond = (targetY - -gamePieceY) * 0.5;
      chassisSpeeds.vyMetersPerSecond = (targetX - -gamePieceX) * 0.5;
      chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -1, 1);
      chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, 0, 1);
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0, 
        Rotation2d.fromDegrees(drivetrain.getHeading()));
    }

   // outdates
   // int closestCube = finder.getClosestCube();
   // int closestCone = finder.getClosestCone();
   // TODO:Pose3d targetPose = finder.getClosestPose();
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
