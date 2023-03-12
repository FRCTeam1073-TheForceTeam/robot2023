// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

public class GamePieceFinder extends SubsystemBase {
  /** Creates a new GamePieceFinder. */
  public class GamePiece {
    public double x = 0.0;
    public double y = 0.0;
    public double w = 0.0;
    public double h = 0.0;
  }

  private NetworkTable gamePieceNetwork;
  private NetworkTableEntry gamePieceEntry;
  private String tableName;
  private String closestGamePiece;
  private double closestDistance;
  private DriveSubsystem driveSubsystem;
  private ArrayList<GamePiece> gamePieceArray;
  private Transform3d cameraTransform;
  private GamePiece gamePiece;
  private GamePiecePoseEstimator poseEstimator;

  public class GamePiecePoseEstimator {
    //public Transform3d estimate(){
    //}
  }

  public GamePieceFinder(DriveSubsystem ds, String tableName, Transform3d cameraTransform3d) {
    driveSubsystem = ds;
    this.tableName = tableName;
    this.cameraTransform = cameraTransform;
    gamePieceNetwork = NetworkTableInstance.getDefault().getTable(tableName);
    gamePieceEntry = gamePieceNetwork.getEntry("Cubes");
    gamePieceArray = new ArrayList<GamePiece>();
    
    closestGamePiece = "None";
    closestDistance = 99999.0;
  }

  @Override
  public void periodic() {
    Number[] gamePieceData = gamePieceEntry.getNumberArray(new Number[0]);
    gamePieceArray.clear();
    int numGamePiece = gamePieceData.length/4;
    // Reset search variables for clostest to empty:
    closestGamePiece = "None";
    closestDistance = 9999.0;


    for (int i = 0; i < numGamePiece; i = i +1){
      
      double[] cubes = new double[3];

      cubes[0] = gamePieceData[i*4 + 1].doubleValue();
      cubes[1] = gamePieceData[i*4 + 2].doubleValue();
      cubes[2] = gamePieceData[i*4 + 3].doubleValue();
      cubes[3] = gamePieceData[i*4 + 4].doubleValue();

      gamePiece.x = cubes[0];
      gamePiece.y = cubes[1];
      gamePiece.w = cubes[2];
      gamePiece.h = cubes[3];


    }


    SmartDashboard.putNumber(String.format("%s/NumGamePieces", tableName), numGamePiece);
    if (closestGamePiece != "None") {
      SmartDashboard.putString(String.format("%s/ClosestGamePiece", tableName), closestGamePiece);
      SmartDashboard.putNumber(String.format("%s/ClosestDistance", tableName), closestDistance);
      SmartDashboard.putNumber(String.format("%s/GamePieceX", tableName), gamePiece.x);
      SmartDashboard.putNumber(String.format("%s/GamePieceY", tableName), gamePiece.y);
      SmartDashboard.putNumber(String.format("%s/GamePieceWidth", tableName), gamePiece.w);
      SmartDashboard.putNumber(String.format("%s/GamePieceHeight", tableName), gamePiece.h);
    }else{
      SmartDashboard.putString(String.format("%s/ClosestGamePiece", tableName), "None");
      SmartDashboard.putNumber(String.format("%s/ClostestDistance", tableName), 99999.0);
      SmartDashboard.putNumber(String.format("%s/GamePieceX", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/GamePieceY", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/GamePieceWidth", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/GamePieceHeight", tableName), 0.0);
    }
  }


  public Pose2d getFieldPose() {
    if (gamePieceArray.size() > 0) {
      //TODO: Implement this method when needed
      return null; // Until we implement it.
    }else{
      return null;
    }
  }


  public String getClosestGamePiece() {
    return closestGamePiece;
  }
}
