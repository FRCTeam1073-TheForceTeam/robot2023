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

  private NetworkTable cubeNetwork;
  private NetworkTable coneNetwork;
  private NetworkTableEntry cubePieceEntry;
  private NetworkTableEntry conePieceEntry;
  private String tableName;
  private String closestCube;
  private double closestCubeDistance;
  private String closestCone;
  private double closestConeDistance;
  private DriveSubsystem driveSubsystem;
  private ArrayList<GamePiece> cubeArray;
  private ArrayList<GamePiece> coneArray;
  private Transform3d cameraTransform;
  private GamePiece cube;
  private GamePiece cone;
  private GamePiecePoseEstimator poseEstimator;

  public class GamePiecePoseEstimator {
    //public Transform3d estimate(){
    //}
  }

  public GamePieceFinder(DriveSubsystem ds, String tableName, Transform3d cameraTransform3d) {
    driveSubsystem = ds;
    this.tableName = tableName;
    cubeNetwork = NetworkTableInstance.getDefault().getTable(tableName);
    cubePieceEntry = cubeNetwork.getEntry("Cubes");
    cubeArray = new ArrayList<GamePiece>();

    closestCube = "None";
    closestCubeDistance = 99999.0;

    coneNetwork = NetworkTableInstance.getDefault().getTable(tableName);
    conePieceEntry = cubeNetwork.getEntry("Cones");
    coneArray = new ArrayList<GamePiece>();

    closestCone = "None";
    closestConeDistance = 99999.0;
  }

  @Override
  public void periodic() {
    Number[] cubeData = cubePieceEntry.getNumberArray(new Number[0]);
    cubeArray.clear();
    int numCube = cubeData.length/4;
    // Reset search variables for clostest to empty:
    closestCube = "None";
    closestCubeDistance = 9999.0;


    for (int i = 0; i < numCube; i = i +1){
      
      double[] cubes = new double[3];

      cubes[0] = cubeData[i*4 + 1].doubleValue();
      cubes[1] = cubeData[i*4 + 2].doubleValue();
      cubes[2] = cubeData[i*4 + 3].doubleValue();
      cubes[3] = cubeData[i*4 + 4].doubleValue();

      cube.x = cubes[0];
      cube.y = cubes[1];
      cube.w = cubes[2];
      cube.h = cubes[3];
    }


    Number[] coneData = conePieceEntry.getNumberArray(new Number[0]);
    coneArray.clear();
    int numCone = coneData.length/4;
    // Reset search variables for clostest to empty:
    closestCone = "None";
    closestConeDistance = 9999.0;

    for (int i = 0; i < numCone; i = i +1){
      
      double[] cones = new double[3];

      cones[0] = coneData[i*4 + 1].doubleValue();
      cones[1] = coneData[i*4 + 2].doubleValue();
      cones[2] = coneData[i*4 + 3].doubleValue();
      cones[3] = coneData[i*4 + 4].doubleValue();

      cone.x = cones[0];
      cone.y = cones[1];
      cone.w = cones[2];
      cone.h = cones[3];
    }
    //~~~~~~~~~~~~~~~~~~~~ ENTERING CUBE AREA ~~~~~~~~~~~~~~~~~~~~
    SmartDashboard.putNumber(String.format("%s/NumCube", tableName), numCube);
    if (closestCube != "None") {
      SmartDashboard.putString(String.format("%s/ClosestCube", tableName), closestCube);
      SmartDashboard.putNumber(String.format("%s/ClosestCubeDistance", tableName), closestCubeDistance);
      SmartDashboard.putNumber(String.format("%s/CubeX", tableName), cube.x);
      SmartDashboard.putNumber(String.format("%s/CubeY", tableName), cube.y);
      SmartDashboard.putNumber(String.format("%s/CubeWidth", tableName), cube.w);
      SmartDashboard.putNumber(String.format("%s/CubeHeight", tableName), cube.h);
    }
    else{
      SmartDashboard.putString(String.format("%s/ClosestCube", tableName), "None");
      SmartDashboard.putNumber(String.format("%s/ClosestCubeDistance", tableName), 99999.0);
      SmartDashboard.putNumber(String.format("%s/CubeX", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/CubeY", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/CubeWidth", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/CubeHeight", tableName), 0.0);
    }
    //~~~~~~~~~~~~~~~~~~~~ ENTERING CONE AREA ~~~~~~~~~~~~~~~~~~~~
    SmartDashboard.putNumber(String.format("%s/NumCone", tableName), numCone);
    if (closestCube != "None") {
      SmartDashboard.putString(String.format("%s/ClosestCone", tableName), closestCone);
      SmartDashboard.putNumber(String.format("%s/ClosestConeDistance", tableName), closestConeDistance);
      SmartDashboard.putNumber(String.format("%s/ConeX", tableName), cone.x);
      SmartDashboard.putNumber(String.format("%s/ConeY", tableName), cone.y);
      SmartDashboard.putNumber(String.format("%s/ConeWidth", tableName), cone.w);
      SmartDashboard.putNumber(String.format("%s/ConeHeight", tableName), cone.h);
    }
    else{
      SmartDashboard.putString(String.format("%s/ClosestCone", tableName), "None");
      SmartDashboard.putNumber(String.format("%s/ClosestConeDistance", tableName), 99999.0);
      SmartDashboard.putNumber(String.format("%s/ConeX", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/ConeY", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/ConeWidth", tableName), 0.0);
      SmartDashboard.putNumber(String.format("%s/ConeHeight", tableName), 0.0);
    }
  }


  public Pose2d getFieldPoseCube() {
    if (cubeArray.size() > 0) {
      //TODO: Implement this method when needed
      return null; // Until we implement it.
    }else{
      return null;
    }
  }

  public Pose2d getFieldPoseCone() {
    if (coneArray.size() > 0) {
      //TODO: Implement this method when needed
      return null; // Until we implement it.
    }else{
      return null;
    }
  }


  public String getClosestCube() {
    return closestCube;
  }

  public String getClosestCone() {
    return closestCone;
  }
}
