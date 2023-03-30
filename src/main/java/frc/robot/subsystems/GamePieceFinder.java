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

  private NetworkTable networkTable;
  private NetworkTableEntry cubePieceEntry;
  private NetworkTableEntry conePieceEntry;
  private String tableName;
  private int closestCube;
  public double closestCubeArea;
  private int closestCone;
  public double closestConeArea;
  private DriveSubsystem driveSubsystem;
  private ArrayList<GamePiece> cubeArray;
  private ArrayList<GamePiece> coneArray;
  private Transform3d cameraTransform;
  private GamePiecePoseEstimator poseEstimator;
  public double closestCubeX;
  public double closestCubeY;
  public double closestConeX;
  public double closestConeY;

  public class GamePiecePoseEstimator {
    //public Transform3d estimate(){
    //}
  }

  public GamePieceFinder(DriveSubsystem ds, String tableName) {
    driveSubsystem = ds;
    this.tableName = tableName;
    networkTable = NetworkTableInstance.getDefault().getTable(tableName);
    cubePieceEntry = networkTable.getEntry("Cubes");
    cubeArray = new ArrayList<GamePiece>();

    closestCube = -1;
    closestCubeArea = 0;

    networkTable = NetworkTableInstance.getDefault().getTable(tableName);
    conePieceEntry = networkTable.getEntry("Cones");
    coneArray = new ArrayList<GamePiece>();

    closestCone = -1;
    closestConeArea = 0;
  }

  @Override
  public void periodic() {
    Number[] cubeData = cubePieceEntry.getNumberArray(new Number[0]);
    cubeArray.clear();
    int numCube = cubeData.length/4;
    // Reset search variables for clostest to empty:
    closestCube = -1;
    closestCubeArea = 0;
    closestCubeX = 0.0;
    closestCubeY = 0.0;

    for (int i = 0; i < numCube; i = i +1){
      
      GamePiece cube = new GamePiece();

      cube.x = cubeData[i*4 + 0].doubleValue();
      cube.y = cubeData[i*4 + 1].doubleValue();
      cube.w = cubeData[i*4 + 2].doubleValue();
      cube.h = cubeData[i*4 + 3].doubleValue();

      cubeArray.add(cube);

      double area = cube.w * cube.h;
      
      if (area > closestCubeArea) {
        closestCubeArea = area;
        closestCube = i;
      }

    }

    if (closestCube > -1) {
      closestCubeX = (cubeArray.get(closestCube).x) + (cubeArray.get(closestCube).w/2);
      closestCubeY = (cubeArray.get(closestCube).y) + (cubeArray.get(closestCube).h/2);
    }


    Number[] coneData = conePieceEntry.getNumberArray(new Number[0]);
    coneArray.clear();
    int numCone = coneData.length/4;
    // Reset search variables for clostest to empty:
    closestCone = -1;
    closestConeArea = 0;

    for (int i = 0; i < numCone; i = i +1){
      
      GamePiece cone = new GamePiece();
      cone.x = coneData[i*4 + 0].doubleValue();
      cone.y = coneData[i*4 + 1].doubleValue();
      cone.w = coneData[i*4 + 2].doubleValue();
      cone.h = coneData[i*4 + 3].doubleValue();

      coneArray.add(cone);

      double area = cone.w * cone.h;
      
      if (area > closestConeArea) {
        closestConeArea = area;
        closestCone = i;
      }
    }

    if (closestCone > -1) {
      closestConeX = (coneArray.get(closestCone).x) + (coneArray.get(closestCone).w/2);
      closestConeY = (coneArray.get(closestCone).y) + (coneArray.get(closestCone).h/2);
    }


    // if (!closestGamePiece.equals("None")){
    //   SmartDashboard.putString(String.format("%s/ClosestGamePiece", tableName), closestGamePiece);
    // }else{
    //   SmartDashboard.putString(String.format("%s/ClosestGamePiece", tableName), closestGamePiece);
    // }
      SmartDashboard.putNumber("closestCube", closestCube);
      SmartDashboard.putNumber("closestCone", closestCone);
      SmartDashboard.putNumber("closestCubeArea", closestCubeArea);
      SmartDashboard.putNumber("closestConeArea", closestConeArea);
      SmartDashboard.putNumber("cloestCubeX", closestCubeX);
      SmartDashboard.putNumber("cloestCubeY", closestCubeY);
      SmartDashboard.putNumber("cloestConeX", closestConeX);
      SmartDashboard.putNumber("cloestConeY", closestConeY);
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


  public int getClosestCube() {
    return closestCube;
  }

  public int getClosestCone() {
    return closestCone;
  }
}
