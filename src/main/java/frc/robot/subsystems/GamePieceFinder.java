// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

public class GamePieceFinder extends SubsystemBase {
  /** Creates a new GamePieceFinder. */
  private NetworkTable gamePieceNetwork;
  private NetworkTableEntry gamePieceEntry;
  private String tableName;
  private String closestGamePiece;
  private double closestDistance;
  private DriveSubsystem driveSubsystem;
  public GamePieceFinder(DriveSubsystem ds, String tableName) {
    driveSubsystem = ds;
    this.tableName = tableName;
    gamePieceNetwork = NetworkTableInstance.getDefault().getTable(tableName);
    gamePieceEntry = gamePieceNetwork.getEntry("Cubes");
    closestGamePiece = "None";
    closestDistance = 99999.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
