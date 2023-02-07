// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OpenMV extends SubsystemBase {

  // The target output from OpenMV
  // TODO: needs to output target positions in robot coordinates
  public class Target{
    public String type;
    public int imagex = 0;
    public int imagey = 0;
    public double confidence = 0.0;
    public double area = 0.0;
    public double targetx = 0.0; // target position in robot coordinates
    public double targety = 0.0; // target position in robot coordinates
  }

  private SerialPort port;
  private double lastUpdateTime = 0;
  private int counter = 0;
  private ArrayList<Target> targets;
  private String leftover;

  /** Creates a new OpenMV interface on given serial port. */
  public OpenMV(SerialPort.Port p) {
    try {
     port = new SerialPort(230400,p,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
     lastUpdateTime = Timer.getFPGATimestamp();
    }
    catch (Exception e) {
      System.out.println("OpenMV Could not open serial port!");
      port = null;
    }
    targets = new ArrayList<Target>();
    leftover = new String();
  }

  @Override
  public void periodic() {
    if (port != null) {
        counter = counter + 1;
        String s = port.readString();
        String input = leftover +s;
        // System.out.println(String.format("OpenMV input: %s", input));
        // Handle message buffering/misalignment.
        int lastMessage = 0;
        for (int i =0; i<input.length();++i){
          if (input.charAt(i)=='\n'){
            parseMessage(input.substring(lastMessage, i));
            lastMessage = i +1;
          }
        }
        if (input.length() != 0){
          leftover = input.substring(lastMessage, input.length());
        }
          // System.out.println(String.format("OpenMV Leftover: %s", leftover));
      }
  }

    public boolean parseMessage(String s){
      // System.out.println(String.format("OpenMV Parse Message %s", s));
      targets.clear();
      String[] fields = s.split("[,]");
      if (fields.length% 5 != 0){
        System.out.println("Invalid OpenMV Message!");
        return false;
      }else {
        // System.out.println("Valid OpenMV Message");
        for (int index = 0; index < fields.length; index += 5) {
          Target t = new Target();
          t.type = fields[index];
          t.imagex = Integer.parseInt(fields[index +1]);
          t.imagey = Integer.parseInt(fields[index +2]);
          t.confidence = Double.parseDouble(fields[index +3]);
          t.area = Double.parseDouble(fields[index +4]);
          // TODO: add conversion to robot coordinates
          t.targetx = (240 - t.imagey)/200;
          t.targety = (320 - t.imagex)/300;
          targets.add(t);
        }
        lastUpdateTime = Timer.getFPGATimestamp(); // Update last valid time since we have a packet.
        return true;
    }
  }

  // Return the current set of active targets
  public ArrayList<Target> getTargets(){
    // If current time is < 2 seconds since last update, then return data, else data is stale/invalid.
    if (Timer.getFPGATimestamp() > (lastUpdateTime + 2.0)) {
      return targets;
    } else {
      return new ArrayList<Target>(); // Return empty list since we don't have new/valid data.
    }
  }

  // Return the last update time in seconds to detect old data.
  public double getLastUpdateTime() {
    return lastUpdateTime;
  }

}

