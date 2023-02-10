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
    public double targetAngle = 0.0; // target angle in robot coordinates
  }

  private SerialPort port;
  private double lastUpdateTime = 0;
  private int counter = 0;
  private ArrayList<Target> targets;
  private String leftover;
  private double focalLength = 0.0028; // 2.8 * 10^-3 meters
  private double pixelSizeX = 0.000011; // 11 μm, 1.1 * 10^-5
  private double pixelSizeY = 0.000011; // 11 μm, 1.1 * 10^-5
  private int resolutionX = 320;
  private int resolutionY = 240; // divide by 2 to find bottom half (ground)
  private double yOfPixels;
  private double xOfPixels;
  private double mountHeight;

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
      }
      else {
        // System.out.println("Valid OpenMV Message");
        for (int index = 0; index < fields.length; index += 5) {
          if(Integer.parseInt(fields[index +2]) >= resolutionY/2) //if image is on bottom half of pixels...
          {
            Target t = new Target();
            t.type = fields[index];
            t.imagex = Integer.parseInt(fields[index +1]);
            t.imagey = Integer.parseInt(fields[index +2]);
            t.confidence = Double.parseDouble(fields[index +3]);
            t.area = Double.parseDouble(fields[index +4]);

            // conversion to robot coordinates (x distance)
            yOfPixels = (t.imagey - (resolutionY/2))*pixelSizeY; 
            t.targetx = (mountHeight * focalLength)/yOfPixels; 
            // TODO: account for type of game piece and subtract length from middle to ground point
  
            // conversion to robot coordinate (angle)
            xOfPixels = (t.imagex - (resolutionX/2))*pixelSizeX;
            t.targetAngle = Math.atan(focalLength/xOfPixels);
  
            // conversion to robot coordinates (y distance)
            t.targety = t.targetx * Math.tan(t.targetAngle);
            targets.add(t);
          }
          
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

