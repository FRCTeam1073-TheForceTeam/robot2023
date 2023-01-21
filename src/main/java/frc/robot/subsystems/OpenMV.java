// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OpenMV extends SubsystemBase {
  public class Target{
    public String type;
    public int imagex =0;
    public int imagey =0;
    public double confidence =0.0;
    public double area =0.0;
  }
  private SerialPort port;
  private int counter = 0;
  private ArrayList<Target> targets;
  private String leftover;
  /** Creates a new OpenMV. */
  public OpenMV(SerialPort.Port p) {
    try {
     port = new SerialPort(230400,p,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);

    }
    catch (Exception e) {
      System.out.println("Could not open serial port");
    }
    targets = new ArrayList<Target>();
    leftover = new String();
  }
  @Override
  public void periodic() {
    counter = counter +1;
    String s = port.readString();
    String input = leftover +s;
    System.out.println(String.format("OpenMV input: %s", input));
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
    System.out.println(String.format("OpenMV Leftover: %s", leftover));
    }

    public void parseMessage(String s){
      System.out.println(String.format("OpenMV Parse Message %s", s));
      targets.clear();
      String[] fields = s.split("[,]");
      if (fields.length% 5 != 0){
        System.out.println("Invaled OpenMV Message");
      }else {
        System.out.println("Valed OpenMV Message");
        for (int index = 0; index < fields.length; index += 5) {
          Target t = new Target();
          t.type = fields[index];
          t.imagex = Integer.parseInt(fields[index +1]);
          t.imagey = Integer.parseInt(fields[index +2]);
          t.confidence = Double.parseDouble(fields[index +3]);
          t.area = Double.parseDouble(fields[index +4]);
          targets.add(t);
        }
    }
  }
  //return the current set of active targets
  public ArrayList<Target> getTargets(){
    return targets;
  }
}

