// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 // Resources:
  // CANifier - https://docs.ctre-phoenix.com/en/stable/ch12_BringUpCANifier.html
  // setLEDOutput() - https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1_c_a_nifier.html


public class Underglow extends SubsystemBase {
  /** Creates a new Underglow. */
  private CANifier canifier;

  /**Underglow constructor. Initializes canfiers and sets them to the default configuration
   */
  public Underglow() {
    canifier = new CANifier(14);
    canifier.configFactoryDefault();
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 1000);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public static void initPreferences() 
  {
    
  }

  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  // This method sets the color
  public void setColor(Color8Bit color){
    //canifier.?
  }

  // returns current color in the type Color8Bit
  public Color8Bit getCurrentColor(){
    return new Color8Bit(0,0,0);
  }

  //Sets the LED intensity
  public void setLEDIntensity(double redPercent, double greenPercent, double bluePercent){
    redPercent = MathUtil.clamp(redPercent, 0, 1);
    bluePercent = MathUtil.clamp(bluePercent, 0, 1);
    canifier.setLEDOutput(redPercent, CANifier.LEDChannel.LEDChannelA);
    canifier.setLEDOutput(greenPercent, CANifier.LEDChannel.LEDChannelB);
    canifier.setLEDOutput(bluePercent, CANifier.LEDChannel.LEDChannelC);
  }
}
