// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Bling extends SubsystemBase {
  /** Creates a new Bling. */
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  public int slotLength;

  public boolean cleared;

  public int ledR = 0;
  public int ledG = 0;
  public int ledB = 0;
  private NetworkTable blingNetworkTable;
  private double blingEntry;
  private double blingEntry2;
  Arm arm;

  public Bling() {
    m_led = new AddressableLED(0);
    blingNetworkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    blingEntry = blingNetworkTable.getEntry("Shoulder Angle").getDouble(0.0);
    //blingEntry2 = blingNetworkTable.getEntry("Shoulder Angle on init").getDouble(0.0);
   // blingEntry = arm.getJointAngles().shoulder;
    // TODO: change for the actual length
    m_ledBuffer = new AddressableLEDBuffer(24);//0 as placeholder
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    slotLength = (int) (m_ledBuffer.getLength() / (8));//1 as placeholder

  }

  public void initialize() {
    clearLEDs();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
    batteryBling(0);
    
  }

  public void setRGB(int i, int r, int g, int b)
  {
    m_ledBuffer.setRGB(i, r, g, b);
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
  
  }
  
  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  public AddressableLEDBuffer getM_LEDBuffer() {
    return m_ledBuffer;
  }

  public void cleared() {
    cleared = true;
  }

  public void uncleared() {
    cleared = false;
  }

  public void clearLEDs() {
    setColorRGBAll(0, 0, 0);
  }

  public void setLED(int i, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
    m_led.setData(m_ledBuffer);
  }



  // This sets two leds with the same color
  public void setLEDs2(int i, int i2, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
    m_ledBuffer.setRGB(i2, r, g, b);
    // m_led.setData(m_ledBuffer);
  }



  // setColorRGBAll sets the LEDs all to one color
  public void setColorRGBAll(int r, int g, int b) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

   // rangeRGB() sets a range of LEDs to one color
   public void rangeRGB(int min, int number, int r, int g, int b) {
    if (number != 1) {
      int max = min + number;
      for (int i = min; i < (max); i++) {
        m_ledBuffer.setRGB(i, r, g, b);
      }
    } else {
      m_ledBuffer.setRGB(min, r, g, b);
    }
  }

  public void setSlot(int slotNumber, int r, int g, int b) {
    rangeRGB(slotNumber*slotLength, slotLength, r, g, b);

  }

  public void batteryBling(int slotNumber) {
    
    
    double volts = RobotController.getBatteryVoltage();

    if (volts > 12) {
      setSlot(slotNumber, 0, 255, 0);
    }
    else if (volts > 10){
      setSlot(slotNumber, 0, 0, 255);
    }
    else{
      setSlot(slotNumber, 255, 0, 0);
    }
  }

  public void setRGBAll(int r, int g, int b)
  {
    ledR = r;
    ledG = g;
    ledB = b;

    for(int i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }
}
