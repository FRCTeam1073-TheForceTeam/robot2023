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
  /** Creates a new Bling. 
   * Creates variables
  */
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
  /**
   * @param key - the key name
   * @param defaultValue - the value to be returned if no value is found
   * @param length - the strip length
   * @param buffer - the buffer to write
   * @return Global default instance
   * @return The network table
   * @return Nework table entry
   * @return the entry's value or the given default value
   * @return the buffer length
   */
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

  /**
   * Clears all of the LEDs on the robot
   */
  public void initialize() {
    clearLEDs();
  }

  /**
   * Sets up battary bling
   * and adds a buffer
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
    batteryBling(0);
    
  }

  /**
   * Is capable of setting all of the the RGBs to one color (including battery bling)
   * @param i - the index to write
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  public void setRGB(int i, int r, int g, int b)
  {
    m_ledBuffer.setRGB(i, r, g, b);
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
  
  }
  
  /**
   * 
   * @return the results
   */
  public String getDiagnostics() {
    ErrorCode error;
    String result = new String();
    //Check errors for all hardware
    return result;
  }

  /**
   * 
   * @return LED buffer
   */
  public AddressableLEDBuffer getM_LEDBuffer() {
    return m_ledBuffer;
  }

  /**
   * Boolean to see if the LED's have been cleared
   */
  public void cleared() {
    cleared = true;
  }

  /**
   * Boolean to see if the LED's have been cleared
   */
  public void uncleared() {
    cleared = false;
  }

  /**
   * Clears all of the LEDs to be colorless
   */
  public void clearLEDs() {
    setColorRGBAll(0, 0, 0);
  }

  /**
   * Can set all of the LEDs to one color
   * @param i - the index to write
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  public void setLED(int i, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
    m_led.setData(m_ledBuffer);
  }



  // This sets two leds with the same color
  /**
   * 
   * @param i - the index to write
   * @param i2 - the index to write
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  public void setLEDs2(int i, int i2, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
    m_ledBuffer.setRGB(i2, r, g, b);
    // m_led.setData(m_ledBuffer);
  }



  // setColorRGBAll sets the LEDs all to one color
  /**
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   * @return buffer length
   */
  public void setColorRGBAll(int r, int g, int b) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

   // rangeRGB() sets a range of LEDs to one color
   /**
    * 
    * @param min
    * @param number
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
    */
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

  /**
   * 
   * @param slotNumber
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  public void setSlot(int slotNumber, int r, int g, int b) {
    rangeRGB(slotNumber*slotLength, slotLength, r, g, b);

  }

  /**
   * Sets the battery bling to 
   * green when battery is grater than 12
   * Blue when battery is grater than 10
   * Red is set to anything lower than 10
   * @param slotNumber
   */
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


  /**
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
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
