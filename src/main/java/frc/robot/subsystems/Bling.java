// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Bling extends SubsystemBase {
  /** Creates a new Bling. */
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  public int slotLength;

  public boolean cleared;

  public Bling() {
    m_led = new AddressableLED(9);
    // TODO: change for the actual length
    m_ledBuffer = new AddressableLEDBuffer(0);//0 as placeholder
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    slotLength = (int) (m_ledBuffer.getLength() / (1));//1 as placeholder
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
  
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
    // m_led.setData(m_ledBuffer);
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
    // m_led.setData(m_ledBuffer);
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

  public void batteryBling(int minLEDsVolts, int numberLEDsVolts, double min_volts, double max_volts) {
    for (int i = minLEDsVolts; i < (minLEDsVolts + numberLEDsVolts); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    
    double volts = RobotController.getBatteryVoltage();

    if (volts > max_volts) {
      volts = max_volts;
    }
  }
}
