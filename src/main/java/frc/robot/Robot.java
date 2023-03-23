// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private int diagnosticCounter = 0;

  @Override
  public void robotInit() {
    // We must call this before creating the robot container: Sets up preferences *once* at startup.
    RobotContainer.initPreferences();

    // Create our robot container
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setStartupLighting();
    m_robotContainer.disableInit();
  }

  @Override
  public void disabledPeriodic() {
    if(diagnosticCounter > 100){
      m_robotContainer.diagnostics();
      diagnosticCounter = 0;
    }
    else{
      diagnosticCounter++;
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    System.out.println("Robot: Autonomous Init");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } else {
      System.out.println("NO Autonomous Command!");
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    System.out.println("Robot: Teleop Init");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    System.out.println("Robot: testInit");
    //m_robotContainer.setTestMode();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
