// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frc7153.Controllers.Joystick;
import com.frc7153.Controllers.OffsetMode;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  // Joysticks
  public static Joystick joy1 = new Joystick(0);

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    System.out.println(joy1.getX());
  }

  @Override
  public void autonomousInit() {
    joy1.calibrateOffset();
    joy1.setOffsetMode(OffsetMode.TRANSlATE);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
