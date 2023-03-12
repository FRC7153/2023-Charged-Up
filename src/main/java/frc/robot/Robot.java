// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.OI.Controller1;

public class Robot extends TimedRobot {
  private RobotContainer container;
  private Command autoCommand;

  // Robot Init
  @Override
  public void robotInit() {
    container = new RobotContainer();
  }

  // Robot Periodic
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //System.out.println(container.driveBase.getPose().toString());
  }

  // Auto Init
  @Override
  public void autonomousInit() {}

  // Auto Periodic
  @Override
  public void autonomousPeriodic() {
    autoCommand = container.getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  // Teleop Init
  @Override
  public void teleopInit() {}

  // Teleop Periodic
  @Override
  public void teleopPeriodic() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  // Disabled Init
  @Override
  public void disabledInit() {
    // Set arms to coast to be reset
    container.claw.setCoastMode(true);
  }

  // Test Init
  @Override
  public void testInit() {
    container.arm.setAngle(0.0);
    ext = 0.0;
  }

  // Test Periodic
  private double ext = 0.0;
  private double debounce = 0.0;
  @Override
  public void testPeriodic() {
    
    container.arm.winchMotor.set(Controller1.getThrottle());

    container.arm.winchEnc.setPosition(0.0);
    

    /*if (Controller1.controller.getRawButton(4) && Timer.getFPGATimestamp() - debounce >= 1.0) {
      ext += (20.0 * 2.0);
      debounce = Timer.getFPGATimestamp();
    }

    container.arm.setExtension(ext);*/

    container.arm.periodic(true);
    container.shuffleboard.periodic();
  }
}
