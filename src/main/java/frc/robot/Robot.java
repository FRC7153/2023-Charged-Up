// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frc7153.inputs.XboxController;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmPI;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.ShuffleboardManager;
import frc.robot.subsystems.SwerveDriveBase;

public class Robot extends TimedRobot {
  // Joysticks
  public static XboxController controller0 = new XboxController(0);

  // Peripherals
  public static IMU imu = new IMU();
  public static ArmPI armPi = new ArmPI();

  // Actuators
  public static SwerveDriveBase driveBase = new SwerveDriveBase();
  public static Arm arm = new Arm();

  // Shuffleboard
  public static ShuffleboardManager shuffleboard = new ShuffleboardManager();

  // Robot Init
  @Override
  public void robotInit() {}

  // Robot Periodic
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    imu.accumulatePosition();
  }

  // Auto Init
  @Override
  public void autonomousInit() {}

  // Auto Periodic
  @Override
  public void autonomousPeriodic() {}

  // Teleop Init
  @Override
  public void teleopInit() {}

  // Teleop Periodic
  @Override
  public void teleopPeriodic() {
    driveBase.driveTeleop();
  }

  // Disabled Init
  @Override
  public void disabledInit() {}

  // Test Init
  @Override
  public void testInit() {}

  // Test Periodic
  @Override
  public void testPeriodic() {}
}
