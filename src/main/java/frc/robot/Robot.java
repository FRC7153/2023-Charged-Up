// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    // Robot Container
    private RobotContainer container = new RobotContainer(); // TODO verify this

    // Auto Commands
    private Command autoCommand;

    // Stop auto command and toggle brakes
    private void switchMode(boolean brakes) {
        if (autoCommand != null) { autoCommand.cancel(); }

        container.toggleBrakes(brakes);
    }

    //// ROBOT ////
    @Override
    public void robotInit() {}

    // Robot Periodic
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    //// AUTO ////
    @Override
    public void autonomousInit() {
        // Turn on brakes
        switchMode(true);

        // Get (and run) command
        autoCommand = container.getAutonomousCommand();

        if (autoCommand != null) { autoCommand.schedule(); }
    }

    // Auto Periodic
    @Override
    public void autonomousPeriodic() {}

    //// TELEOP ////
    @Override
    public void teleopInit() {
        // Stop commands
        switchMode(true);
    }

    // Teleop Periodic
    @Override
    public void teleopPeriodic() {}

    //// DISABLED ////
    @Override
    public void disabledInit() {
        switchMode(false);
    }

    //// TEST ////
    @Override
    public void testInit() {
        switchMode(true);

        // Get and run test command
        autoCommand = container.getTestingCommand();

        if (autoCommand != null) { autoCommand.schedule(); }
    }

    // Test Periodic
    @Override
    public void testPeriodic() {}
}
