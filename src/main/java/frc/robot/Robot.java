// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    // Robot Container
    private RobotContainer container = new RobotContainer();

    // Running Command (command that is running for each mode)
    private Command runningCommand;

    // Stop auto command and toggle brakes
    private void switchMode(boolean brakes) {
        if (runningCommand != null) { runningCommand.cancel(); }

        container.toggleBrakes(brakes);
    }

    //// ROBOT ////
    @Override
    public void robotInit() {
        container.setLimelightLED(false);
    }

    // Robot Periodic
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        container.shuffleboardUpdate();
    }

    //// AUTO ////
    @Override
    public void autonomousInit() {
        DriverStation.reportWarning("-- AUTO INIT --", false);

        // Turn on brakes
        switchMode(true);

        // Get (and run) command
        runningCommand = container.getAutonomousCommand();

        if (runningCommand == null) {
            // If no auto command, just unlock hands
            CommandScheduler.getInstance().schedule(container.unlockClawCommand);
        } else {
            // Assumed that autos will unlock hands first
            //CommandScheduler.getInstance().schedule(autoCommand);
            runningCommand.schedule();
        }
    }

    // Auto Periodic
    @Override
    public void autonomousPeriodic() {}

    //// TELEOP ////
    @Override
    public void teleopInit() {
        // Stop commands
        switchMode(true);

        // Unlock
        if (container.checkHandsLocked()) {
            CommandScheduler.getInstance().schedule(container.unlockClawCommand); // Unlock
        }

        runningCommand = container.getTeleopCommand();

        if (runningCommand != null) {
            runningCommand.schedule();
        }
    }

    // Teleop Periodic
    @Override
    public void teleopPeriodic() {}

    //// DISABLED ////
    @Override
    public void disabledInit() {
        // Brakes only if the hands are not locked or its in test mode
        // At comps, these will ALWAYS lock, so make sure to retract arm before turning on
        switchMode(!(container.checkHandsLocked() && Constants.kTEST_DEPLOY));
    }

    //// TEST ////
    @Override
    public void testInit() {
        switchMode(false);

        // Get and run test command
        runningCommand = container.getTestingCommand();

        if (runningCommand != null) { runningCommand.initialize(); }
    }

    // Test Periodic
    @Override
    public void testPeriodic() { if (runningCommand != null) { runningCommand.execute();} }
}
