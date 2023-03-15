// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.GameState;

public class Robot extends TimedRobot {
    // Robot Container
    private RobotContainer container = new RobotContainer();

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
        container.shuffleboardUpdate();
    }

    //// AUTO ////
    @Override
    public void autonomousInit() {
        container.state = GameState.AUTO;

        // Turn on brakes
        switchMode(true);

        // Unlock
        if (container.checkHandsLocked()) {
            CommandScheduler.getInstance().schedule(container.unlockClawCommand);
        }

        // Get (and run) command
        autoCommand = container.getAutonomousCommand();

        if (autoCommand == null) {
            // If no auto command, just unlock hands
            CommandScheduler.getInstance().schedule(container.unlockClawCommand);
        } else {
            // Assumed that autos will unlock hands first
            autoCommand.schedule();
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

        container.state = GameState.TELEOP;
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
        container.state = GameState.DISABLED;
    }

    //// TEST ////
    @Override
    public void testInit() {
        switchMode(true);
        container.state = GameState.DISABLED;

        // Get and run test command
        autoCommand = container.getTestingCommand();

        if (autoCommand != null) { autoCommand.initialize(); }
    }

    // Test Periodic
    @Override
    public void testPeriodic() { if (autoCommand != null) { autoCommand.execute();} }
}
