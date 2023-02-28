// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

/**
 * Pre-moves the claw to either the front or back of the robot, depending on where the 
 * nearest game piece is (or which way the robot is driving)
 */
public class PremoveClawCommand extends CommandBase {
    // Subsystems
    private Arm arm;
    private Claw claw;

    // Constructor
    public PremoveClawCommand(Arm armSubsys, Claw clawSubsys) {
        arm = armSubsys;
        claw = clawSubsys;

        addRequirements(arm, claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
