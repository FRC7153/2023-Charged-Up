// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

// TODO this is broken
public class PresetArmCommand extends CommandBase {
    private Arm armSubsys;
    private Translation2d pos;

    /** Creates a new PresetArmCommand. */
    public PresetArmCommand(Arm armSubsystem, Translation2d pos) {
        armSubsys = armSubsystem;

        this.pos = pos;

        addRequirements(armSubsys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsys.setTarget(pos.getX(), pos.getY());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsys.setAngle(0.0);
        armSubsys.setExtension(0.0);
    }

    // Close other commands
    @Override
    public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
