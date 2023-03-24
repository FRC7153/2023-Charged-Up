

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.GrabPositions;
import frc.robot.subsystems.Claw;

/**
 * Moves claw hands to specific position (Instant command)
 */
public class GrabCommand extends InstantCommand {
    /** Creates a new GrabCommand. */
    public GrabCommand(Claw clawSubsys, double lPos, double rPos) {
        super(() -> { clawSubsys.setPosition(lPos, rPos); }, clawSubsys);
    }

    public GrabCommand(Claw clawSubsys, GrabPositions pos) { this(clawSubsys, pos.lPos, pos.rPos); }
}