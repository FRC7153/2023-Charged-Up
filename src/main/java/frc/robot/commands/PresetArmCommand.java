// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frc7153.math.MathUtils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.peripherals.IMU;
import frc.robot.subsystems.Arm;

public class PresetArmCommand extends CommandBase {
    private Arm armSubsys;
    private IMU imu;
    private Translation2d pos;
    private boolean checkInvert;

    /** Creates a new PresetArmCommand. */
    public PresetArmCommand(Arm armSubsystem, IMU imu, Translation2d pos, boolean checkInvert) {
        armSubsys = armSubsystem;
        this.imu = imu;

        this.pos = pos;
        this.checkInvert = checkInvert;

        addRequirements(armSubsys);
    }

    public PresetArmCommand(Arm armSubsystem, Translation2d pos) { this(armSubsystem, null, pos, false); }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (checkInvert && MathUtils.normalizeAngle180(imu.getYaw()) < 90.0 && MathUtils.normalizeAngle180(imu.getYaw()) > -90.0) {
            armSubsys.setTarget(-pos.getX(), pos.getY());
        } else {
            armSubsys.setTarget(pos.getX(), pos.getY());
        }
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
