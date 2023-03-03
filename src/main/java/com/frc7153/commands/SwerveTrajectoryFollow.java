// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc7153.commands;

import java.io.IOException;
import java.util.function.Supplier;

import com.frc7153.swervedrive.SwerveBase;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveTrajectoryFollow extends CommandBase {
    // Trajectory Control
    private Trajectory trajectory;
    private RamseteController rController;
    private SwerveBase swerveBase;
    private Supplier<Pose2d> poseSupplier;
    private Timer timer = new Timer();

    // Whether trajectory was loaded
    private boolean loaded;

    /** Creates a new FollowTrajectory. */
    public SwerveTrajectoryFollow(String trajectoryFileName, RamseteController ramsete, SwerveBase base, Supplier<Pose2d> odometrySupplier) {
        // Try to load JSON file
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(String.format("paths/%s", trajectoryFileName))
            );
            loaded = true;
        } catch (IOException e) {
            DriverStation.reportWarning(String.format("Could not load trajectory %s: %s", trajectoryFileName, e), false);
            trajectory = new Trajectory();
            loaded = false;
        }

        // Save objects
        rController = ramsete;
        swerveBase = base;
        poseSupplier = odometrySupplier;

        // Start timer
        timer.start();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset timer
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Check if trajectory failed to load
        if (!loaded) { return; }

        // Run current state
        State state = trajectory.sample(timer.get());
        swerveBase.setChassisSpeeds(rController.calculate(poseSupplier.get(), state));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop, but do not reset wheels
        swerveBase.stop(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Check if time has expired
        return timer.get() > trajectory.getTotalTimeSeconds();
    }
}
