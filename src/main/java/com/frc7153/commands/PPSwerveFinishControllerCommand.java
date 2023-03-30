package com.frc7153.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

// TODO javadocs
public class PPSwerveFinishControllerCommand extends PPSwerveControllerCommand {
    // Error tolerance
    private double translationTolerance;
    private double thetaTolerance;

    // These objects are needed
    private Supplier<Pose2d> m_poseSupplier;
    private Pose2d m_finalState;

    // Default Constructor
    public PPSwerveFinishControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      boolean useAllianceColor,
      double translationTolerance,
      double thetaTolerance,
      Subsystem... requirements) {
        super(trajectory, poseSupplier, kinematics, xController, yController, rotationController, outputModuleStates, useAllianceColor, requirements);

        // Save for later
        this.translationTolerance = translationTolerance;
        this.thetaTolerance = thetaTolerance;

        m_poseSupplier = poseSupplier;
        m_finalState = new Pose2d(
            trajectory.getEndState().poseMeters.getX(),
            trajectory.getEndState().poseMeters.getY(),
            trajectory.getEndState().holonomicRotation
        );
    }

    // Check end
    @Override
    public boolean isFinished() {
        if (!super.isFinished()) { return false; }

        Pose2d pose = m_poseSupplier.get();

        return (
            Math.abs(pose.getX() - m_finalState.getX()) <= translationTolerance &&
            Math.abs(pose.getY() - m_finalState.getY()) <= translationTolerance &&
            Math.abs(pose.getRotation().getDegrees() - m_finalState.getRotation().getDegrees()) <= thetaTolerance
        );
    }
}
