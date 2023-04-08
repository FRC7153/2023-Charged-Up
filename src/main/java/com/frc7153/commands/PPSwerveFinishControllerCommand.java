package com.frc7153.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Extends the custom PathPlanner version of SwerveControllerCommand, making it not finish
 * until the robot has actually reached (is within the tolerance of) the endpoint.
 */
public class PPSwerveFinishControllerCommand extends PPSwerveControllerCommand {
    // Error tolerance
    private double translationTolerance;
    private double thetaTolerance;

    // These objects are needed
    private Supplier<Pose2d> m_poseSupplier;
    private Pose2d m_finalState;

    // Constructor
    /**
     * Constructs a new PPSwerveFinishControllerCommand that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * @param trajectory The trajectory to follow.
     * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
     *     to provide this.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param xController The Trajectory Tracker PID controller for the robot's x position.
     * @param yController The Trajectory Tracker PID controller for the robot's y position.
     * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputModuleStates The raw output module states from the position controllers.
     * @param useAllianceColor Should the path states be automatically transformed based on alliance
     *     color? In order for this to work properly, you MUST create your path on the blue side of
     *     the field.
     * @param translationTolerance The acceptable error for the X and Y position of the robot (meters)
     * @param thetaTolerance The acceptable error for the theta (rotation) position of the robot (degrees)
     * @param requirements The subsystems to require.
     */
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
        // First check if the parent command has finished
        if (!super.isFinished()) { return false; }

        // Check tolerance
        Pose2d pose = m_poseSupplier.get();

        return (
            Math.abs(pose.getX() - m_finalState.getX()) <= translationTolerance &&
            Math.abs(pose.getY() - m_finalState.getY()) <= translationTolerance &&
            Math.abs(pose.getRotation().getDegrees() - m_finalState.getRotation().getDegrees()) <= thetaTolerance
        );
    }
}
