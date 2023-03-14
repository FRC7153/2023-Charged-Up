package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class SwerveTrajectoryCommand extends CommandBase {
    // Objects
    private DriveBase base;
    private HolonomicDriveController controller;

    // File location
    private String loc;

    // Path
    private PathPlannerTrajectory trajectory;
    private boolean loaded = false;
    private double start;

    // Constructor
    public SwerveTrajectoryCommand(DriveBase swerveBase, String deployDirName, String fileName) {
        loc = String.format("%s/%s", deployDirName, fileName);
        base = swerveBase;

        loadTrajectory();
    }

    /**
     * Assumes path name is 'paths' inside of the deploy directory
     * @param fileName
     */
    public SwerveTrajectoryCommand(DriveBase swerveBase, String fileName) { this(swerveBase, "paths", fileName); }

    /**
     * Loads the trajectory into memory
     * @return Whether it was success
     */
    public boolean loadTrajectory() {
        if (loaded) { return true; }

        trajectory = PathPlanner.loadPath(loc, new PathConstraints(start, start));
        /*try {
            //trajectory = TrajectoryUtil.fromPathweaverJson(loc);
            loaded = true;
            return true;
        } catch (IOException e) {
            DriverStation.reportWarning(String.format("Failed to load trajectory '%s': %s", loc.toString(), e), false);
            return false;
        }*/
        return false;
    }

    // Start
    @Override
    public void initialize() {
        base.setPose(trajectory.getInitialPose());
        start = Timer.getFPGATimestamp();
    }

    // Drive
    @Override
    public void execute() {
        /*ChassisSpeeds chassis = base.holonomicDrive.calculate(
            base.getPose(), 
            trajectory.sample(Timer.getFPGATimestamp() - start), 
            Rotation2d.fromDegrees(0.0)
        );*/

        trajectory.sample(3.4);
    }
}
