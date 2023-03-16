package frc.robot.subsystems;

import java.util.HashMap;

import com.frc7153.swervedrive.SwerveBase;
import com.frc7153.swervedrive.wheeltypes.SwerveWheel_FN;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.peripherals.IMU;

/**
 * For controlling the swerve drive base
 */
public class DriveBase extends SubsystemBase {
    // Drive Base
    private SwerveWheel_FN fl = new SwerveWheel_FN(9, 5, 13, SwerveConstants.kWHEEL_DISTANCE.getX(), -SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kFL_OFFSET);
    private SwerveWheel_FN fr = new SwerveWheel_FN(10, 6, 14, -SwerveConstants.kWHEEL_DISTANCE.getX(), -SwerveConstants.kWHEEL_DISTANCE.getY() , SwerveConstants.kFR_OFFSET);
    private SwerveWheel_FN rl = new SwerveWheel_FN(7, 3, 11, SwerveConstants.kWHEEL_DISTANCE.getX(), SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kRL_OFFSET);
    private SwerveWheel_FN rr = new SwerveWheel_FN(8, 4, 12, -SwerveConstants.kWHEEL_DISTANCE.getX(), SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kRR_OFFSET);

    private SwerveBase base = new SwerveBase(fl, fr, rl, rr);
    public IMU imu = new IMU();

    // Constructor (init)
    public DriveBase() {
        // Start odometry by default
        setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

        // Config max speeds (only used for percentages, not autos)
        base.setMaxSpeed(4.75, 540.0); // 4.0, 360
    }

    // Get Odometry Position
    public Pose2d getPose() {
        Pose2d baseOdometry = new Pose2d(
            base.getOdometryPose().getX(),
            base.getOdometryPose().getY(),
            base.getOdometryPose().getRotation()
        );
        DriverStation.reportWarning(baseOdometry.toString(), false);
        DriverStation.reportWarning(DriverStation.getAlliance().name(), false);
        return baseOdometry;
    }

    // Reset Odometry Position
    public void setPose(Pose2d origin) {
        base.startOdometry(imu.getYaw(), origin.getX(), origin.getY(), origin.getRotation().getDegrees());
        //imu.resetPose(origin);
    }

    // Update odometry
    @Override
    public void periodic() {
        base.updateOdometry(imu.getYaw());
    }

    // Autonomous Command Creator
    public Command getTrajectoryCommand(String trajectoryName, HashMap<String, Command> eventMap, boolean resetPos, double maxVelocity, double maxAccel) {
        PathPlannerTrajectory trajectory;

        try {
            trajectory = PathPlanner.loadPath(
                trajectoryName, 
                maxVelocity,
                maxAccel
            );
        } catch (Exception e) {
            DriverStation.reportWarning(String.format("Could not load trajectory '%s': %s", trajectoryName, e), false);
            
            return new InstantCommand(() -> {
                DriverStation.reportWarning(String.format("Could not run trajectory '%s'", trajectoryName), false);
            });
        }

        PPSwerveControllerCommand trajectoryCommand = new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            this.base.getKinematics(),
            new PIDController(0.9, 0.0, 0.0),
            new PIDController(0.9, 0.0, 0.0),
            new PIDController(0.9, 0.0, 0.0),
            base::distributeStates,
            false,
            this
        );

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (resetPos) { this.setPose(trajectory.getInitialHolonomicPose()); }
            }),
            new FollowPathWithEvents(trajectoryCommand, trajectory.getMarkers(), eventMap)
        );
    }

    public Command getTrajectoryCommand(String trajectoryName, HashMap<String, Command> eventMap, boolean resetPos) { return getTrajectoryCommand(trajectoryName, eventMap, resetPos, 2.0, 1.5); }
    
    // Drive
    public void stop() { base.stop(true); }
    public void driveFieldOriented(double x, double y, double rot) { base.driveFieldOriented(y, x, rot, imu.getYaw()); }
    public void driveRobotOriented(double x, double y, double rot) { base.drive(y, x, rot); }
    public void driveTankAbsolute(double lSpeed, double rSpeed) { base.tankDriveAbsolute(lSpeed, rSpeed);}
    public void setCoast(boolean coast) { base.toggleCoastMode(coast, true); }
}
