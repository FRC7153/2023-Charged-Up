package frc.robot.subsystems;

import java.util.HashMap;

import com.frc7153.commands.PPSwerveFinishControllerCommand;
import com.frc7153.math.MathUtils;
import com.frc7153.swervedrive.SwerveBase;
import com.frc7153.swervedrive.wheeltypes.SwerveWheel_FN;
import com.frc7153.validation.Validatable;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.peripherals.IMU;

/**
 * For controlling the swerve drive base
 */
public class DriveBase extends SubsystemBase implements Validatable {
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
        base.setMaxSpeed(5.25, 540.0); // 1.5, 360.0
    }

    // Get Odometry Position
    public Pose2d getPose() {
        // Collect positions
        /*Pose2d baseOdometry = new Pose2d(
            base.getOdometryPose().getX(),
            base.getOdometryPose().getY(),
            base.getOdometryPose().getRotation()
        );*/

        // Orient to field
        return new Pose2d(
            -base.getOdometryPose().getX(),
            4.0 - base.getOdometryPose().getY() + 4.0,
            Rotation2d.fromDegrees(-base.getOdometryPose().getRotation().getDegrees())
        );
    }

    // Reset Odometry Position (Relative to field)
    public void setPose(Pose2d origin) {
        DriverStation.reportWarning(origin.toString(), false);

        base.startOdometry(
            imu.getYaw(), 
            -origin.getX(), 
            4.0 - origin.getY() + 4.0, 
            MathUtils.normalizeAngle180(-origin.getRotation().getDegrees())
        );
        //imu.resetPose(origin);
    }

    // Update odometry
    @Override
    public void periodic() {
        base.updateOdometry(imu.getYaw());
    }

    // Set Wheel Speeds (from trajectory)
    public void setWheelStates(SwerveModuleState[] states) {
        for (SwerveModuleState s : states) {
            s.speedMetersPerSecond = -s.speedMetersPerSecond;
        }

        base.distributeStates(states);
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

        PPSwerveFinishControllerCommand trajectoryCommand = new PPSwerveFinishControllerCommand(
            trajectory,
            this::getPose,
            this.base.getKinematics(),
            AutoConstants.kDRIVE_PID.toWPIPidController(),
            AutoConstants.kDRIVE_PID.toWPIPidController(),
            AutoConstants.kTHETA_PID.toWPIPidController(),
            this::setWheelStates,
            false,
            0.5,
            2.0,
            this
        );

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (resetPos) { this.setPose(trajectory.getInitialHolonomicPose()); }
            }),
            new FollowPathWithEvents(trajectoryCommand, trajectory.getMarkers(), eventMap)
        );
    }

    // TODO
    /*public Command goToTrajectoryHomeCommand(String trajectoryName) {
        PathPlannerTrajectory
        try {

        }
    }*/

    public Command getTrajectoryCommand(String trajectoryName, HashMap<String, Command> eventMap, boolean resetPos) { return getTrajectoryCommand(trajectoryName, eventMap, resetPos, 2.0, 1.5); }
    
    // Drive
    public void stop() { base.stop(true); }
    public void setMaxSpeed(double drive, double angle) { base.setMaxSpeed(drive, angle); }

    public void driveFieldOriented(double x, double y, double rot) { base.driveFieldOriented(y, x, rot, imu.getYaw()); }
    public void driveRobotOriented(double x, double y, double rot) { base.drive(y, x, rot); }
    public void driveTankAbsolute(double lSpeed, double rSpeed) { base.tankDriveAbsolute(lSpeed, rSpeed);}
    public void driveDiag(double forSpeed, double ang) { base.setAngle(ang); base.setSpeed(forSpeed * base.getMaxDriveSpeed()); }

    public void setCoast(boolean coast) { base.toggleCoastMode(coast, true); }
    public void lockWheels() { base.stop(false); base.setAngle(90.0); }

    // Validate for faults
    private HashMap<String, Boolean> validationMap = new HashMap<>(17);

    @Override
    public HashMap<String, Boolean> validate() {
        validationMap.put("Swerve 1 - Neo", rl.validateNEO());
        validationMap.put("Swerve 1 - Falcon", rl.validateFalcon());
        validationMap.put("Swerve 1 - Abs Enc", rl.validateCANCoder());
        validationMap.put("Swerve 1 - Rel Enc", rl.validateRelEncoder());

        validationMap.put("Swerve 2 - Neo", rr.validateNEO());
        validationMap.put("Swerve 2 - Falcon", rr.validateFalcon());
        validationMap.put("Swerve 2 - Abs Enc", rr.validateCANCoder());
        validationMap.put("Swerve 2 - Rel Enc", rr.validateRelEncoder());

        validationMap.put("Swerve 3 - Neo", fl.validateNEO());
        validationMap.put("Swerve 3 - Falcon", fl.validateFalcon());
        validationMap.put("Swerve 3 - Abs Enc", fl.validateCANCoder());
        validationMap.put("Swerve 3 - Rel Enc", fl.validateRelEncoder());

        validationMap.put("Swerve 4 - Neo", fr.validateNEO());
        validationMap.put("Swerve 4 - Falcon", fr.validateFalcon());
        validationMap.put("Swerve 4 - Abs Enc", fr.validateCANCoder());
        validationMap.put("Swerve 4 - Rel Enc", fr.validateRelEncoder());

        validationMap.put("Gyro", imu.isCalibrated());

        return validationMap;
    }
}
