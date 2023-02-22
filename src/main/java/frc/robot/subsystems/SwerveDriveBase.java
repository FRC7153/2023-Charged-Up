package frc.robot.subsystems;

import com.frc7153.swervedrive.SwerveBase;
import com.frc7153.swervedrive.wheeltypes.SwerveWheel_FN;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * For controlling the swerve drive base
 */
public class SwerveDriveBase {
    // Wheels and Base
    private SwerveWheel_FN fl = new SwerveWheel_FN(8, 4, 12, -Constants.kWHEEL_DISTANCE.getX(), Constants.kWHEEL_DISTANCE.getY(), Constants.kFL_SWERVE_OFFSET);
    private SwerveWheel_FN fr = new SwerveWheel_FN(7, 3, 11, Constants.kWHEEL_DISTANCE.getX(), Constants.kWHEEL_DISTANCE.getY(), Constants.kFR_SWERVE_OFFSET);
    private SwerveWheel_FN rl = new SwerveWheel_FN(10, 6, 14, -Constants.kWHEEL_DISTANCE.getX(), -Constants.kWHEEL_DISTANCE.getY() , Constants.kRL_SWERVE_OFFSET);
    private SwerveWheel_FN rr = new SwerveWheel_FN(9, 5, 13, Constants.kWHEEL_DISTANCE.getX(), -Constants.kWHEEL_DISTANCE.getY(), Constants.kRR_SWERVE_OFFSET);

    private SwerveBase base = new SwerveBase(fl, fr, rl, rr, Robot.imu.getYaw());

    // Drive Teleop
    public void driveTeleop() {
        base.driveFieldOriented(
            Robot.controller0.getLeftY(), 
            Robot.controller0.getLeftX(), 
            Robot.controller0.getRightX(), 
            Robot.imu.getYaw()
        );
    }

    // Get Odometry Position
    public Pose2d getPose() { return base.getOdometricPosition(); }

    // Reset Odometry Position
    public void setPose(Pose2d origin) {
        base.startOdometry(origin.getRotation().getDegrees(), origin.getX(), origin.getY());
    }
}
