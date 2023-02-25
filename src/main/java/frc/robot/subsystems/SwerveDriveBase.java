package frc.robot.subsystems;

import com.frc7153.swervedrive.SwerveBase;
import com.frc7153.swervedrive.wheeltypes.SwerveWheel_FN;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

/**
 * For controlling the swerve drive base
 */
public class SwerveDriveBase {
    // Wheels and Base
    private SwerveWheel_FN fl = new SwerveWheel_FN(8, 4, 12, -SwerveConstants.kWHEEL_DISTANCE.getX(), SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kFL_OFFSET);
    private SwerveWheel_FN fr = new SwerveWheel_FN(7, 3, 11, SwerveConstants.kWHEEL_DISTANCE.getX(), SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kFR_OFFSET);
    private SwerveWheel_FN rl = new SwerveWheel_FN(10, 6, 14, -SwerveConstants.kWHEEL_DISTANCE.getX(), -SwerveConstants.kWHEEL_DISTANCE.getY() , SwerveConstants.kRL_OFFSET);
    private SwerveWheel_FN rr = new SwerveWheel_FN(9, 5, 13, SwerveConstants.kWHEEL_DISTANCE.getX(), -SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kRR_OFFSET);

    private SwerveBase base = new SwerveBase(fl, fr, rl, rr, 0.0);

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
