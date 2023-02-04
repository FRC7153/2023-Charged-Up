package frc.robot.subsystems;

import com.frc7153.SwerveDrive.SwerveBase;
import com.frc7153.SwerveDrive.WheelTypes.SwerveWheel_FN;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

/**
 * For controlling the swerve drive base
 */
public class SwerveDriveBase {
    // Base Size
    private double halfHeight = Units.inchesToMeters(30.5)/2.0;
    private double halfWidth = Units.inchesToMeters(20.0)/2.0;
    
    // Wheels and Base
    private SwerveWheel_FN fl = new SwerveWheel_FN(8, 4, 12, -halfWidth, halfHeight, 181.67);
    private SwerveWheel_FN fr = new SwerveWheel_FN(7, 3, 11, halfWidth, halfHeight, 178.77);
    private SwerveWheel_FN rl = new SwerveWheel_FN(10, 6, 14, -halfWidth, -halfHeight , 8.35);
    private SwerveWheel_FN rr = new SwerveWheel_FN(9, 5, 13, halfWidth, -halfHeight, 17.139);

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
}
