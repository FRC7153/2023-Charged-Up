package frc.robot.subsystems;

import java.sql.Driver;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;

/**
 * For reading rotation and acceleration of ADIS16470.
 */
public class IMU {
    // IMU
    private ADIS16470_IMU imu = new ADIS16470_IMU();

    // Odometry
    private LinearFilter xFilter = LinearFilter.movingAverage(10);
    private LinearFilter yFilter = LinearFilter.movingAverage(10);

    private Double lastIntegration = Double.NaN;
    private Translation2d velocity = new Translation2d(0.0, 0.0);
    private Pose2d imuPosition = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    private GenericEntry output1 = Shuffleboard.getTab("odometry").add("output1", "").getEntry();

    // Set Position
    public void resetPose(Pose2d origin) {
        imu.reset();

        Robot.driveBase.setPose(origin);
        imuPosition = origin;
    }

    // Get Position
    public void accumulatePosition() {
        // Get time
        if (Double.isNaN(lastIntegration)) { lastIntegration = Timer.getFPGATimestamp(); return; }

        double timeDiff = Timer.getFPGATimestamp() - lastIntegration;
        lastIntegration = Timer.getFPGATimestamp();

        if (timeDiff > 1.0) {
            DriverStation.reportWarning(String.format("Last acceleration integration is very stale (%s seconds ago)", timeDiff), false);
        }

        // Get accel
        double xAccel = xFilter.calculate(imu.getAccelX());
        double yAccel = yFilter.calculate(imu.getAccelY());

        // Calculate for turn
        Translation2d yReport = new Translation2d(
            Math.cos(Units.degreesToRadians(getYaw())) * yAccel,
            Math.sin(Units.degreesToRadians(getYaw())) * yAccel
        );

        Translation2d xReport = new Translation2d(
            Math.cos(Units.degreesToRadians(getYaw() + 90.0)) * xAccel,
            Math.sin(Units.degreesToRadians(getYaw() + 90.0)) * xAccel
        );

        velocity = velocity.plus(new Translation2d(
            (yReport.getX() + xReport.getX()) * timeDiff, 
            (yReport.getY() + xReport.getY()) * timeDiff
        ));

        imuPosition = new Pose2d(
            imuPosition.getX() + (velocity.getX() * timeDiff), 
            imuPosition.getY() + (velocity.getY() * timeDiff),
            Rotation2d.fromDegrees(getYaw())
        );

        output1.setString(imuPosition.toString());
    }

    // Get Values
    /**
     * @return Yaw angle, in degrees (CCW positive)
     */
    public double getYaw() { return imu.getAngle(); }

    /**
     * @return Pitch angle, in degrees
     */
    public double getPitch() { return imu.getXComplementaryAngle(); }

    /**
     * @return Roll angle, in degrees
     */
    public double getRoll() { return imu.getYComplementaryAngle(); }
}
