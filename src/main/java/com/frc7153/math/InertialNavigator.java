package com.frc7153.math;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Uses acceleration and yaw to track position
 */
public class InertialNavigator {
    // Filters
    private LinearFilter xFilter = LinearFilter.movingAverage(10);
    private LinearFilter yFilter = LinearFilter.movingAverage(10);

    // State
    private Double lastIntegration = Double.NaN;
    private Translation2d velocity = new Translation2d(0.0, 0.0);
    private Pose2d pos = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    /**
     * Resets the position.
     * The gyroscope's angle should be reset too
     * @param origin The position it is at
     */
    public void resetPose(Pose2d origin) {
        pos = origin;
    }

    /**
     * Integrates the acceleration
     * @param xAccel Acceleration, m/s^2
     * @param yAccel Acceleration m/s^2
     * @param yaw Degrees
     */
    public void integrateAcceleration(double xAccel, double yAccel, double yaw) {
        // Get time
        if (lastIntegration.isNaN()) { lastIntegration = Timer.getFPGATimestamp(); return; }

        double timeDiff = Timer.getFPGATimestamp() - lastIntegration;
        lastIntegration = timeDiff;

        if (timeDiff > 1.0) {
            DriverStation.reportWarning(String.format("Last acceleration integration is very state (%s seconds ago)", timeDiff), false);
        }

        // Get accel
        xAccel = xFilter.calculate(xAccel);
        yAccel = yFilter.calculate(yAccel);

        // Calculate Acceleration With Yaw
        Translation2d yReport = new Translation2d(
            Math.cos(Units.degreesToRadians(yaw)) * yAccel,
            Math.sin(Units.degreesToRadians(yaw)) * yAccel
        );

        Translation2d xReport = new Translation2d(
            Math.cos(Units.degreesToRadians(yaw + 90.0)) * xAccel,
            Math.sin(Units.degreesToRadians(yaw + 90.0)) * xAccel
        );

        // Get Velocity
        velocity = velocity.plus(new Translation2d(
            (yReport.getX() + xReport.getX()) * timeDiff, 
            (yReport.getY() + xReport.getY()) * timeDiff
        ));

        // Get Position
        pos = new Pose2d(
            pos.getX() + (velocity.getX() * timeDiff), 
            pos.getY() + (velocity.getY() * timeDiff),
            Rotation2d.fromDegrees(yaw)
        );
    }

    /**
     * Get current position
     * @return Current position from acceleration
     */
    public Pose2d getPosition() { return pos;}

    /**
     * Get current position, as a string
     * @return Position
     */
    @Override
    public String toString() { return getPosition().toString(); }
}
