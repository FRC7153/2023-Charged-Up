package com.frc7153.math;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Uses acceleration and yaw from an IMU to track position.<br><br>
 * Uses trapezoidal integration for better accuracy.
 */
public class DeadReckoning {
    // Filters
    private LinearFilter xFilter, yFilter;

    // Constructor
    /**
     * Creates new object to track inertial position
     * @param numToAverage The number of acceleration inputs to average, to reduce noise. Higher = smoother, but slower
     */
    public DeadReckoning(int numToAverage) {
        xFilter = LinearFilter.movingAverage(numToAverage);
        yFilter = LinearFilter.movingAverage(numToAverage);
    }

    // State
    private Double lastIntegration = Double.NaN;
    public Translation2d velocity = new Translation2d(0.0, 0.0);
    private Translation2d prevAccel = new Translation2d(0.0, 0.0);
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
        if (lastIntegration.isNaN()) {lastIntegration = Timer.getFPGATimestamp(); return; }

        double timeDiff = Timer.getFPGATimestamp() - lastIntegration;
        lastIntegration = Timer.getFPGATimestamp();

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

        Translation2d accel = new Translation2d(yReport.getX() + xReport.getX(), yReport.getY() + xReport.getY());

        // Get Velocity

        /*velocity = velocity.plus(new Translation2d(
            (yReport.getX() + xReport.getX()) * timeDiff, 
            (yReport.getY() + xReport.getY()) * timeDiff
        ));*/

        velocity = velocity.plus(new Translation2d(
            (accel.getX() * timeDiff) + ((accel.getX() - prevAccel.getX()) / 2.0),
            (accel.getY() * timeDiff) + ((accel.getY() - prevAccel.getY()) / 2.0)
        ));

        // If accel has not changed, velocity is probably 0
        if (accel.getX() == prevAccel.getX() && accel.getY() == prevAccel.getY() && false) {
            velocity = new Translation2d(0, 0);
        }

        // Store acceleration
        prevAccel = accel;

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
    public Pose2d getPosition() { return pos; }

    /**
     * Get current position, as a string
     * @return Position
     */
    @Override
    public String toString() { return getPosition().toString(); }
}
