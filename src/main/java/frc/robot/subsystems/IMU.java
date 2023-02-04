package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

/**
 * For reading rotation and acceleration of ADIS16470.
 */
public class IMU {
    // IMU
    private ADIS16448_IMU imu = new ADIS16448_IMU();

    // Get Values
    /**
     * @return Angle, in degrees (CCW positive)
     */
    public double getAngle() { return imu.getAngle(); }

    /**
     * @return X acceleration (meters per second per second)
     */
    public double getXAccel() { return imu.getAccelX(); }

    /**
     * @return Y acceleration (meters per second per second)
     */
    public double getYAccel() { return imu.getAccelY(); }

    /**
     * @return Z acceleration (meters per second per second)
     */
    public double getZAccel() { return imu.getAccelZ(); }
}
