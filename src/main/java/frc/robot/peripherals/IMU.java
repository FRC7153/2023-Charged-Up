package frc.robot.peripherals;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * For reading rotation and acceleration of ADIS16470.
 */
public class IMU {
    // IMU
    public ADIS16470_IMU imu = new ADIS16470_IMU();

    /*
    // Set Position
    public void resetPose(Pose2d origin) {
        imu.reset();
        navigator.resetPose(origin);
    }

    // Get Position
    public void integrateAcceleration() {
        output2.setDouble(imu.getPitch());
        output3.setDouble(navigator.velocity.getX());

        debug.log(String.format("accel: %s, %s, %s; gyro: %s, %s, %s", round(imu.getAccelX()), round(imu.getAccelY()), round(imu.getAccelZ()), imu.getYaw(), imu.getRoll(), imu.getPitch()));
        if (!imu.isCalibrated()) { return; }

        navigator.integrateAcceleration(round(imu.getAccelX()), round(imu.getAccelY()), imu.getYaw());

        output1.setString(navigator.toString());

        //output1.setString(navigator.toString());
        //output1.setString(String.format("accel: %s %s %s, ", ));
    }
    */

    // Get Values
    public boolean isConnected() { return imu.isConnected(); }

    public double getYaw() { return imu.getAngle(); }
    public double getRoll() { return imu.getXComplementaryAngle(); }
    public double getPitch() { return imu.getYComplementaryAngle(); }
}
