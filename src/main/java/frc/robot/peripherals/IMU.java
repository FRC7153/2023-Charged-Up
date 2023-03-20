package frc.robot.peripherals;

import com.frc7153.math.MathUtils;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

/**
 * For reading rotation and acceleration of ADIS16470.
 */
public class IMU {
    // IMU
    private ADIS16470_IMU imu = new ADIS16470_IMU();
    private double lastCalibration = Timer.getFPGATimestamp();
    
    // Set Yaw on init
    public IMU() {
        imu.setYawAxis(IMUAxis.kY);
    }

    // Calibrate
    public void calibrate() {
        imu.configCalTime(CalibrationTime._1s);
        imu.calibrate();
        imu.reset();
        lastCalibration = Timer.getFPGATimestamp();
        DriverStation.reportWarning("Calibrating Gyro!", false);
    }

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
    public boolean isCalibrated() { return imu.isConnected() && Timer.getFPGATimestamp() - lastCalibration >= 1.0; }

    public double getYaw() { return MathUtils.normalizeAngle360(imu.getAngle()); }
    public double getRoll() { return MathUtils.normalizeAngle360(imu.getYComplementaryAngle()); }
    public double getPitch() { return MathUtils.normalizeAngle360(imu.getXComplementaryAngle()); }
}
