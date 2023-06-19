package frc.robot.peripherals;

import com.frc7153.controllers.ADIS16470_3Axis;
import com.frc7153.controllers.ADIS16470_3Axis.CalibrationTime;
import com.frc7153.controllers.ADIS16470_3Axis.IMUAxis;
import com.frc7153.math.MathUtils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * For reading rotation and acceleration of ADIS16470.
 */
public class IMU {
    // IMU
    private ADIS16470_3Axis imu = new ADIS16470_3Axis(IMUAxis.kY, IMUAxis.kX, IMUAxis.kZ, CalibrationTime._2s);
    private double lastCalibration = Timer.getFPGATimestamp();

    // Calibrate
    public void calibrate() {
        imu.configCalTime(CalibrationTime._2s);
        imu.calibrate();
        imu.resetAllAngles();
        
        lastCalibration = Timer.getFPGATimestamp();
        DriverStation.reportWarning("Calibrating Gyro!", false);
    }

    // Get Values
    public boolean isConnected() { return imu.isConnected(); }
    public boolean isCalibrated() { return imu.isConnected() && Timer.getFPGATimestamp() - lastCalibration >= 2.0; }

    public double getYaw() { return MathUtils.normalizeAngle180(imu.getAngle(imu.getYawAxis())); }
    public double getRoll() { return MathUtils.normalizeAngle180(imu.getAngle(imu.getRollAxis())); }
    public double getPitch() { return MathUtils.normalizeAngle180(imu.getAngle(imu.getPitchAxis())); }
    public double getPitchRate() { return imu.getRate(imu.getPitchAxis()); }
}
