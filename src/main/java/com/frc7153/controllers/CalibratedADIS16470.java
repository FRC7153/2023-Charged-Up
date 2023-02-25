package com.frc7153.controllers;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Wrapper for the ADIS16470 that has acceleration calibration.
 */
public class CalibratedADIS16470 {
    // Axes
    private static IMUAxis[] axes = {IMUAxis.kX, IMUAxis.kY, IMUAxis.kZ};

    // IMU object
    private ADIS16470_IMU imu = new ADIS16470_IMU();

    // State
    private boolean calibrated = false;
    private double[] offsets = {0.0, 0.0, 0.0};

    /**
     * Creates new object, but does not calibrate it.<br><br>
     * @param yaw The yaw axis (0 = x, 1 = y, 2 = z)
     */
    public CalibratedADIS16470(int yaw) {
        imu.setYawAxis(axes[yaw]); 
        imu.configCalTime(CalibrationTime._4s);
    }

    // Calibrate
    public class CalibrateIMU extends CommandBase {
        private double start;

        // Start
        @Override
        public void initialize() {
            calibrated = false;
            imu.reset();

            offsets[0] = imu.getAccelX();
            offsets[1] = imu.getAccelY();
            offsets[2] = imu.getAccelZ();

            imu.calibrate();
            start = Timer.getFPGATimestamp();
        }

        // Periodic
        @Override
        public void execute() {
            offsets[0] = (offsets[0] + imu.getAccelX()) / 2.0;
            offsets[1] = (offsets[1] + imu.getAccelY()) / 2.0;
            offsets[2] = (offsets[2] + imu.getAccelZ()) / 2.0;
        }

        // Done
        @Override
        public void end(boolean terminated) { if (!terminated) { calibrated = true; } }

        // Is done (4 seconds)
        @Override
        public boolean isFinished() { return (Timer.getFPGATimestamp() - start) >= 15.0; }

        // Config
        @Override
        public boolean runsWhenDisabled() { return true; }

        @Override
        public String getName() { return "Calibrate"; }
    }

    /**
     * @return Whether the IMU has been calibrated yet (takes 4 seconds)
     */
    public boolean isCalibrated() { return calibrated; }

    /**
     * Resets the IMU
     */
    public void reset() { imu.reset(); }

    // Getters
    private void warnIfNotCalibrated() { if (!calibrated) { DriverStation.reportWarning("ADIS16470 is not yet calibrated!", false); } }

    /**
     * @return Current x acceleration of the gyro
     */
    public double getAccelX() { warnIfNotCalibrated(); return imu.getAccelX() - offsets[0]; }

    /**
     * @return Current y acceleration of the gyro
     */
    public double getAccelY() { warnIfNotCalibrated(); return imu.getAccelY() - offsets[1]; }

    /**
     * @return Current z acceleration of the gyro
     */
    public double getAccelZ() { warnIfNotCalibrated(); return imu.getAccelZ() - offsets[2]; }

    /**
     * @return Yaw angle in degrees, CCW positive
     */
    public double getYaw() { return imu.getAngle(); }

    /**
     * @return Roll angle in degrees, CCW positive
     */
    public double getRoll() { return imu.getXComplementaryAngle(); }

    /**
     * @return Pitch angle in degrees, CCW positive
     */
    public double getPitch() { return imu.getYComplementaryAngle(); }
}
