package frc.robot.subsystems;

import com.frc7153.logging.FileDump;
import com.frc7153.math.InertialNavigator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;

/**
 * For reading rotation and acceleration of ADIS16470.
 */
public class IMU {
    // IMU
    private ADIS16470_IMU imu = new ADIS16470_IMU();

    // Odometry
    private InertialNavigator navigator = new InertialNavigator(10);
    private GenericEntry output1 = Shuffleboard.getTab("odometry").add("output1", "").getEntry();

    private FileDump debug = new FileDump("IMU-debug");

    public void calibrate() {
        imu.configCalTime(CalibrationTime._8s);
        imu.calibrate();
    }

    // Set Position
    public void resetPose(Pose2d origin) {
        imu.reset();

        Robot.driveBase.setPose(origin);
        navigator.resetPose(origin);
    }

    // Get Position
    public void accumulatePosition() {
        navigator.integrateAcceleration(imu.getAccelX(), imu.getAccelY(), imu.getAngle());

        //output1.setString(navigator.toString());
        //output1.setString(String.format("accel: %s %s %s, ", ));
        debug.log(String.format("accel: %s, %s, %s; gyro: %s, %s, %s", imu.getAccelX(), imu.getAccelY(), imu.getAccelZ(), imu.getYawAxis(), imu.getXComplementaryAngle(), imu.getYComplementaryAngle()));
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
