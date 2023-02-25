package frc.robot.subsystems;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import com.frc7153.controllers.CalibratedADIS16470;
import com.frc7153.logging.FileDump;
import com.frc7153.math.DeadReckoning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;

/**
 * For reading rotation and acceleration of ADIS16470.
 */
public class IMU {
    // IMU
    public CalibratedADIS16470 imu = new CalibratedADIS16470(0);

    // Odometry
    private DeadReckoning navigator = new DeadReckoning(10);
    private GenericEntry output1 = Shuffleboard.getTab("odometry").add("output1", "").getEntry();
    private GenericEntry output2  = Shuffleboard.getTab("odometry").add("val", 0.0).getEntry();
    private GenericEntry output3 = Shuffleboard.getTab("odometry").add("val2", 0.0).getEntry();

    private FileDump debug = new FileDump("IMU-debug");

    // Set Position
    public void resetPose(Pose2d origin) {
        imu.reset();

        Robot.driveBase.setPose(origin);
        navigator.resetPose(origin);
    }

    // Round to 1 decimal point
    private static double round(double input) {
        DecimalFormat format = new DecimalFormat("####.#");
        format.setRoundingMode(RoundingMode.DOWN);
        return Double.parseDouble(format.format(input));
    }

    // Get Position
    public void accumulatePosition() {
        output2.setDouble(round(imu.getAccelX()));
        output3.setDouble(navigator.velocity.getX());

        debug.log(String.format("accel: %s, %s, %s; gyro: %s, %s, %s", round(imu.getAccelX()), round(imu.getAccelY()), round(imu.getAccelZ()), imu.getYaw(), imu.getRoll(), imu.getPitch()));
        if (!imu.isCalibrated()) { return; }

        navigator.integrateAcceleration(round(imu.getAccelX()), round(imu.getAccelY()), imu.getYaw());

        output1.setString(navigator.toString());

        //output1.setString(navigator.toString());
        //output1.setString(String.format("accel: %s %s %s, ", ));
    }



    // Get Values
    public boolean isCalibrated() { return imu.isCalibrated(); }

    public double getYaw() { return imu.getYaw(); }
}
