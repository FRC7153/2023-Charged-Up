//because i remembered why i REALLY hate commandbased robots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;

public class AutoCenter {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    private NetworkTableEntry ty = table.getEntry("ty"); 
    private static NetworkTableEntry tx = table.getEntry("tx");
    private static NetworkTableEntry tv = table.getEntry("tv");
    private static PIDController pid = new PIDController(0.015, 0.005, 0.0);

    private static Double xCache;
    private Double yCache;
    private static Double xCacheTime = -1.0;
    private Double yCacheTime = -1.0;

    private static Double maxSpeed = 0.4;
    private Double err = 0.5;

    public AutoCenter() {
        pid.setSetpoint(0.0); // Adjust error here
    }

    public static double getX() {
        if (tv.getDouble(0.0) == 1.0) {
            xCache = tx.getDouble(15.0);
            xCacheTime = Timer.getFPGATimestamp();
            return xCache;
        } else if (Timer.getFPGATimestamp() - xCacheTime <= 1.5 && xCacheTime != -1) {
            return xCache;
        } else {
            return 15.0;
        }
    }

    private double getY() {
        if (tv.getDouble(0.0) == 1.0) {
            yCache = ty.getDouble(0.0);
            yCacheTime = Timer.getFPGATimestamp();
            return yCache;
        } else if (Timer.getFPGATimestamp() - yCacheTime <= 1.5 && yCacheTime != -1) {
            return yCache;
        } else {
            return 0.0;
        }
    }

    private static double clampSpeed(double speed) {
        return Math.min(Math.max(speed, -maxSpeed), maxSpeed);
    }

    public static double getTurn() {
        double x = getX();
        double pidOut = pid.calculate(x);
        return clampSpeed(pidOut);
    }


    public boolean isInTarget() {
        if (tv.getDouble(0.0) == 0.0) { return false; }
        if (Math.abs(getX()) < err) { return true; }
        return false;
    }


}
