package frc.robot.subsystems;

import com.frc7153.swervedrive.SwerveBase;
import com.frc7153.swervedrive.wheeltypes.SwerveWheel_FN;

import frc.robot.peripherals.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.peripherals.IMU;

/**
 * aimbot
 */
public class Aim extends SubsystemBase {
    // Drive Base
    private SwerveWheel_FN fl = new SwerveWheel_FN(9, 5, 13, SwerveConstants.kWHEEL_DISTANCE.getX(), -SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kFL_OFFSET);
    private SwerveWheel_FN fr = new SwerveWheel_FN(10, 6, 14, -SwerveConstants.kWHEEL_DISTANCE.getX(), -SwerveConstants.kWHEEL_DISTANCE.getY() , SwerveConstants.kFR_OFFSET);
    private SwerveWheel_FN rl = new SwerveWheel_FN(7, 3, 11, SwerveConstants.kWHEEL_DISTANCE.getX(), SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kRL_OFFSET);
    private SwerveWheel_FN rr = new SwerveWheel_FN(8, 4, 12, -SwerveConstants.kWHEEL_DISTANCE.getX(), SwerveConstants.kWHEEL_DISTANCE.getY(), SwerveConstants.kRR_OFFSET);

    private SwerveBase base = new SwerveBase(fl, fr, rl, rr);
    public IMU imu = new IMU();

//limelight reqs
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    public Limelight limelightF = new Limelight("limelight-front");
    private NetworkTableEntry ty = table.getEntry("ty"); 
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry tv = table.getEntry("tv");
    private Double xCache;
    private Double xCacheTime = -1.0;
    private PIDController pid = new PIDController(0.015, 0.005, 0.0);
    private Double maxSpeed = 0.4;
    //fuck it im just ripping everything

    // Reset odometry on boot
    public Aim() {
        pid.setSetpoint(0.0); // Adjust error here
        base.setMaxSpeed(2.0, 360.0);
    }

    // Get Odometry Position
    public Pose2d getPose() { return base.getOdometryPose(); }

    // Reset Odometry Position
    public void setPose(Pose2d origin) {
        base.startOdometry(imu.getYaw(), origin.getX(), origin.getY(), origin.getRotation().getDegrees());
        //imu.resetPose(origin);
    }

    // Update odometry
    @Override
    public void periodic() {
        base.updateOdometry(imu.getYaw());
    }

    //ll auto center

    private double getX() {
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

    private double clampSpeed(double speed) {
        return Math.min(Math.max(speed, -maxSpeed), maxSpeed);
    }

    public double getTurn() {
        double x = getX();
        double pidOut = pid.calculate(x);
        return clampSpeed(pidOut);
    }


    
    // Drive
    public void stop() { base.stop(true); }
    public void driveFieldOriented(double x, double y, double rot) { base.driveFieldOriented(y, x, getTurn(), imu.getYaw()); }
    //public void driveRobotOriented(double x, double y, double rot) { base.drive(y, x, rot); }
    //public void driveTankAbsolute(double lSpeed, double rSpeed) { base.tankDriveAbsolute(lSpeed, rSpeed);}
    public void setCoast(boolean coast) { base.toggleCoastMode(coast, true); }
}
