package com.frc7153.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.frc7153.math.MathUtils;
import com.frc7153.swervedrive.wheeltypes.SwerveWheel;

public class SwerveBase extends SubsystemBase {
    // Wheels
    private SwerveWheel fl, fr, rl, rr;
    
    // Kinematics & Odometry
    private SwerveDriveKinematics kinematics;

    private SwerveDriveOdometry odometry;
    private Pose2d odometryPosition;

    // Max Speeds
    private double maxDriveSpeed = 4.0;
    private double maxSpinSpeed = 360.0;

    // Get Kinematics Object
    public SwerveDriveKinematics getKinematics() { return kinematics; }

    // Get Module Position
    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                fl.getState(),
                fr.getState(),
                rl.getState(),
                rr.getState()
        };
    }
    
    /**
     * Creates a new SwerveBase, with four SwerveWheels. Odometry is not started by default
     * @param frontLeft
     * @param frontRight
     * @param rearLeft
     * @param rearRight
     */
    public SwerveBase(SwerveWheel frontLeft, SwerveWheel frontRight, SwerveWheel rearLeft, SwerveWheel rearRight) {
        fl = frontLeft;
        fr = frontRight;
        rl = rearLeft;
        rr = rearRight;

        kinematics = new SwerveDriveKinematics(fl.getPosition(), fr.getPosition(), rl.getPosition(), rr.getPosition());
    }

    /**
     * Starts (or restarts) odometry
     * @param gyroAngle The angle of the gyroscope (degrees)
     * @param startX Starting X position (meters)
     * @param startY Starting Y position (meters)
     * @param startRot The starting rotation
     */
    public void startOdometry(double gyroAngle, double startX, double startY, double startRot) {
        odometry = new SwerveDriveOdometry(
            kinematics, 
            Rotation2d.fromDegrees(gyroAngle), 
            getSwerveModulePositions(),
            new Pose2d(startX, startY, Rotation2d.fromDegrees(startRot))
        );
    }

    /**
     * Sets the max speed for all the wheels. See the specific classes for units.
     * @param driveSpeed Max speed of driving (Meters/Second)
     * @param spinSpeed Max speed of spin (Degrees/Second)
     */
    public void setMaxSpeed(double driveSpeed, double spinSpeed) {
        maxDriveSpeed = driveSpeed;
        maxSpinSpeed = spinSpeed;
    }

    /**
     * @return The max drive speed (meters/second)
     */
    public double getMaxDriveSpeed() { return maxDriveSpeed; }

    /**
     * @return The max spin speed (degrees/second)
     */
    public double getMaxSpinSpeed() { return maxSpinSpeed; }

    /**
     * Scale down speeds and distribute them to the wheels
     * @param y
     * @param x
     * @param r
     */
    public void distributeStates(SwerveModuleState[] states) {
        //SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);

        fl.set(states[0]);
        fr.set(states[1]);
        rl.set(states[2]);
        rr.set(states[3]);
    }

    /**
     * Drives the robot
     * @param y forward/backward speed (meters per second)
     * @param x left/right speed (meters per second)
     * @param r rotation speed (degrees per second)
     */
    public void driveAbsolute(double y, double x, double r) {
        ChassisSpeeds speed = new ChassisSpeeds(y, x, -Units.degreesToRadians(r));
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed, new Translation2d(0, 0));

        distributeStates(states);
    }

    /**
     * Drives the robot, using a percentage for speed
     * @param y forward/backward speed (percentage of max drive speed)
     * @param x left/right speed (percentage of max drive speed)
     * @param r rotation speed (percentage of max spin speed)
     */
    public void drive(double y, double x, double r) { driveAbsolute(y*maxDriveSpeed, x*maxDriveSpeed, r*maxSpinSpeed); }

    /**
     * Drives the robot, with field-oriented drive
     * @param y forward/backward speed (meters per second)
     * @param x left/right speed (meters per second)
     * @param r rotation speed (degrees per second)
     * @param deg current angle of the robot (degrees)
     */
    public void driveFieldOrientedAbsolute(double y, double x, double r, double deg) {
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
            y, x, 
            Units.degreesToRadians(r), 
            Rotation2d.fromDegrees(deg)
        );
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

        distributeStates(states);
    }

    /**
     * Drives the robot, with field-oriented drive, using a percentage for speed
     * @param y forward/backward speed (percentage of max drive speed)
     * @param x left/right speed (percentage of max drive speed)
     * @param r rotation (percentage of max spin speed)
     * @param deg current angle of the robot (degrees)
     */
    public void driveFieldOriented(double y, double x, double r, double deg) { driveFieldOrientedAbsolute(y*maxDriveSpeed, x*maxDriveSpeed, r*maxSpinSpeed, deg); }

    /**
     * Orbits the robot around a point. There is no non-absolute implementation of this yet.
     * @param rotation The speed, in degrees per second
     * @param centerX The X position of the center of rotation, relative to the robot's base (meters)
     * @param centerY The Y position of the center of rotation, relative to the robot's base (meters)
     */
    public void orbitAbsolute(double rotation, double centerX, double centerY) {
        ChassisSpeeds speed = new ChassisSpeeds(0.0, 0.0, -Units.degreesToRadians(rotation));
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed, new Translation2d(centerX, centerY));

        distributeStates(states);
    }

    /**
     * Drives the robot like its a tank drive base
     * @param left The speed of the left side of the robot (meters/second)
     * @param right The speed of the right side of the robot (meters/second)
     */
    public void tankDriveAbsolute(double left, double right) {
        fl.set(0.0, MathUtils.symmetricClamp(left, maxDriveSpeed));
        rl.set(0.0, MathUtils.symmetricClamp(left, maxDriveSpeed));
        fr.set(0.0, MathUtils.symmetricClamp(right, maxDriveSpeed));
        rr.set(0.0, MathUtils.symmetricClamp(right, maxDriveSpeed));
    }

    /**
     * Drives the robot like its a tank drive base, using percentages for speed
     * @param left The speed of the left side of the robot (percentage of max drive speed)
     * @param right The speed of the right side of the robot (percentage of max drive speed)
     */
    public void tankDrive(double left, double right) { tankDriveAbsolute(left*maxDriveSpeed, right*maxDriveSpeed); }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] state = kinematics.toSwerveModuleStates(speeds);
        
        distributeStates(state);
    }

    /**
     * Sets all the motors to a specific angle (for testing usually).
     * @param angle in degrees
     */
    public void setAngle(double angle) {
        fl.setAngle(angle);
        fr.setAngle(angle);
        rl.setAngle(angle);
        rr.setAngle(angle);
    }

    /**
     * Sets all the motors to a specific speed (for testing usually).
     * @param speed
     */
    public void setSpeed(double speed) {
        fl.setSpeed(speed);
        fr.setSpeed(speed);
        rl.setSpeed(speed);
        rr.setSpeed(speed);
    }
    
    /**
     * Stops the robot
     * @param reset Whether the wheels should return to a forward position
     */
    public void stop(boolean reset) {
        if (reset) {
            fl.set(0.0, 0.0);
            fr.set(0.0, 0.0);
            rl.set(0.0, 0.0);
            rr.set(0.0, 0.0);
        } else {
            fl.setSpeed(0.0);
            fr.setSpeed(0.0);
            rl.setSpeed(0.0);
            rr.setSpeed(0.0);
        }
    }

    /**
     * @param coast Whether the wheels should coast or not (brake)
     * @param freeze Whether the wheels should stop moving
     */
    public void toggleCoastMode(boolean coast, boolean freeze) {
        if (freeze) { stop(false); }
        
        fl.toggleCoastMode(coast);
        fr.toggleCoastMode(coast);
        rl.toggleCoastMode(coast);
        rr.toggleCoastMode(coast);
    }

    /**
     * Gets the position, according to odometry (wheel states)
     * @return The position
     */
    public Pose2d getOdometryPose() {
        if (odometryPosition == null) {
            DriverStation.reportWarning("Odometry position is null!", false);
            return new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        }
        return odometryPosition;
    }

    /**
     * Updates the odometry
     * @param gyroAngle
     */
    public void updateOdometry(double gyroAngle) {
        if (odometry == null) {
            DriverStation.reportWarning("Could not update odometry, it has not been started", false);
            return;
        }

        odometryPosition = odometry.update(
            Rotation2d.fromDegrees(gyroAngle), 
            getSwerveModulePositions()
        );
    }
}
