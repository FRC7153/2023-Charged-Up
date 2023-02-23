package com.frc7153.swervedrive.wheeltypes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import com.frc7153.math.PIDConstant;
import com.frc7153.swervedrive.SwerveBase;
import com.frc7153.swervedrive.SwerveMathUtils;

/**
 * Swerve Wheel that uses a Falcon500 for the drive motor and Neo Brushless (with CAN Spark Max) for spin motor.
 * Uses CANCoder absolute encoder for absolute position. <br><br>
 * <b>First used in 2023 season (Charged Up)</b>
 */
public class SwerveWheel_FN implements SwerveWheel {
    // CONFIG VALUES //
    private static double k_SPIN_RATIO = 150.0 / 7.0;
    private static double k_DRIVE_RATIO = 6.12; // For standard speed module
    private static double k_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI; // 3.75

    private static int k_SPIN_CURRENT_LIMIT = 40;
    private static int k_DRIVE_CURRENT_LIMIT = 20;
    private static int k_DRIVE_CURRENT_PEAK = 25;
    private static double k_DRIVE_CURRENT_PEAK_DURATION = 0.1;
    private static double k_DRIVE_DEADBAND = 0.5; // Meters ber second

    private static PIDConstant kSPIN_PID = new PIDConstant(0, 0.3, 0.00001, 0.0, 0.5, -10.0, 10.0);
    private static PIDConstant kDRIVE_PID = new PIDConstant(0, 0.05, 0.0, 0.0, 0.5, 0.0);

    private static double k_SPIN_MAX_ACCEL = 0.8;
    
    // Motors, Encoders, PID
    private TalonFX driveWheel;
    private CANSparkMax spinWheel;

    private RelativeEncoder spinRelEncoder;
    private SparkMaxPIDController spinPID;

    // Position
    private Translation2d pos;

    @Override
    public Translation2d getPosition() {return pos; }

    /**
     * Creates a new Swerve Wheel. Expects a Falcon500 (TalonFX) for the drive wheel and a Rev Brushless NEO for spin motor.
     * Both should communicate over CAN bus.
     * 
     * @param drive The CAN id of the drive wheel (Falcon500)
     * @param spin The CAN id of the spin wheel (Rev Brushless NEO)
     * @param canCoder The CAN id of the CANCoder
     * @param x The x position of the wheel, relative to the center of the base, in meters
     * @param y The y position of the wheel, relative to the center of the base, in meters
     * @param spinHomeLocation When the wheel's direction is 0 degrees (forward), what is the output of the ABSOLUTE encoder (degrees)?
     */
    public SwerveWheel_FN(int spin, int drive, int canCoder, double x, double y, double spinHomeLocation) {
        // Declare and Configure Motors
        driveWheel = new TalonFX(drive);
        spinWheel = new CANSparkMax(spin, MotorType.kBrushless);
        
        toggleCoastMode(false);

        spinWheel.setSmartCurrentLimit(k_SPIN_CURRENT_LIMIT);
        driveWheel.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, k_DRIVE_CURRENT_LIMIT, k_DRIVE_CURRENT_PEAK, k_DRIVE_CURRENT_PEAK_DURATION));
        driveWheel.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, k_DRIVE_CURRENT_LIMIT, k_DRIVE_CURRENT_PEAK, k_DRIVE_CURRENT_PEAK_DURATION));

        spinWheel.setInverted(true);

        // Declare and Configure Encoders
        CANCoder spinAbsEncoder = new CANCoder(canCoder);
        spinRelEncoder = spinWheel.getEncoder();
        
        spinAbsEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // Set Relative Encoder Offset
        spinRelEncoder.setPosition((spinAbsEncoder.getAbsolutePosition() - spinHomeLocation) * k_SPIN_RATIO / 360.0);

        // Spin PID
        spinPID = spinWheel.getPIDController();

        kSPIN_PID.apply(spinPID);

        // Drive PID
        driveWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        kDRIVE_PID.apply(driveWheel);

        /*
         * The X and Y values are implemented in WPI's library oddly:
         * "Positive x values represent moving toward the front of the robot whereas positive 
         * y values represent moving toward the left of the robot."
         * It is changed below to make it easier to understand.
         * 
         * See https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html 
         */
        pos = new Translation2d(y, -x);
    }

    /**
     * Enables/configs trapezoidal motion trajectory
     * @param base The robot base (used to get max drive speed and spin speed)
     */
    public void enableMotionAccelerationStrategy(SwerveBase base) {
        spinPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, kSPIN_PID.kSLOT);
        spinPID.setSmartMotionMaxAccel(k_SPIN_MAX_ACCEL, kSPIN_PID.kSLOT);
        spinPID.setSmartMotionMaxVelocity(base.getMaxDriveSpeed(), kSPIN_PID.kSLOT);
        spinPID.setSmartMotionMinOutputVelocity(0.05, kSPIN_PID.kSLOT);
    }

    // Get Angle from Relative Encoder (degrees)
    private double getAngleFromRelative() {
        return SwerveMathUtils.normalizeAngle180(spinRelEncoder.getPosition() * 360.0 / k_SPIN_RATIO);
    }

    // Coast
    @Override
    public void toggleCoastMode(boolean coast) {
        spinWheel.setIdleMode((coast) ? IdleMode.kCoast : IdleMode.kBrake);
        driveWheel.setNeutralMode((coast) ? NeutralMode.Coast : NeutralMode.Brake);
    }

    // Get State
    @Override
    public SwerveModulePosition getState() {
        return new SwerveModulePosition(
            SwerveMathUtils.falcon500PositionToRotations(driveWheel.getSelectedSensorPosition()) / k_DRIVE_RATIO * k_WHEEL_CIRCUMFERENCE,
            Rotation2d.fromDegrees(SwerveMathUtils.normalizeAngle360(getAngleFromRelative()))
        );
    }

    // Set State
    @Override
    public void setAngle(double angle) {
        angle = SwerveMathUtils.normalizeAngle180(angle); // Normalize -180 to 180
        angle = (angle / 360.0 * k_SPIN_RATIO); // Convert to NEO position
        angle = SwerveMathUtils.calculateContinuousMovement(spinRelEncoder.getPosition(), angle, k_SPIN_RATIO); // Find quickest route
        spinPID.setReference(angle, ControlType.kPosition, kSPIN_PID.kSLOT); // Set PID setpoint
    }

    @Override
    public void setSpeed(double speed) {
        speed = SwerveMathUtils.applyDeadband(speed, k_DRIVE_DEADBAND); // Apply deadband
        double velocity = (speed / k_WHEEL_CIRCUMFERENCE) * 60.0; // Convert to rotations per minute
        velocity *= k_DRIVE_RATIO; // Convert to Falcon500 position
        driveWheel.set(ControlMode.Velocity, SwerveMathUtils.rpmToFalcon500Velocity(velocity)); // Set set point, in Falcon500 encoder's velocity
    }

    @Override
    public void set(SwerveModuleState state) {
        state = SwerveModuleState.optimize(
            state, 
            Rotation2d.fromDegrees(SwerveMathUtils.normalizeAngle360(getAngleFromRelative()))
        );
        set(state.angle.getDegrees(), state.speedMetersPerSecond);
    }

    // Periodic
    @Override
    public void periodic() {}
}