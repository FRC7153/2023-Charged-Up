package com.frc7153.SwerveDrive.WheelTypes;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import com.frc7153.SwerveDrive.SwerveBase;
import com.frc7153.SwerveDrive.SwerveMathUtils;

/**
 * Swerve Wheel that uses a Falcon500 for the drive motor and Neo Brushless (with CAN Spark Max) for spin motor.
 * Uses CANCoder absolute encoder for absolute position. <br><br>
 * <b>First used in 2023 season (Charged Up)</b>
 */
public class SwerveWheel_FN implements SwerveWheel {
    // CONFIG VALUES //
    private static double k_SPIN_RATIO = 150.0 / 7.0;
    private static double k_DRIVE_RATIO = 6.12; // For standard speed module TODO check this
    private static double k_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI; // 3.75

    private static int k_SPIN_CURRENT_LIMIT = 40;
    private static int k_DRIVE_CURRENT_LIMIT = 20;
    private static int k_DRIVE_CURRENT_PEAK = 25;
    private static double k_DRIVE_CURRENT_PEAK_DURATION = 0.1;
    private static double k_DRIVE_ERR = 0.5;
    private static double k_DRIVE_DEADBAND = 0.5; // Meters ber second

    private static int k_SPIN_PID_INDEX = 0;
    private static int k_DRIVE_PID_INDEX = 0;

    private static double spin_kP = 0.3;
    private static double spin_kI = 0.00001;
    private static double spin_kD = 0.0;
    private static double spin_kO = 10.0;

    private static double drive_kP = 0.05;
    private static double drive_kI = 0.0;
    private static double drive_kD = 0.0;
    private static double drive_kF = 0.0;

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

        if (spinPID.getP(k_SPIN_PID_INDEX) != spin_kP || spinPID.getI(k_SPIN_PID_INDEX) != spin_kI || spinPID.getD(k_SPIN_PID_INDEX) != spin_kD|| spinPID.getOutputMax(k_SPIN_PID_INDEX) != spin_kO || spinPID.getOutputMin(k_SPIN_PID_INDEX) != -spin_kO) {
            spinPID.setP(spin_kP, k_SPIN_PID_INDEX);
            spinPID.setI(spin_kI, k_SPIN_PID_INDEX);
            spinPID.setD(spin_kD, k_SPIN_PID_INDEX);
            spinPID.setOutputRange(-spin_kO, spin_kO, k_SPIN_PID_INDEX);
            spinWheel.burnFlash();
            DriverStation.reportWarning(String.format("Had to re-write Swerve PID values for CAN Spark Max %s", spin), false);
        }

        // Drive PID
        driveWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveWheel.configAllowableClosedloopError(k_DRIVE_PID_INDEX, k_DRIVE_ERR);
        
        driveWheel.config_kP(k_DRIVE_PID_INDEX, drive_kP);
        driveWheel.config_kI(k_DRIVE_PID_INDEX, drive_kI);
        driveWheel.config_kD(k_DRIVE_PID_INDEX, drive_kD);
        driveWheel.config_kF(k_DRIVE_PID_INDEX, drive_kF);

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
        spinPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, k_SPIN_PID_INDEX);
        spinPID.setSmartMotionMaxAccel(k_SPIN_MAX_ACCEL, k_SPIN_PID_INDEX);
        spinPID.setSmartMotionMaxVelocity(base.getMaxDriveSpeed(), k_SPIN_PID_INDEX);
        spinPID.setSmartMotionMinOutputVelocity(0.05, k_SPIN_PID_INDEX);
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
            SwerveMathUtils.falcon500VelocityToRPM(driveWheel.getSelectedSensorVelocity()) / k_DRIVE_RATIO * k_WHEEL_CIRCUMFERENCE / 60.0, 
            Rotation2d.fromDegrees(SwerveMathUtils.normalizeAngle360(getAngleFromRelative()))
        );
    }

    // Set State
    @Override
    public void setAngle(double angle) {
        angle = SwerveMathUtils.normalizeAngle180(angle); // Normalize -180 to 180
        angle = (angle / 360.0 * k_SPIN_RATIO); // Convert to NEO position
        angle = SwerveMathUtils.calculateContinuousMovement(spinRelEncoder.getPosition(), angle, k_SPIN_RATIO); // Find quickest route
        spinPID.setReference(angle, ControlType.kPosition, k_SPIN_PID_INDEX); // Set PID setpoint
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