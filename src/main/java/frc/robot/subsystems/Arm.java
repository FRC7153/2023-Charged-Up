package frc.robot.subsystems;

import com.frc7153.math.Encoder;
import com.frc7153.math.MathUtils;
import com.frc7153.math.Encoder.Range;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    // Arm State
    public static class ArmState {
        public double angle;
        public double extension;

        public ArmState(double angle, double extension) { this.angle = angle; this.extension = extension; }
    }

    // Motors
    private CANSparkMax angleMotor = new CANSparkMax(16, MotorType.kBrushless);
    private CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless);

    // PID
    //private PIDController anglePID = ArmConstants.kARM_PID.toWPIPidController();
    private ProfiledPIDController anglePID = ArmConstants.kARM_PID.toWPIProfiledPidController(ArmConstants.kMAX_ANGLE_VELOCITY, ArmConstants.kMAX_ANGLE_ACCELERATION);
    private SparkMaxPIDController winchPID = winchMotor.getPIDController();

    private Double angleSP = Double.NaN;
    private Double extSP = Double.NaN;
    private Translation2d pose = new Translation2d(0.0, 0.0);
    private ArmState currentState = new ArmState(0.0, 0.0);

    // Encoders
    private Encoder angleAbsEncoder = new Encoder((new DutyCycleEncoder(8))::getAbsolutePosition);
    private RelativeEncoder winchEnc = winchMotor.getEncoder();

    // State
    public boolean hasBeenReleased = false;

    // Init
    public Arm() {
        // Config Arm
        angleMotor.setInverted(true);
        angleMotor.setSmartCurrentLimit(70);

        angleAbsEncoder.setConversionFactor(360.0);
        angleAbsEncoder.setInverted(false);
        angleAbsEncoder.setZeroOffset(0.0);
        angleAbsEncoder.setRange(Range.FROM_NEGATIVE_180_TO_180);

        // Config Winch
        winchMotor.setInverted(true);
        ArmConstants.kEXT_PID.apply(winchPID);
    }

    // Go to setpoint
    @Override
    public void periodic() { periodic(false); }

    public void periodic(boolean testing) {
        if (!DriverStation.isDisabled() && !angleSP.isNaN() && !extSP.isNaN()) {
            // Verify winch motor is safe
            if (winchEnc.getPosition() < 0.0) {
                DriverStation.reportWarning(String.format("Winch is below zero! -> %s rotations", winchEnc.getPosition()), false);
            }

            if (testing) {
                // Set angle (only testing)
                //anglePID.setSetpoint(angleSP);
                anglePID.setGoal(angleSP);
            } else {
                // Constrain and set position
                Translation2d commandedPose = kinematics(extSP, angleSP);
                double x = commandedPose.getX();
                double y = commandedPose.getY();

                double MAX_X = ArmConstants.kMAX_REACH + ArmConstants.kJOINT_TO_BUMPER_DIST - ArmConstants.kCLEARANCE;
                double MAX_Y = ArmConstants.kMAX_HEIGHT - ArmConstants.kCLEARANCE;
                double MIN_Y = ArmConstants.kCLEARANCE;

                if (x > MAX_X ) {
                    x = MAX_X;
                } else if (x < -MAX_X) {
                    x = -MAX_X;
                }

                if (y > MAX_Y) {
                    y = MAX_Y;
                } else if (y < MIN_Y) {
                    y = MIN_Y;
                }

                pose = new Translation2d(x, y);
                currentState = inverseKinematics(x, y);

                //anglePID.setSetpoint(currentState.angle);
                anglePID.setGoal(currentState.angle);
                winchPID.setReference(Math.max(ArmConstants.extToWinchRots(currentState.extension - ArmConstants.kHAND_LENGTH), 0.0), ControlType.kPosition, ArmConstants.kEXT_PID.kSLOT);

                //System.out.println(String.format("ext: %s, angle: %s -> %s, %s -> driven ext: %s, driven angle: %s", extSP, angleSP, x,y, newPos.getX(), newPos.getY()));
            }

            // Set angle voltage
            angleMotor.setVoltage(
                anglePID.calculate(angleAbsEncoder.getPosition())
            );
        }
    }

    /**
     * Set angle
     * @angle angle, in degrees
     */
    public void setAngle(double angle) { angleSP = angle; }
    
    /**
     * Set the extension
     * @param ext Total extension, joint to grab point
     */
    public void setExtension(double ext) { extSP = ext; }

    /**
     * Gets the position of the arm (forward kinematics)
     * @param ext Extension of arm, in inches (joint to grab point)
     * @param angle Angle of arm, in degrees from zero
     * @return Arm position
     */
    private Translation2d kinematics(double ext, double angle) {
        ext += ArmConstants.kHAND_LENGTH;

        return new Translation2d(
            Math.sin(Units.degreesToRadians(angle)) * ext,
            (Math.cos(Units.degreesToRadians(angle)) * ext) + ArmConstants.kJOINT_TO_FLOOR_DIST
        );
    }

    // Inverse kinematics
    // X is extension, y is angle
    private ArmState inverseKinematics(double x, double y) {
        return new ArmState(
            Units.radiansToDegrees(Math.atan2(x, y - ArmConstants.kJOINT_TO_FLOOR_DIST)),
            Math.sqrt(Math.pow(x, 2) + Math.pow(y - ArmConstants.kJOINT_TO_FLOOR_DIST, 2))
        );
    }

    /**
     * Sets the target position
     * <ul>
     * <li>x = 0 is straight up and down</li>
     * <li>y = 0 is floor</li>
     * </ul>
     * @param x target (inches)
     * @param y target (inches)
     */
    public void setTarget(double x, double y) {
        ArmState newPos = inverseKinematics(x, y);

        angleSP = newPos.angle;
        extSP = newPos.extension;
    }

    // Set Raw Speed as percentage (for testing mode, no sanity checks done)
    public void setRawSpeed(double speed) { winchMotor.set(speed); }

    // Set voltage of winch
    public void setWinchVolts(double volts) { winchMotor.setVoltage(volts); }

    // Reset relative encoder position (testing mode)
    public void setWinchEncPosition(double rots) { winchEnc.setPosition(rots); }

    // Set brake for extension
    public void setBrake(boolean brake) {
        winchMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    // Getters
    public double getAngleSetpoint() { return angleSP; }
    public double getAngleActual() { return angleAbsEncoder.getPosition(); }
    public double getAngleVoltage() { return angleMotor.getAppliedOutput(); }
    public double getWinchEncPos() { return winchEnc.getPosition(); }
    public double getWinchEncVelocity() { return winchEnc.getVelocity(); }
    public Translation2d getPose() { return pose; }
    public ArmState getState() { return currentState; }

    public double getAngleTemp() { return MathUtils.celsiusToFahrenheit(angleMotor.getMotorTemperature()); }
    public double getExtTemp() { return MathUtils.celsiusToFahrenheit(winchMotor.getMotorTemperature()); }
}
