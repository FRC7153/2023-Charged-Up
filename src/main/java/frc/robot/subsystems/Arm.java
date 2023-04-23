package frc.robot.subsystems;

import java.util.HashMap;

import com.frc7153.math.Encoder;
import com.frc7153.math.MathUtils;
import com.frc7153.math.ShuffleboardProfiledPIDController;
import com.frc7153.math.Encoder.Range;
import com.frc7153.validation.DeviceChecker;
import com.frc7153.validation.Validatable;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase implements Validatable {
    // Arm State
    public static class ArmState {
        public double angle;
        public double extension;

        /**
         * @param angle degrees, 0 upwards
         * @param extension inches (joint to grab point)
         */
        public ArmState(double angle, double extension) { this.angle = angle; this.extension = extension; }

        /**
         * @param angle degrees, 0 upwards
         * @param extension rotations
         */
        public static ArmState fromRots(double angle, double extension) { return new ArmState(angle, ArmConstants.winchRotsToTargetExt(extension)); }
    }

    // Motors
    private CANSparkMax angleMotor = new CANSparkMax(16, MotorType.kBrushless);
    private CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless);

    // PID
    private ShuffleboardProfiledPIDController anglePID = ArmConstants.kARM_PID.toWPIProfiledPidController(ArmConstants.kMAX_ANGLE_VELOCITY, ArmConstants.kMAX_ANGLE_ACCELERATION);
    private SparkMaxPIDController winchPID = winchMotor.getPIDController();

    private Double angleSP = Double.NaN;
    private Double extSP = Double.NaN;
    private Double extRef = Double.NaN; // The actual value passed to the PID controller, post-calculations
    private Double angRef = Double.NaN;
    private Translation2d pose = new Translation2d(0.0, 0.0);
    private ArmState currentState = new ArmState(0.0, 0.0);
    private double currentAngleVolts = 0.0;

    // Encoders
    private DutyCycleEncoder rawAngleAbsEncoder = new DutyCycleEncoder(8);
    private Encoder angleAbsEncoder = new Encoder(rawAngleAbsEncoder::getAbsolutePosition);
    private RelativeEncoder winchEnc = winchMotor.getEncoder();

    // State
    public boolean hasBeenReleased = false;

    GenericEntry tempFF = Shuffleboard.getTab("Arm test").add("kFF", -0.009).getEntry();

    // Init
    public Arm() {
        // Config Arm
        angleMotor.setInverted(true);
        angleMotor.setSecondaryCurrentLimit(150);
        angleMotor.setSmartCurrentLimit(100);

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
        //anglePID.refresh();

        if (!DriverStation.isDisabled() && !angleSP.isNaN() && !extSP.isNaN()) {
            // Verify winch motor is safe
            if (winchEnc.getPosition() < 0.0) {
                DriverStation.reportWarning(String.format("Winch is below zero! -> %s rotations", winchEnc.getPosition()), false);
            }

            if (testing) {
                // Set angle (only testing)
                anglePID.setGoal(angleSP);
            } else {
                // Constrain position
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

                // Set position safely
                extRef = Math.max(ArmConstants.extToWinchRots(currentState.extension - ArmConstants.kHAND_LENGTH), 0.0);
                angRef = currentState.angle;

                // Safety Check
                /*if (kinematics(ArmConstants.winchRotsToTargetExt(winchEnc.getPosition()), angleAbsEncoder.getPosition()).getY() <= 72.0 && Math.abs(angleSP) > 0.1) {
                    anglePID.setGoal(angRef);
                }*/

                if (Math.abs(angleSP) < 0.1) {
                //if (Math.abs(angleSP) < 0.1 || (angleAbsEncoder.getPosition() > 0.1 && angleSP < 0.1) || (angleAbsEncoder.getPosition() < -0.1 && angleSP > -0.1)) {
                    // Going to straight up, ANGLE is problem
                    boolean setAngle;

                    if (angleAbsEncoder.getPosition() > 0.0) {
                        // Forwards
                        // angleAbsEncoder.getPosition()
                        setAngle = kinematics(ArmConstants.winchRotsToTargetExt(winchEnc.getPosition()), angleAbsEncoder.getPosition()).getY() <= 50.0;
                    } else {
                        // Backwards
                        setAngle = kinematics(ArmConstants.winchRotsToTargetExt(winchEnc.getPosition()), angleAbsEncoder.getPosition()).getY() <= 60.0;
                    }

                    // Only set angle if safe
                    if (setAngle) {
                        anglePID.setGoal(angRef);
                    } else {
                        DriverStation.reportWarning("Arm SP above limit, angle not set!", false);
                    }

                    // Set ext
                    winchPID.setReference(extRef, ControlType.kPosition, ArmConstants.kEXT_PID.kSLOT);
                } else {
                    // Going to not straight up, EXTENSION is problem
                    boolean setExt = true;

                    // extRef
                    if (angleAbsEncoder.getPosition() > 0.0) {
                        // Forwards
                        setExt = kinematics(ArmConstants.winchRotsToTargetExt(winchEnc.getPosition()), angleAbsEncoder.getPosition()).getY() <= 58.0;
                    } else {
                        // Backwards
                        setExt = kinematics(ArmConstants.winchRotsToTargetExt(winchEnc.getPosition()), angleAbsEncoder.getPosition()).getY() <= 68.0;
                    }

                    // Only set extension if safe
                    if (setExt) {
                        winchPID.setReference(extRef, ControlType.kPosition, ArmConstants.kEXT_PID.kSLOT);
                    } else {
                        DriverStation.reportWarning("Arm SP above limit, extension not set!", false);
                    }

                    // Set angle
                    anglePID.setGoal(angRef);
                }

                /*if (angleAbsEncoder.getPosition() > 0.0) { // TODO dependent on actual side not SP
                    // Forwards
                    DriverStation.reportError(kinematics(ArmConstants.winchRotsToTargetExt(winchEnc.getPosition()), angleAbsEncoder.getPosition()).toString(), false);
                    DriverStation.reportError(angleSP.toString(), false);
                    if (Math.abs(angleSP) < 0.1 && kinematics(ArmConstants.winchRotsToTargetExt(winchEnc.getPosition()), angleAbsEncoder.getPosition()).getY() > 50.0) {
                        DriverStation.reportWarning("Arm SP above limit, angle not set!", false);
                    } else {
                        anglePID.setGoal(angRef);
                    }
                } else {
                    // Backwards
                    if (Math.abs(angleSP) < 0.1 && kinematics(ArmConstants.winchRotsToTargetExt(winchEnc.getPosition()), angleAbsEncoder.getPosition()).getY() > 60.0) {
                        DriverStation.reportWarning("Arm SP above limit, angle not set!", false);
                    } else {
                        anglePID.setGoal(angRef);
                    }
                }*/
            }

            // Set angle voltage
            if (Math.abs(angRef) <= 0.08 && Math.abs(angleAbsEncoder.getPosition()) <= 10.0) {
                // Stop the arm wiggling
                currentAngleVolts = 0.0;
                anglePID.reset(0.0);
            }  else {
                // Get voltage position
                currentAngleVolts = anglePID.calculate(angleAbsEncoder.getPosition());
                currentAngleVolts += (Math.sin(Units.degreesToRadians(angleAbsEncoder.getPosition())) * winchEnc.getPosition()) * -0.009; // Feed forward (0.0 if this causes problems)
            }
            angleMotor.setVoltage(currentAngleVolts);
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

    public boolean atSetpoint() {
        return (Math.abs(angleAbsEncoder.getPosition() - angRef) <= ArmConstants.kANGLE_TOLERANCE) && (Math.abs(winchEnc.getPosition() - extRef) <= ArmConstants.kWINCH_TOLERANCE);
    }

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

    public void setTarget(Translation2d pos) { setTarget(pos.getX(), pos.getY()); }

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
    public double getAngleSetpoint() { return angRef; }
    public double getAngleActual() { return angleAbsEncoder.getPosition(); }
    public double getAngleVoltage() { return currentAngleVolts; }
    public double getAngleCurrent() { return angleMotor.getOutputCurrent(); }
    public double getWinchEncPos() { return winchEnc.getPosition(); }
    public double getWinchSetpoint() { return extRef; }
    public double getWinchEncVelocity() { return winchEnc.getVelocity(); }
    public Translation2d getPose() { return pose; }
    public ArmState getState() { return currentState; }
    public double getAngleDutyCycle() { return angleMotor.getAppliedOutput(); }

    public double getAngleTemp() { return MathUtils.celsiusToFahrenheit(angleMotor.getMotorTemperature()); }
    public double getExtTemp() { return MathUtils.celsiusToFahrenheit(winchMotor.getMotorTemperature()); }

    // Validate
    private HashMap<String, Boolean> validationMap = new HashMap<>(4);

    @Override
    public HashMap<String, Boolean> validate() {
        validationMap.put("Angle NEO", DeviceChecker.validateMotor(angleMotor));
        validationMap.put("Angle Abs Enc", rawAngleAbsEncoder.isConnected());

        validationMap.put("Ext NEO", DeviceChecker.validateMotor(winchMotor));
        validationMap.put("Ext Enc", DeviceChecker.validateEncoder(winchMotor));

        return validationMap;
    }
}
