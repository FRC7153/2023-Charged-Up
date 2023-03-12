package frc.robot.subsystems;

import com.frc7153.logging.FileDump;
import com.frc7153.math.Encoder;
import com.frc7153.math.Encoder.Range;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    // Motors
    private CANSparkMax angleMotor = new CANSparkMax(16, MotorType.kBrushless);
    public CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless);

    // PID
    private PIDController anglePID = ArmConstants.kARM_PID.toWPIPidController();
    private SparkMaxPIDController winchPID = winchMotor.getPIDController();

    private double angleSP = 0.0;
    private double extSP = 0.0;
    private Translation2d pose = new Translation2d(0.0, 0.0);

    private FileDump debug = new FileDump("MovementConstraint");

    // Encoders
    private Encoder angleAbsEncoder = new Encoder((new DutyCycleEncoder(8))::getAbsolutePosition);
    public RelativeEncoder winchEnc = winchMotor.getEncoder();

    // Init
    public Arm() {
        // Config Arm
        angleMotor.setInverted(true);
        angleMotor.setSmartCurrentLimit(60);

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
        if (!DriverStation.isDisabled()) {
            // Verify winch motor is safe
            if (winchEnc.getPosition() < 0.0) {
                DriverStation.reportError("Winch is below zero!", false);
            }

            // Constrain and set position
            if (testing) {
                anglePID.setSetpoint(angleSP);
            } else {
                Translation2d pose = kinematics(extSP, angleSP);
                double x = pose.getX();
                double y = pose.getY();

                double MAX_X = ArmConstants.kMAX_REACH + ArmConstants.kJOINT_TO_BUMPER_DIST - ArmConstants.kCLEARANCE;
                double MAX_Y = ArmConstants.kMAX_HEIGHT - ArmConstants.kCLEARANCE;
                double MIN_Y = ArmConstants.kCLEARANCE;

                if (x > MAX_X ) 
                    x = MAX_X;
                else if (x < -MAX_X) 
                    x = -MAX_X;

                if (y > MAX_Y)
                    y = MAX_Y;
                else if (y < MIN_Y)
                    y = MIN_Y; 

                Translation2d newPos = inverseKinematics(x, y);

                anglePID.setSetpoint(newPos.getX());
                winchPID.setReference(ArmConstants.extToWinchRots(newPos.getY()), ControlType.kPosition, ArmConstants.kEXT_PID.kSLOT);

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
        return new Translation2d(
            Math.sin(Units.degreesToRadians(angle)) * ext,
            (Math.cos(Units.degreesToRadians(angle)) * ext) + ArmConstants.kJOINT_TO_FLOOR_DIST
        );
    }

    // Inverse kinematics
    // X is angle, y is extension
    private Translation2d inverseKinematics(double x, double y) {
        return new Translation2d(
            Math.sqrt(Math.pow(x, 2) + Math.pow(y - ArmConstants.kJOINT_TO_FLOOR_DIST, 2)),
            Units.radiansToDegrees(Math.atan2(x, y - ArmConstants.kJOINT_TO_FLOOR_DIST))
        );
    }

    /**
     * Verify a position is legal
     * @param pos The x and y coordinates of the arm, relative the the center of the floor
     * @return boolean
     */
    // public boolean sanityCheckPosition(Translation2d pos) {
    //     return (
    //         pos.getX() >= -ArmConstants.kMAX_REACH - ArmConstants.kJOINT_TO_BUMPER_DIST + ArmConstants.kCLEARANCE &&
    //         pos.getX() <= ArmConstants.kMAX_REACH + ArmConstants.kJOINT_TO_BUMPER_DIST  - ArmConstants.kCLEARANCE &&
    //         pos.getY() >= ArmConstants.kCLEARANCE &&
    //         pos.getY() <= ArmConstants.kMAX_HEIGHT - ArmConstants.kCLEARANCE
    //     );
    // }

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
        Translation2d newPos = inverseKinematics(x, y);

        angleSP = newPos.getX();
        extSP = newPos.getY();
    }

    // Getters
    public double getAngleSetpoint() { return angleSP; }
    public double getAngleActual() { return angleAbsEncoder.getPosition(); }
    public double getAngleVoltage() { return angleMotor.getAppliedOutput(); }
    public Translation2d getPose() { return pose; }
}
