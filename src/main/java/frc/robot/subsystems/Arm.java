package frc.robot.subsystems;

import com.frc7153.controllers.AbsoluteDutyCycleEncoder;
import com.frc7153.controllers.AbsoluteDutyCycleEncoder.Range;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    // Motors
    private CANSparkMax angleMotor = new CANSparkMax(16, MotorType.kBrushless);
    private CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless);

    // PID
    //private SparkMaxPIDController anglePID = angleMotor.getPIDController();
    //private PIDController anglePID = new PIDController(0.0084, 1e-6, 0.0);
    //private PIDController anglePID = new PIDController(0.09, 0.01, 0.0); // previous
    private ProfiledPIDController anglePID = new ProfiledPIDController(0.09, 0.01, 0.0, new Constraints(0.1, 0.1));
    private SparkMaxPIDController winchPID = winchMotor.getPIDController();

    private double angleSP = 0.0;
    private double extSP = 0.0;

    // Encoders
    private AbsoluteDutyCycleEncoder angleAbsEncoder = new AbsoluteDutyCycleEncoder(8);

    // Init
    public Arm() {
        // Config Arm
        angleMotor.setInverted(true);

        angleAbsEncoder.setConversionFactor(360.0);
        angleAbsEncoder.setInverted(false);
        angleAbsEncoder.setZeroOffset(0.0);
        angleAbsEncoder.setRange(Range.FROM_NEGATIVE_180_TO_180);

        //ArmConstants.kARM_PID.apply(anglePID);

        //angleMotor.setIdleMode(IdleMode.kBrake);

        // Verify values have been set
        /*System.out.println(String.format(
            "Angle PID coefficients -> %s, %s, %s",
            anglePID.getP(ArmConstants.kARM_PID.kSLOT),
            anglePID.getI(ArmConstants.kARM_PID.kSLOT),
            anglePID.getD(ArmConstants.kARM_PID.kSLOT)
        ));*/

        // Config Winch
        ArmConstants.kEXT_PID.apply(winchPID);
    }

    // Go to setpoint
    @Override
    public void periodic() {
        // TODO max min
        if (!DriverStation.isDisabled()) {
            angleMotor.setVoltage(
                //MathUtils.symmetricClamp(anglePID.calculate(angleAbsEncoder.getAbsolutePosition()), 0.5)
                anglePID.calculate(angleAbsEncoder.getAbsolutePosition())
            );
        }
    }

    /**
     * Set angle
     * @angle angle, in degrees
     * @return whether this position can be legally obtained (not outside max extension)
     */
    public boolean setAngle(double angle) {
        //if (!sanityCheckPosition(kinematics(extSP, angle))) { return false; }

        angleSP = angle;
        //anglePID.setReference(angle / 360.0 * ArmConstants.kANGLE_RATIO, ControlType.kPosition, ArmConstants.kARM_PID.kSLOT);
        //anglePID.setSetpoint(angle);
        anglePID.setGoal(angle);
        return true;
    }
    
    /**
     * Set the extension
     * @param ext Total extension, joint to grab point
     * @return Whether this position is legal
     */
    public boolean setExtension(double ext) {
        if (!sanityCheckPosition(kinematics(ext, angleSP))) { return false; }

        extSP = ext - ArmConstants.kMIN_EXTENSION;
        winchPID.setReference(extSP, ControlType.kPosition, ArmConstants.kEXT_PID.kSLOT);
        return true;
    }

    /**
     * Gets the position of the arm (forward kinematics)
     * @param ext Extension of arm, in inches (joint to grab point)
     * @param angle Angle of arm, in degrees from zero
     * @return Arm position
     */
    public Translation2d kinematics(double ext, double angle) {
        return new Translation2d(
            Math.cos(Units.degreesToRadians(angle)) * ext,
            (Math.sin(Units.degreesToRadians(angle)) * ext) + ArmConstants.kJOINT_TO_FLOOR_DIST
        );
    }

    /**
     * Verify a position is legal
     * @param pos The x and y coordinates of the arm, relative the the center of the floor
     * @return boolean
     */
    public boolean sanityCheckPosition(Translation2d pos) {
        return (
            pos.getX() >= -ArmConstants.kMAX_ARM_ANGLE - ArmConstants.kJOINT_TO_BUMPER_DIST &&
            pos.getX() <= ArmConstants.kMAX_REACH + ArmConstants.kJOINT_TO_BUMPER_DIST &&
            pos.getY() >= 0 &&
            pos.getY() <= ArmConstants.kMAX_HEIGHT
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
     * @return whether the specified position is possible (legally and physically)
     */
    public boolean setTarget(double x, double y) {
        // Sanity check
        if (!sanityCheckPosition(new Translation2d(x, y))) {
            DriverStation.reportWarning(String.format("Illegal target arm position (%s, %s)", x, y), false);
            return false;
        }

        // Get extension and angle (INVERSE kinematics)
        double extension = Math.sqrt(Math.pow(x, 2) + Math.pow(y - ArmConstants.kJOINT_TO_FLOOR_DIST, 2));
        double angle = Units.radiansToDegrees(Math.atan((y - ArmConstants.kJOINT_TO_FLOOR_DIST) / x));

        extension -= ArmConstants.kMIN_EXTENSION;

        // Sanity check 2
        if (
            extension < 0.0 || 
            extension > ArmConstants.kMAX_EXTENSION ||
            angle > ArmConstants.kMAX_ARM_ANGLE ||
            angle < -ArmConstants.kMAX_ARM_ANGLE
        ) {
            DriverStation.reportWarning(String.format("Illegal arm kinematic position (%s, %s -> %s and %s deg)", x, y, extension, angle), false);
            return false;
        }

        // Move to motor positions (non short-circuit evaluation)
        if (!(setAngle(angle) & setExtension(extension))) {
            DriverStation.reportWarning(String.format("Could not set arm and angle pose (%s, %s -> %s and %s deg), final sanity check failed but the arm may already be in motion!", x, y, extension, angle), false);
            return false;
        }

        return true;
    }

    // Getters
    public double getAngleSetpoint() { return angleSP; }
    public double getAngleActual() { return angleAbsEncoder.getAbsolutePosition(); }
    public double getAngleVoltage() { return angleMotor.getAppliedOutput(); }
}
