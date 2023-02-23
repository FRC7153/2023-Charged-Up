package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ArmConstants;

public class Arm {
    // Motors
    private CANSparkMax armMotor = new CANSparkMax(16, MotorType.kBrushless);
    private CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless);

    // PID
    private SparkMaxPIDController armPID = armMotor.getPIDController();
    private SparkMaxPIDController winchPID = winchMotor.getPIDController();

    // Encoders
    private SparkMaxAbsoluteEncoder angleEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    // Init
    public Arm() {
        // Config Arm
        angleEncoder.setZeroOffset(ArmConstants.kANGLE_0_ANGLE);
        angleEncoder.setPositionConversionFactor(ArmConstants.kANGLE_RATIO);

        ArmConstants.kANGLE_PID.apply(armPID);
        armPID.setFeedbackDevice(angleEncoder);

        // Config Winch
        ArmConstants.kEXT_PID.apply(winchPID);
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
        if (
            x < -ArmConstants.kMAX_REACH - ArmConstants.kJOINT_TO_BUMPER_DIST || 
            x > ArmConstants.kMAX_REACH + ArmConstants.kJOINT_TO_BUMPER_DIST ||
            y < 0 ||
            y > ArmConstants.kMAX_HEIGHT
        ) {
            DriverStation.reportWarning(String.format("Illegal target arm position (%s, %s)", x, y), false);
            return false;
        }

        // Get extension and angle
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

        // Move to motor positions
        // TODO
        armPID.setReference(angle - ArmConstants.kANGLE_0_ANGLE, ControlType.kPosition, ArmConstants.kANGLE_PID.kSLOT);

        return true;
    }
}
