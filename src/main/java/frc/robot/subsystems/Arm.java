package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Arm {
    // Motors
    private CANSparkMax armMotor = new CANSparkMax(16, MotorType.kBrushless);
    private CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless);

    // PID
    private SparkMaxPIDController armPID = armMotor.getPIDController();
    private SparkMaxPIDController winchPID = winchMotor.getPIDController();
    

    // Init
    public Arm() {
        // TODO pid values
    }

    /**
     * Sets the target position
     * <ul>
     * <li>x = 0 is straight up and down</li>
     * <li>y = 0 is floor</li>
     * </ul>
     * @param x target (meters)
     * @param y target (meters)
     * @return whether the specified position is possible (legally and physically)
     */
    public boolean setTarget(double x, double y) {
        // Sanity check
        if (
            x < -48.0 - Constants.kJOINT_TO_BUMPER_DIST || 
            x > 48.0 + Constants.kJOINT_TO_BUMPER_DIST ||
            y < 0 ||
            y > 72.0
        ) {
            DriverStation.reportWarning(String.format("Illegal target arm position (%s, %s)", x, y), false);
            return false;
        }

        // Get extension and angle
        double extension = Math.sqrt(Math.pow(x, 2) + Math.pow(y - Constants.kJOINT_TO_FLOOR_DIST, 2));
        double angle = Units.radiansToDegrees(Math.atan((y - Constants.kJOINT_TO_FLOOR_DIST) / x));

        extension -= Constants.kARM_MIN_EXTENSION;

        // Sanity check 2
        if (
            extension < 0.0 || 
            extension > Constants.kARM_MAX_EXTENSION ||
            angle > Constants.kMAX_ARM_ANGLE ||
            angle < -Constants.kMAX_ARM_ANGLE
        ) {
            DriverStation.reportWarning(String.format("Illegal arm kinematic position (%s, %s -> %s and %s deg)", x, y, extension, angle), false);
            return false;
        }

        // Move to motor positions
        // TODO
        armPID.setReference(angle - Constants.kARM_0_ANGLE, ControlType.kPosition);
        

        return true;
    }
}
