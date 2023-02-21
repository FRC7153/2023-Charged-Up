package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;

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
     * @param target x and y position (meters)
     * @return whether the specified position is possible (legally and physically)
     */
    public boolean setTarget(Pose2d target) {
        return false;
    }
}
