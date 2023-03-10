package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    // Motors
    private CANSparkMax lHand = new CANSparkMax(18, MotorType.kBrushless);
    private CANSparkMax rHand = new CANSparkMax(17, MotorType.kBrushless);

    // PID loops
    private SparkMaxPIDController lHandPid = lHand.getPIDController();
    private SparkMaxPIDController rHandPid = rHand.getPIDController();

    // Encoders
    private RelativeEncoder lHandEnc = lHand.getEncoder();
    private RelativeEncoder rHandEnc = rHand.getEncoder();

    // Encoders
    //private SparkMaxAbsoluteEncoder lHandEncoder = lHand.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    //private SparkMaxAbsoluteEncoder rHandEncoder = rHand.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    // Constructor
    public Claw() {
        // Config encoders
        //lHandEncoder.setZeroOffset(ClawConstants.kLHAND_OFFSET);
        //rHandEncoder.setZeroOffset(ClawConstants.kRHAND_OFFSET);

        //lHandEncoder.setPositionConversionFactor(ClawConstants.kANGLE_RATIO);
        //rHandEncoder.setPositionConversionFactor(ClawConstants.kANGLE_RATIO);

        // Config PID
        lHandPid.setFeedbackDevice(lHandEnc);
        rHandPid.setFeedbackDevice(rHandEnc);

        // Config motors
        lHand.setInverted(false);
        rHand.setInverted(true);

        setCoastMode(false);

        // Config PID
        ClawConstants.kHAND_PID.apply(lHandPid);
        ClawConstants.kHAND_PID.apply(rHandPid);
    }

    // Set Coast/Brake Mode
    public void setCoastMode(boolean coast) {
        lHand.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        rHand.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
    }

    // Checks if value is possible, and applies it if so
    private boolean applyIfPossible(SparkMaxPIDController pid, double value) {
        
        if (value <= ClawConstants.kMAX_ANGLE && value >= ClawConstants.kMIN_ANGLE) {
            pid.setReference(value / 360.0 * ClawConstants.kANGLE_RATIO, ControlType.kPosition, ClawConstants.kHAND_PID.kSLOT);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets the position of each individual hand.
     * Also disabled coast mode<br><br>
     * 0 degrees is downward, positive is outwards for both
     * @param lAngle Angle of left hand
     * @param rAngle Angle of right hand
     * @return True, if this is a possible location
     */
    public boolean setPosition(double lAngle, double rAngle) {
        setCoastMode(false);
        return (applyIfPossible(lHandPid, lAngle) & applyIfPossible(rHandPid, rAngle));
    }

    /**
     * Sets the position of both claws to the same
     * @param angle The angle (0 is forward, negative is outward)
     * @return True, if this is a possible location
     */
    public boolean setSymmetricPosition(double angle) { return setPosition(angle, angle); }

    // Angle getters
    public double getLHandPos() { return lHandEnc.getPosition(); }
    public double getRHandPos() { return rHandEnc.getPosition(); }

    // Reset Encoders
    public void resetEncoders() { lHandEnc.setPosition(0.0); rHandEnc.setPosition(0.0); }
}
