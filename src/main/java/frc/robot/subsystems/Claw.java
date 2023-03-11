package frc.robot.subsystems;

import com.frc7153.math.Encoder;
import com.frc7153.math.MathUtils;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

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
    private AbsoluteEncoder lHandEnc = lHand.getAbsoluteEncoder(Type.kDutyCycle);
    private AbsoluteEncoder rHandEnc = rHand.getAbsoluteEncoder(Type.kDutyCycle);


    // Encoders
    //private SparkMaxAbsoluteEncoder lHandEncoder = lHand.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    //private SparkMaxAbsoluteEncoder rHandEncoder = rHand.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    // Constructor
    public Claw() {
        // Config encoders
        lHandEnc.setInverted(true);

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

    // Checks if value is possible
    private boolean isPossible(double value) {
        return (value <= ClawConstants.kMAX_ANGLE && value >= ClawConstants.kMIN_ANGLE);
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

        if (isPossible(lAngle)) {
            lHandPid.setReference(MathUtils.wrap0To1((lAngle/360.0) - 0.25), ControlType.kPosition, ClawConstants.kHAND_PID.kSLOT);
        }
        if (isPossible(rAngle)) {
            rHandPid.setReference(MathUtils.wrap0To1((rAngle/360.0) - 0.11), ControlType.kPosition, ClawConstants.kHAND_PID.kSLOT);
        }

        return (isPossible(lAngle) && isPossible(rAngle));
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
}
