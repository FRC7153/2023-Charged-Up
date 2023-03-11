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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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
        lHandEnc.setInverted(false);
        rHandEnc.setInverted(false);

        lHandEnc.setZeroOffset(ClawConstants.kL_HAND_OFFSET);
        rHandEnc.setZeroOffset(ClawConstants.kR_HAND_OFFSET);

        // Config PID
        lHandPid.setFeedbackDevice(lHandEnc);
        rHandPid.setFeedbackDevice(rHandEnc);

        // Config motors
        lHand.setInverted(true);
        rHand.setInverted(true);

        lHand.setSmartCurrentLimit(ClawConstants.kCURRENT_LIMIT);
        rHand.setSmartCurrentLimit(ClawConstants.kCURRENT_LIMIT);

        setCoastMode(false);

        // Config PID
        ClawConstants.kHAND_PID.apply(lHandPid);
        ClawConstants.kHAND_PID.apply(rHandPid);

        lHandPid.setPositionPIDWrappingEnabled(false);
        rHandPid.setPositionPIDWrappingEnabled(false);
    }

    // Set Coast/Brake Mode
    public void setCoastMode(boolean coast) {
        lHand.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        rHand.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
    }

    /**
     * Sets the position of each individual hand in rotations (0 - 1).
     * Also disabled coast mode<br><br>
     * 0 degrees is forward, positive is outwards for both
     * @param lAngle Angle of left hand
     * @param rAngle Angle of right hand
     * @return True, if this is a possible location
     */
    public void setPosition(double lAngle, double rAngle) {
        setCoastMode(false);

        lHandPid.setReference(MathUtils.wrap0To1(lAngle), ControlType.kPosition, ClawConstants.kHAND_PID.kSLOT);
        rHandPid.setReference(MathUtils.wrap0To1(rAngle), ControlType.kPosition, ClawConstants.kHAND_PID.kSLOT);
    }

    // Angle getters
    public double getLHandPos() { return lHandEnc.getPosition(); }
    public double getRHandPos() { return rHandEnc.getPosition(); }
}
