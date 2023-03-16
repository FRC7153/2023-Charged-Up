package frc.robot.commands;

import java.util.function.Supplier;

import com.frc7153.math.MathUtils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameState;
import frc.robot.subsystems.Arm;

/**
 * Command to home the claw, moving it to 0 extensions straight up and down
 * with the hands facing upward.
 */
public class TeleopArmCommand extends CommandBase {
    // Subsystems
    private Arm arm;

    // Suppliers
    private Supplier<Double> angleSupplier;
    private Supplier<Double> extensionSupplier;
    private Supplier<GameState> stateSupply;

    // Velocity Integration
    private Double lastAngleIntegration;
    private Double currentAngle;
    private double maxAngleVelocity;

    // Constructor
    public TeleopArmCommand(Arm armSubsys, Supplier<Double> angleSupp, Supplier<Double> extSupplier, Supplier<GameState> stateSupplier, double maxAngleVelocity) {
        arm = armSubsys;

        angleSupplier = angleSupp;
        extensionSupplier = extSupplier;
        stateSupply = stateSupplier;

        this.maxAngleVelocity = maxAngleVelocity;
    
        addRequirements(arm);
    }

    // Init
    @Override
    public void initialize() {
        if (!stateSupply.get().equals(GameState.TELEOP)) { cancel(); return; }
        
        lastAngleIntegration = Timer.getFPGATimestamp();
        currentAngle = 0.0;
    }

    @Override
    public void execute() {
        if (ArmConstants.kUSE_POSITION_NOT_VELOCITY) {
            arm.setAngle(-angleSupplier.get() * ArmConstants.kMAX_ANGLE);
        } else {
            currentAngle += (-angleSupplier.get() * maxAngleVelocity * (Timer.getFPGATimestamp() - lastAngleIntegration));
            currentAngle = MathUtils.symmetricClamp(currentAngle, ArmConstants.kMAX_ANGLE);
            lastAngleIntegration = Timer.getFPGATimestamp();
            arm.setAngle(currentAngle);
        }

        arm.setExtension(((extensionSupplier.get() + 1.0) / 2.0 * ArmConstants.kWINCH_MAX_POSITION) + ArmConstants.kJOINT_TO_EXT_PT);
    }

    // Give priority to ALL other commands
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }

    // Never finished
    @Override
    public boolean isFinished() { return false; }
}