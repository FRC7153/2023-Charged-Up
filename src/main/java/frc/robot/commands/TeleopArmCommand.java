package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

/**
 * Command to home the claw, moving it to 0 extensions straight up and down
 * with the hands facing upward.
 */
public class TeleopArmCommand extends CommandBase {
    // Subsystems
    private Arm arm;
    private Supplier<Double> angleSupplier;
    private Supplier<Double> extensionSupplier;

    // Constructor
    public TeleopArmCommand(Arm armSubsys, Supplier<Double> angleSupp, Supplier<Double> extSupplier) {
        arm = armSubsys;
        angleSupplier = angleSupp;
        extensionSupplier = extSupplier;
    
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setAngle(-angleSupplier.get() * ArmConstants.kMAX_ANGLE);
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
