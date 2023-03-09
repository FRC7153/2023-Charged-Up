package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

/**
 * Command to home the claw, moving it to 0 extensions straight up and down
 * with the hands facing upward.
 */
public class TeleopClawCommand extends CommandBase {
    // Subsystems
    private Arm arm;
    private Claw claw;
    private Supplier<Double> angleSupplier;
    private Supplier<Double> extensionSupplier;

    // Constructor
    public TeleopClawCommand(Arm armSubsys, Claw clawSubsys, Supplier<Double> angleSupp, Supplier<Double> extSupplier) {
        claw = clawSubsys;
        arm = armSubsys;
        angleSupplier = angleSupp;
        extensionSupplier = extSupplier;
    
        addRequirements(claw, arm);
    }

    // Setup
    @Override
    public void initialize() {
        //arm.setTarget(0, ArmConstants.kJOINT_TO_FLOOR_DIST + 2.0);
        //claw.setSymmetricPosition(0.0);
        //arm.setAngle(0.0);
    }

    @Override
    public void execute() {
        arm.setAngle(angleSupplier.get()*90.0);
        arm.setExtension(((extensionSupplier.get() + 1.0) / 2.0) * 100.0);
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
