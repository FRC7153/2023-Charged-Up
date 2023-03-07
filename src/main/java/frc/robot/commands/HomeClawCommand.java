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
public class HomeClawCommand extends CommandBase {
    // Subsystems
    private Arm arm;
    private Claw claw;
    private Supplier<Double> posSupplier; // TODO temp

    // Constructor
    public HomeClawCommand(Arm armSubsys, Claw clawSubsys, Supplier<Double> supp) {
        claw = clawSubsys;
        arm = armSubsys;
        posSupplier = supp;
    
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
        arm.setAngle(posSupplier.get() * 90.0);
    }

    // Never finished
    @Override
    public boolean isFinished() { return false; }
}
