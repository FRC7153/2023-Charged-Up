package frc.robot.commands;

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

    // Constructor
    public HomeClawCommand(Arm armSubsys, Claw clawSubsys) {
        claw = clawSubsys;
        arm = armSubsys;
    
        addRequirements(claw, arm);    
    }

    // Setup
    @Override
    public void initialize() {
        arm.setTarget(0, ArmConstants.kJOINT_TO_FLOOR_DIST + 2.0);
        claw.setSymmetricPosition(0.0);
    }

    // Never finished
    @Override
    public boolean isFinished() { return false; }
}
