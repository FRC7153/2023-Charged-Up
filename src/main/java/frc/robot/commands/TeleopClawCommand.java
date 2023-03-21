package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GrabPositions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class TeleopClawCommand extends CommandBase {
    // Subsystems + Supplier
    private Arm arm;
    private Claw claw;
    private Supplier<Boolean> triggerSupp;

    // Button Press
    private boolean grabbing;
    private double debounce;

    // Init
    public TeleopClawCommand(Arm armSubsys, Claw clawSubsys, Supplier<Boolean> triggerSupplier) {
        arm = armSubsys;
        claw = clawSubsys;
        triggerSupp = triggerSupplier;

        addRequirements(claw);
    }

    @Override
    public void initialize() {        
        grabbing = true;
        debounce = 0.0;

        claw.setPosition(GrabPositions.GRAB);
    }

    // Execute
    @Override
    public void execute() {
        if (!arm.hasBeenReleased) { return; }

        if (triggerSupp.get() && Timer.getFPGATimestamp() - debounce >= 0.8) {
            debounce = Timer.getFPGATimestamp();
            grabbing = !grabbing;

            if (grabbing) { // Grab
                claw.setPosition(GrabPositions.GRAB);
            } else { // Release
                claw.setPosition(GrabPositions.RELEASE);
            }
        }
    }

    // End
    @Override
    public void end(boolean terminated) {
        // Release
        claw.setPosition(GrabPositions.RELEASE);
    }
}
