package frc.robot.autos;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.peripherals.ShuffleboardManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

/**
 * Command that runs in testing mode to home arm extension.
 * This isn't technically a command group, but it is here because
 * it runs the entire robot during a specific mode (testing)
 */
public class TestCommand extends CommandBase {
    // Subsystems
    private Arm arm;
    private ShuffleboardManager shuffleboard;

    // Suppliers
    private Supplier<Double> extSupp;

    // Constructor
    public TestCommand(Arm armSubsys, ShuffleboardManager shuffleboardManager, Supplier<Double> extSupplier) {
        arm = armSubsys;
        shuffleboard = shuffleboardManager;
        extSupp = extSupplier;
    }

    // Init
    @Override
    public void initialize() {
        arm.setAngle(0.0);
        //claw.setPosition(GrabPositions.RELEASE);
    }

    // Periodic
    @Override
    public void execute() {
        arm.setRawSpeed(extSupp.get());
        arm.setWinchEncPosition(0.0);

        arm.periodic(true);
        shuffleboard.periodic();
    }

    // Stop movement
    @Override
    public void end(boolean terminated) {
        arm.setRawSpeed(0.0);
    }

    // Does not finish
    @Override
    public boolean isFinished() { return false; }
}
