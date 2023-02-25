package frc.robot;

import com.frc7153.inputs.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.peripherals.ArmPI;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ShuffleboardManager;
import frc.robot.subsystems.SwerveDriveBase;

public class RobotContainer {
    // Joysticks
    private final XboxController controller0 = new XboxController(0);

    // Peripherals
    private final ArmPI armPi = new ArmPI();

    // Subsystems
    private final SwerveDriveBase driveBase = new SwerveDriveBase();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();

    // Constructor
    public RobotContainer() {
        // Create command bindings
        configureBindings();

        // Start Shuffleboard
        new ShuffleboardManager(controller0, armPi, driveBase.imu);
    }

    // Configure Command Bindings
    private void configureBindings() {
        driveBase.setDefaultCommand(new TeleopDriveCommand(
            driveBase,
            () -> controller0.getLeftX(),
            () -> controller0.getLeftY(),
            () -> controller0.getRightX()
        ));
    }

    // Get Auto Command
    public Command getAutonomousCommand() {
        return null;
    }
}
