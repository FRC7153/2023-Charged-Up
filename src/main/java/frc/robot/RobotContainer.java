package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI.Controller0;
import frc.robot.commands.HomeClawCommand;
import frc.robot.commands.PremoveClawCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.peripherals.ArmPI;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ShuffleboardManager;
import frc.robot.subsystems.SwerveDriveBase;

public class RobotContainer {
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
        new ShuffleboardManager(Controller0.controller, armPi, driveBase.imu);
    }

    // Configure Command Bindings
    private void configureBindings() {
        // Default Teleop Commands
        driveBase.setDefaultCommand(new TeleopDriveCommand(
            driveBase,
            () -> Controller0.getLeftX(),
            () -> Controller0.getLeftY(),
            () -> Controller0.getRightX()
        ));

        arm.setDefaultCommand(new HomeClawCommand(arm, claw));

        // Auto Move Arm
        Controller0.lBumper.and(Controller0.lTrigger.negate()).onTrue(new PremoveClawCommand(arm, claw));
        Controller0.lTrigger.onTrue(new GrabCom)
    }

    // Get Auto Command
    public Command getAutonomousCommand() {
        return null;
    }
}
