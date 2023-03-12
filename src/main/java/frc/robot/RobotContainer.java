package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.OI.Controller0;
import frc.robot.OI.Controller1;
import frc.robot.commandgroups.TestCommand;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.commands.PresetArmCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.peripherals.ArmPI;
import frc.robot.peripherals.ShuffleboardManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

public class RobotContainer {
    // Peripherals
    private final ArmPI armPi = new ArmPI();

    // Subsystems
    private final DriveBase driveBase = new DriveBase();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();

    // Shuffleboard
    private final ShuffleboardManager shuffleboard;

    // Constructor
    public RobotContainer() {
        // Create command bindings
        configureBindings();

        // Start Shuffleboard
        shuffleboard = new ShuffleboardManager(armPi, driveBase.imu, arm, claw);
    }

    // Configure Command Bindings
    private void configureBindings() {
        // Teleop Drive Command
        driveBase.setDefaultCommand(new TeleopDriveCommand(
            driveBase,
            Controller0::getLeftX,
            Controller0::getLeftY,
            Controller0::getRightX
        ));

        // Teleop Arm Command (position setpoint)
        arm.setDefaultCommand(new TeleopArmCommand(arm, Controller1::getY, Controller1::getThrottle));

        // Default Claw Command (open position)
        claw.setDefaultCommand(new GrabCommand(claw, 0.47, 0.81));

        // TODO Arm to Preset Location Commands
        Controller1.button11.whileTrue(new PresetArmCommand(arm, 119.09, 0.90));
        Controller1.button12.whileTrue(new PresetArmCommand(arm, -114.6, 0.0));

        // Claw Grab Command
        Controller1.trigger.whileTrue(new GrabCommand(claw, 0.26, 0.98));

        // Stow Position (arm 34 degrees, claw stowed)
        Controller1.button2.whileTrue(new ParallelCommandGroup(
            new PresetArmCommand(arm, 34.0, 0.0),
            new GrabCommand(claw, 0.89, 0.39)
        ));
    }

    // Toggle Brakes (in drive and claw)
    public void toggleBrakes(boolean brake) {
        driveBase.setCoast(!brake);
        claw.setCoastMode(!brake);
    }

    // Get Auto Command
    public Command getAutonomousCommand() {
        return null;
    }

    // Get Testing Command
    public Command getTestingCommand() {
        return new TestCommand(arm, shuffleboard, Controller1::getThrottle);
    }
}
