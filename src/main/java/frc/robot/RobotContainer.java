package frc.robot;

import com.frc7153.commands.ConfigCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.OI.Controller0;
import frc.robot.OI.Controller1;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.commands.PresetArmCommand;
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
    public final SwerveDriveBase driveBase = new SwerveDriveBase();
    public final Arm arm = new Arm();
    public final Claw claw = new Claw();

    public final ShuffleboardManager shuffleboard;

    // Constructor
    public RobotContainer() {
        // Create command bindings
        configureBindings();

        // Start Shuffleboard
        shuffleboard = new ShuffleboardManager(Controller0.controller, armPi, driveBase.imu, arm, claw);
    }

    // Configure Command Bindings
    private void configureBindings() {
        // Default Teleop Commands
        driveBase.setDefaultCommand(new TeleopDriveCommand(
            driveBase,
            Controller0::getLeftX,
            Controller0::getLeftY,
            Controller0::getRightX
        ));

        arm.setDefaultCommand(new TeleopArmCommand(arm, claw, Controller1::getY, Controller1::getThrottle));
        claw.setDefaultCommand(new GrabCommand(claw, 0.47, 0.81));

        // Arm to Preset
        Controller1.button11.whileTrue(new PresetArmCommand(arm, 119.09, 0.90));
        Controller1.button12.whileTrue(new PresetArmCommand(arm, -114.6, 0.0));

        // Claw
        Controller1.trigger.whileTrue(new GrabCommand(claw, 0.26, 0.98));

        // Stow Position
        Controller1.button2.whileTrue(new ParallelCommandGroup(
            new PresetArmCommand(arm, 34.0, 1.0),
            new GrabCommand(claw, 0.89, 0.39)
        ));

        // Testing Commands
        if (Constants.kTEST_DEPLOY) {
            //Controller1.button5.onTrue(new ConfigCommand(claw::resetEncoders, "", claw));
        }

        // Auto Move Arm
        //Controller0.lBumper.and(Controller0.lTrigger.negate()).onTrue(new PremoveClawCommand(arm, claw));
        //Controller0.lTrigger.onTrue(new GrabCommand());
    }

    // Get Auto Command
    public Command getAutonomousCommand() {
        return null;
        /*
        Trajectory loaded = new Trajectory();
        try {
            Path traj = Filesystem.getDeployDirectory().toPath().resolve("paths/straightTest2.wpilib.json");
            loaded = TrajectoryUtil.fromPathweaverJson(traj);
        } catch (IOException e) {
            return null;
        }

        return new SwerveTrajectoryFollow(
            new RamseteController(2.1, 0.8), 
            loaded, 
            driveBase.base,
            () -> driveBase.getPose()
        );*/
    }
}
