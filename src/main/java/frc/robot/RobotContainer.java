package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GameState;
import frc.robot.Constants.GrabPositions;
import frc.robot.OI.Controller0;
import frc.robot.OI.Controller1;
import frc.robot.commandgroups.TestCommand;
import frc.robot.commands.TeleopClawCommand;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.PresetArmCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.UnlockClawCommand;
import frc.robot.peripherals.ArmPI;
import frc.robot.peripherals.Limelight;
import frc.robot.peripherals.ShuffleboardManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

public class RobotContainer {
    // State
    public GameState state = GameState.DISABLED;

    // Peripherals
    private final ArmPI armPi = new ArmPI();
    private final Limelight frontLL = new Limelight("front");
    private final Limelight rearLL = new Limelight("back");

    // Subsystems
    private final DriveBase driveBase = new DriveBase();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();

    // Autonomous
    private Autonomous auto = new Autonomous(driveBase, arm, claw);

    // Shuffleboard + Commands
    private final ShuffleboardManager shuffleboard;
    public final Command unlockClawCommand = new UnlockClawCommand(claw, arm);

    // Constructor
    public RobotContainer() {
        // Create command bindings
        configureBindings();

        // Start Shuffleboard
        shuffleboard = new ShuffleboardManager(this, armPi, arm, claw, driveBase);
    }

    // Configure Command Bindings
    private void configureBindings() {
        // Teleop Drive Command
        driveBase.setDefaultCommand(new TeleopDriveCommand(
            driveBase,
            Controller0::getLeftX,
            Controller0::getLeftY,
            Controller0::getRightX,
            this::getGameState
        ));

        // Teleop Arm Command (position setpoint)
        arm.setDefaultCommand(new TeleopArmCommand(
            arm, 
            Controller1::getY, 
            Controller1::getThrottle, 
            this::getGameState,
            60.0
        ));

        // Default Claw Command (open position)
        claw.setDefaultCommand(new TeleopClawCommand(arm, claw, Controller0::getRightTrigger, this::getGameState));

        // Force wide release
        //Controller1.trigger.whileTrue(new GrabCommand(claw, GrabPositions.WIDE_RELEASE));

        // Arm Preset Positions
        Controller1.button7.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kFRONT_CONE_HIGH, true));
        Controller1.button8.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kFRONT_CUBE_HIGH, true));
        Controller1.button9.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kFRONT_CONE_MID, true));
        Controller1.button10.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kFRONT_CUBE_MID, true));
        Controller1.button11.whileTrue(new PresetArmCommand(arm, ArmPositions.kFRONT_GROUND));
        Controller1.button12.whileTrue(new PresetArmCommand(arm, ArmPositions.kREAR_GROUND));

        Controller1.trigger.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kREAR_LOADING_STATION, true));

        // Auto Balance
        Controller0.aButton.whileTrue(new BalanceCommand(driveBase));
        //Controller0.aButton.whileTrue(unlockClawCommand);

        // Stow Position (arm 34 degrees, claw stowed)
        /*Controller1.button2.whileTrue(new ParallelCommandGroup(
            new PresetArmCommand(arm, 34.0, 0.0),
            new GrabCommand(claw, GrabPos.STOW)
        ));*/
    }

    // Toggle Brakes (in drive and claw)
    public void toggleBrakes(boolean brake) {
        driveBase.setCoast(!brake);
        claw.setCoastMode(!brake);
        arm.setBrake(brake);
    }

    // Turn off limelights LED
    public void setLimelightLED(boolean state) {
        frontLL.forceLEDMode(state);
        rearLL.forceLEDMode(state);
    }

    // Run Shuffleboard (even when disabled)
    public void shuffleboardUpdate() { shuffleboard.periodic(); }

    // Get Auto Command
    public Command getAutonomousCommand() {
        return auto.getTestSpinAuto();
    }

    // Get Testing Command
    public Command getTestingCommand() {
        return new TestCommand(arm, claw, shuffleboard, Controller1::getThrottle);
    }

    // Check if arms are locked
    public boolean checkHandsLocked() { return !arm.hasBeenReleased; }

    // Game state supplier
    public GameState getGameState() { return state; }
}
