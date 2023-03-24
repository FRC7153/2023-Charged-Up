package frc.robot.autos;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GrabPositions;
import frc.robot.commands.UnlockClawCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

/**
 * Manages auto events and commands
 */
public class Autonomous {
    // Subsystems
    private DriveBase drive;
    private Arm arm;
    private Claw claw;

    // Chooser
    private SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

    // Event Map
    public HashMap<String, Command> autoEventMap = new HashMap<>();

    // Constructor
    public Autonomous(DriveBase driveSubsys, Arm armSubsys, Claw clawSubsys) {
        // Save subsystems
        drive = driveSubsys;
        arm = armSubsys;
        claw = clawSubsys;

        // Create Auto Chooser
        autoChooser.setDefaultOption("No-op (unlock hands)", this::createNoOpAuto);
        autoChooser.addOption("Basic/Time-based drive", this::createSimpleDriveAuto);
        autoChooser.addOption("Testing/Time-based shake drive", this::createSimpleShakeAuto);
        autoChooser.addOption("Testing/Forward Test Trajectory", this::createTestTrajAuto);

        // Create event map //
        // Bring claw to front ground and open
        autoEventMap.put("clawFrontGrabPos", new ParallelCommandGroup(
            new InstantCommand(() -> { arm.setTarget(ArmPositions.kFRONT_GROUND); }, arm),
            new InstantCommand(() -> { claw.setPosition(GrabPositions.RELEASE); }, claw)
        ));
    }

    // Get Selected Auto
    public Command getSelectedAuto() {
        Supplier<Command> auto = autoChooser.getSelected();
        return (auto == null) ? null : auto.get();
    }

    public SendableChooser<Supplier<Command>> getChooser() { return autoChooser; }

    // CREATE AUTO COMMANDS //
    public Command createNoOpAuto() {
        return new UnlockClawCommand(claw, arm);
    }

    public Command createTestTrajAuto() {
        return new ParallelCommandGroup(
            drive.getTrajectoryCommand("spot1/spot1ToPiece1", autoEventMap, true, 2.0, 1.5)
        );
    }

    public Command createSimpleDriveAuto() {
        return new SequentialCommandGroup(
            new UnlockClawCommand(claw, arm),
            new SimpleAutoForward(drive, arm)
        );
    }

    public Command createSimpleShakeAuto() {
        return new SequentialCommandGroup(
            new UnlockClawCommand(claw, arm),
            new SimpleAutoShake(drive, arm)
        );
    }
}
