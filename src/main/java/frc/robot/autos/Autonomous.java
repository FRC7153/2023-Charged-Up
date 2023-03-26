package frc.robot.autos;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GrabPositions;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.PresetArmCommand;
import frc.robot.commands.UnlockClawCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Arm.ArmState;

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
        autoChooser.addOption("Basic - Time-based drive", this::createSimpleDriveAuto);
        autoChooser.addOption("Testing - Time-based shake drive", this::createSimpleShakeAuto);
        autoChooser.addOption("Testing - Square Test Trajectory", this::createTestTrajAuto);
        autoChooser.addOption("Spot 1 - 3 Piece", this::createSpot1_3PieceAuto);

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
            drive.getTrajectoryCommand("SquareTest", autoEventMap, true, 3.0, 4.0)
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

    // SPOT 1
    // 3 Piece Auto
    public Command createSpot1_3PieceAuto() {
        return new SequentialCommandGroup(
            new UnlockClawCommand(claw, arm, true),
            // Piece 1 (CONE, staged)
            new GrabCommand(claw, GrabPositions.GRAB),
            new PresetArmCommand(arm, ArmPositions.kREAR_CONE_HIGH),
            new GrabCommand(claw, GrabPositions.WIDE_RELEASE),
            new PresetArmCommand(arm, new ArmState(0.0, ArmConstants.kJOINT_TO_EXT_PT)),
            new GrabCommand(claw, GrabPositions.RELEASE),
            // Grab Piece 2 (CUBE)
            drive.getTrajectoryCommand("spot1/spot1ToPiece1", autoEventMap, true, 2.0, 1.0)
            //new GrabCommand(claw, GrabPositions.GRAB)
        );
    }
}
