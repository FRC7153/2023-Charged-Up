package frc.robot.autos;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GrabPositions;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.PresetArmCommand;
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
    private SendableChooser<Alliance> allianceChooser = new SendableChooser<>();

    // Event Map
    public HashMap<String, Command> autoEventMap = new HashMap<>();

    // Create Instant Arm Command
    private Command instantArmCommand(double x, double y) { return new InstantCommand(() -> { arm.setTarget(x, y);}, arm); }
    private Command instantArmCommand(Translation2d pos) { return new InstantCommand(() -> { arm.setTarget(pos); }, arm); }


    // Instant Home Command (for fast autos)
    private Command createInstantHomeCommand() {
        return new InstantCommand(() -> { arm.setWinchEncPosition(ArmConstants.kWINCH_HOME_ROT_POS); arm.hasBeenReleased = true; }, arm);
    }

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
        autoChooser.addOption("Spot 1 - 2 Piece", this::createSpot1_2PieceAuto);
        autoChooser.addOption("Spot 3 - 2 Piece", this::createSpot3_2PieceAuto);
        autoChooser.addOption("High Cone + Sit", this::create_HighConeAndSitAuto);
        autoChooser.addOption("Spot 2 - Balance (!)", this::createSpot2_BalanceAuto);
        autoChooser.addOption("Spot 2 - 1 Piece Balance (!)", this::createSpot2_1PieceBalanceAuto);
        autoChooser.addOption("Spot 1/3 - 1 Piece Drive", this::createAnySpot_1PieceMove);

        // Create Alliance Chooser
        allianceChooser.setDefaultOption("Blue Alliance", Alliance.Blue);
        allianceChooser.addOption("Red Alliance", Alliance.Red);

        // Create event map //
        // Bring arm to straight up
        autoEventMap.put("armUp", instantArmCommand(0.0, ArmConstants.kJOINT_TO_EXT_PT));

        autoEventMap.put("armSafeUp", new SequentialCommandGroup(
            //new InstantCommand(() -> { arm.setExtension(ArmConstants.kJOINT_TO_EXT_PT); }), 
            //new WaitCommand(1.5), // 0.5
            instantArmCommand(0.0, ArmConstants.kJOINT_TO_EXT_PT)
        ));

        // Bring claw to front ground and open
        autoEventMap.put("clawFrontGrabPos", new ParallelCommandGroup(
            instantArmCommand(AutoConstants.kFRONT_CUBE_GROUND),
            new GrabCommand(clawSubsys, GrabPositions.WIDE_RELEASE)
        ));

        // Bring claw lower for certain autos
        autoEventMap.put("clawFrontLowGrabPos", new ParallelCommandGroup(
            instantArmCommand(AutoConstants.kFRONT_LOW_CUBE_GROUND),
            new GrabCommand(clawSubsys, GrabPositions.WIDE_RELEASE)
        ));

        // Grab with claw
        autoEventMap.put("grab", new GrabCommand(clawSubsys, GrabPositions.GRAB));
    }

    // Get Selected Auto
    public Command getSelectedAuto() {
        Supplier<Command> auto = autoChooser.getSelected();
        return (auto == null) ? null : auto.get();
    }

    public SendableChooser<Supplier<Command>> getChooser() { return autoChooser; }
    public SendableChooser<Alliance> getAllianceChooser() { return allianceChooser; }

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
    // 2 Piece Auto
    public Command createSpot1_2PieceAuto() {
        return new SequentialCommandGroup(
            // Unlock Claw
            createInstantHomeCommand(),
            // Piece 1 (CONE, staged)
            new GrabCommand(claw, GrabPositions.GRAB),
            //new PresetArmCommand(arm, new ArmState(-45.0, 215.39)),
            new PresetArmCommand(arm, AutoConstants.kREAR_CONE_HIGH),
            new GrabCommand(claw, GrabPositions.STOW),
            new WaitCommand(0.2),
            new GrabCommand(claw, GrabPositions.WIDE_RELEASE),
            // Grab Piece 2 (CUBE)
            drive.getTrajectoryCommand(
                String.format("%sSpot1/spot1ToPiece1", allianceChooser.getSelected().equals(Alliance.Red) ? "red" : "blue"), 
                autoEventMap, 
                true, 
                3.0, // 2.0
                1.5 //1.0
            ),
            new PresetArmCommand(arm, ArmPositions.kREAR_CUBE_HIGH),
            new GrabCommand(claw, GrabPositions.WIDE_RELEASE),
            new WaitCommand(0.3),
            new GrabCommand(claw, GrabPositions.RELEASE),
            instantArmCommand(0.0, ArmConstants.kJOINT_TO_EXT_PT)
        );
    }

    // SPOT 2
    // Only Balance
    public Command createSpot2_BalanceAuto() {
        return new SequentialCommandGroup(
            // Unlock arm
            createInstantHomeCommand(),
            // Over shoot for taxi points
            new InstantCommand(() -> drive.driveRobotOriented(0.0, -0.7, 0.0)),
            new WaitCommand(3.5),
            new InstantCommand(() -> drive.driveRobotOriented(0.0, 0.0, 0.0)),
            // Balancing
            new AutoBalance(drive, arm)
        );
    }

    // 1 High, balance
    public Command createSpot2_1PieceBalanceAuto() {
        return new SequentialCommandGroup(
            // Unlock Claw
            createInstantHomeCommand(),
            // Piece 1 (CONE, staged)
            new GrabCommand(claw, GrabPositions.GRAB),
            new PresetArmCommand(arm, AutoConstants.kREAR_CONE_HIGH),
            new GrabCommand(claw, GrabPositions.STOW),
            new WaitCommand(0.2),
            new GrabCommand(claw, GrabPositions.WIDE_RELEASE),
            // Claw to 0
            instantArmCommand(0.0, ArmConstants.kJOINT_TO_EXT_PT),
            // Over shoot for taxi points
            new InstantCommand(() -> drive.driveRobotOriented(0.0, -0.7, 0.0)),
            new WaitCommand(3.5),
            new PrintCommand("## MOVING TO BALANCE ROUTINE"),
            new InstantCommand(() -> drive.driveRobotOriented(0.0, 0.0, 0.0)),
            // Balance
            new AutoBalance(drive, arm)
        );
    }

    // 1 High and sit
    public Command create_HighConeAndSitAuto() {
        return new SequentialCommandGroup(
            // Unlock Claw
            createInstantHomeCommand(),
            // Piece 1 (CONE, staged)
            new GrabCommand(claw, GrabPositions.GRAB),
            new PresetArmCommand(arm, AutoConstants.kREAR_CONE_HIGH),
            new GrabCommand(claw, GrabPositions .STOW),
            new WaitCommand(0.2),
            new GrabCommand(claw, GrabPositions.WIDE_RELEASE),
            // Claw to 0
            instantArmCommand(0.0, ArmConstants.kJOINT_TO_EXT_PT)
        );
    }

    // SPOT 3
    // 2 Piece Auto
    public Command createSpot3_2PieceAuto() {
        return new SequentialCommandGroup(
            // Unlock Claw
            createInstantHomeCommand(),
            // Piece 1 (CONE, staged)
            new GrabCommand(claw, GrabPositions.GRAB),
            new PresetArmCommand(arm, AutoConstants.kREAR_CONE_HIGH),
            new GrabCommand(claw, GrabPositions.STOW),
            new WaitCommand(0.2),
            new GrabCommand(claw, GrabPositions.WIDE_RELEASE),
            // Grab Piece 2 (CUBE)
            drive.getTrajectoryCommand(
                String.format("%sSpot3/spot3ToPiece1", allianceChooser.getSelected().equals(Alliance.Red) ? "red" : "blue"), 
                autoEventMap, 
                true, 
                3.0, // 2.0
                1.5 // 1.0
            ),
            new PresetArmCommand(arm, ArmPositions.kREAR_CUBE_HIGH),
            new GrabCommand(claw, GrabPositions.WIDE_RELEASE),
            new WaitCommand(0.3),
            new GrabCommand(claw, GrabPositions.RELEASE),
            instantArmCommand(0.0, ArmConstants.kJOINT_TO_EXT_PT)
        );
    }

    // ANY SPOT
    // Cone high and time based move
    public Command createAnySpot_1PieceMove() {
        return  new SequentialCommandGroup(
            // Unlock Claw
            createInstantHomeCommand(),
            // Piece 1 (CONE, staged)
            new GrabCommand(claw, GrabPositions.GRAB),
            new PresetArmCommand(arm, AutoConstants.kREAR_CONE_HIGH),
            new GrabCommand(claw, GrabPositions.STOW),
            new WaitCommand(0.2),
            new GrabCommand(claw, GrabPositions.WIDE_RELEASE),
            // Move
            new SimpleAutoForward(drive, arm, 3.0)
        );
    }
}
