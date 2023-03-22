package frc.robot.autos;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GrabPositions;
import frc.robot.commands.BalanceCommand;
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
    private static enum AutoType {NOP, SIMPLE_TIME, SIMPLE_TIME_SHAKE, GYRO_BALANCE, TRAJ_TEST};
    private SendableChooser<AutoType> autoChooser = new SendableChooser<>();

    // Event Map
    public HashMap<String, Command> autoEventMap = new HashMap<>();

    // Constructor
    public Autonomous(DriveBase driveSubsys, Arm armSubsys, Claw clawSubsys) {
        // Save subsystems
        drive = driveSubsys;
        arm = armSubsys;
        claw = clawSubsys;

        // Create Auto Chooser
        autoChooser.setDefaultOption("No-op (unlock hands)", AutoType.NOP);
        autoChooser.addOption("Time-based drive", AutoType.SIMPLE_TIME);
        autoChooser.addOption("Time-based shake drive", AutoType.SIMPLE_TIME_SHAKE);
        autoChooser.addOption("Gyro-based balance", AutoType.GYRO_BALANCE);
        autoChooser.addOption("Forward Test Trajectory", AutoType.TRAJ_TEST);

        // Create event map //
        // Bring claw to front ground and open
        autoEventMap.put("clawFrontGrabPos", new ParallelCommandGroup(
            new InstantCommand(() -> { arm.setTarget(ArmPositions.kFRONT_GROUND); }, arm),
            new InstantCommand(() -> { claw.setPosition(GrabPositions.RELEASE); }, claw)
        ));
    }

    // Get Selected Auto
    public Command getSelectedAuto() {
        switch (autoChooser.getSelected()) {
            case SIMPLE_TIME:
                // Simple time-based drive forward
                return getSimpleDriveAuto();
            case SIMPLE_TIME_SHAKE:
                return getSimpleShakeAuto();
            case GYRO_BALANCE:
                // Use gyro to balance
                return getGyroBalanceAuto();
            case TRAJ_TEST:
                return getTestTrajAuto();
            case NOP:
            default:
                // No-op
                return new UnlockClawCommand(claw, arm);
        }
    }

    public SendableChooser<AutoType> getChooser() { return autoChooser; }

    // GETTERS //
    public Command getTestTrajAuto() {
        return new ParallelCommandGroup(
            drive.getTrajectoryCommand("spot1/spot1ToPiece1", autoEventMap, true, 2.0, 1.5)
        );
    }

    public Command getSimpleDriveAuto() {
        return new SequentialCommandGroup(
            new UnlockClawCommand(claw, arm),
            new SimpleAutoForward(drive, arm)
        );
    }

    public Command getSimpleShakeAuto() {
        return new SequentialCommandGroup(
            new UnlockClawCommand(claw, arm),
            new SimpleAutoShake(drive, arm)
        );
    }

    public Command getGyroBalanceAuto() {
        return new ParallelCommandGroup(
            new UnlockClawCommand(claw, arm),
            new SequentialCommandGroup(
                new DriveUntilAngle(drive),
                new BalanceCommand(drive)
            )
        );
    }
}