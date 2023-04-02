package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BangBalanceCommand;
import frc.robot.subsystems.DriveBase;

public class AutoBalance extends SequentialCommandGroup {
    /**
     * Create auto balance routine. 
     * @param drive
     * @param invertDriveUp
     */
    public AutoBalance(DriveBase drive, boolean invertDriveUp) {
        addRequirements(drive);

        addCommands(
            new InstantCommand(() -> { drive.driveRobotOriented(0.0, (invertDriveUp) ? 0.7 : -0.7, 0.0); }, drive),
            //new InstantCommand(() -> drive.driveDiag((invertDriveUp) ? 0.7 : -0.7, true)),
            new WaitUntilCommand(() -> { return Math.abs(drive.imu.getPitch()) > 10.0; }), // TODO higher angle
            //new WaitUntilCommand(() -> { return Math.abs(drive.imu.getPitch()) < 10.0; }),
            new PrintCommand("## STARTED BALANCING!"),
            new InstantCommand(() -> { drive.driveRobotOriented(0.0, 0.0, 0.0); }, drive),
            //new BalanceCommand(drive)
            new BangBalanceCommand(drive)
        );
    }

    // Give up control
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }

    // Default, do not invert drive up speed
    public AutoBalance(DriveBase drive) { this(drive, false); }
}
