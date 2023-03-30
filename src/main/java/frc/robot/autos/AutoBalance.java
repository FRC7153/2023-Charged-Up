package frc.robot.autos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.DriveBase;

public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance(DriveBase drive) {
        addRequirements(drive);

        addCommands(
            new InstantCommand(() -> { drive.driveRobotOriented(0.0, -0.7, 0.0); }, drive),
            new WaitUntilCommand(() -> { return Math.abs(drive.imu.getPitch()) > 20.0; }),
            new WaitUntilCommand(() -> { return Math.abs(drive.imu.getPitch()) < 10.0; }),
            new InstantCommand(() -> { drive.driveRobotOriented(0.0, 0.0, 0.0); }, drive),
            new BalanceCommand(drive)
        );
    }
}
