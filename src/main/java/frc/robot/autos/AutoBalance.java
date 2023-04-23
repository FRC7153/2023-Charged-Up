package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

public class AutoBalance extends SequentialCommandGroup {
    /**
     * Create auto balance routine. 
     * @param drive
     * @param direction true is forward, false is backwards
     */
    public AutoBalance(DriveBase drive, Arm arm) {
        addRequirements(drive);

        addCommands(
            // Drive up
            new InstantCommand(() -> { drive.driveRobotOriented(0.0, 0.7, 0.0); }, drive),
            new WaitUntilCommand(() -> { return drive.imu.getPitch() >= 10.5; }),
            // Balance
            new BalanceCommand(drive, arm, false),
            new InstantCommand(() -> { drive.lockWheels(); }, drive),
            new WaitCommand(0.7),
            // Fix Balance
            //new BangBalancer(drive, arm),
            (new Rebalance(drive)).repeatedly()
        );
    }

    // Give up control
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
