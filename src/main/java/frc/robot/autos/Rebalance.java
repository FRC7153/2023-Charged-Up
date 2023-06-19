package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveBase;

public class Rebalance extends SequentialCommandGroup {
    // Subsystems
    private DriveBase drive;

    private double diag = 45.0;
    private boolean ang = true;
    
    // Init
    public Rebalance(DriveBase drive) {
        this.drive = drive;

        addRequirements(drive);

        addCommands(
            new InstantCommand(this::setWheels2),
            new WaitCommand(0.45), // 0.5
            new InstantCommand(drive::lockWheels),
            new WaitCommand(0.5) // 0.9
        );
    }

    // Run
    private void setWheels() {
        if (drive.imu.getPitch() > 4.2) {
            //drive.driveRobotOriented(0.0, 0.67, 0.0);
            drive.driveDiag(0.67, diag);
        } else if (drive.imu.getPitch() < -4.2) {
            //drive.driveRobotOriented(0.0, -0.67, 0.0);
            drive.driveDiag(0.67, diag);
        } else {
            drive.lockWheels();
        }

        diag = (diag == 45.0) ? 90.0 + 45.0 : 45.0;
    }

    private void setWheels2() {
        if (Math.abs(drive.imu.getPitch()) > 4.2) {
            double dir = (drive.imu.getPitch() > 4.2) ? 0.67 : -0.67;

            drive.driveDiag(
                (ang) ? dir : -dir,
                (ang) ? 45.0 : 90.0 + 45.0
            );

            ang = !ang;
        } else {
            drive.lockWheels();
        }
    }
}
