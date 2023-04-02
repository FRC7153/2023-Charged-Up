package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class BangBalanceCommand extends CommandBase {
    // Drive
    private DriveBase drive;

    private Double hasAngleFallen = Double.NaN;

    // Constructor
    public BangBalanceCommand(DriveBase driveSubsys) {
        drive = driveSubsys;

        addRequirements(driveSubsys);
    }

    // Run
    @Override
    public void initialize() {
        drive.stop();
        hasAngleFallen = Double.NaN;
    }

    @Override
    public void execute() {
        if (hasAngleFallen.isNaN()) {
            if (drive.imu.getPitch() < 0.0)  { // TODO: aggressive angle
                drive.stop();
                hasAngleFallen = Timer.getFPGATimestamp();
            } else {
                drive.driveRobotOriented(0.0, 0.55, 0.0);
            }
        } else if (Timer.getFPGATimestamp() - hasAngleFallen > 1.0) {
            System.out.println("BANG CONTROLLER");
            if (true) { return; }
            // bang?
            if (drive.imu.getPitch() > 2.0) {
                drive.driveRobotOriented(0.0, 0.3, 0.0);
            } else if (drive.imu.getPitch() < 2.0) {
                drive.driveRobotOriented(0.0, -0.3, 0.0);
            } else {
                drive.driveRobotOriented(0.0, 0.0, 0.0);
            }
        }
    }

    @Override
    public void end(boolean terminated) {
        drive.stop();
    }

    @Override
    public boolean isFinished() { return false; }
}
