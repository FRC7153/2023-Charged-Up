package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

/**
 * This is untested, do not use!
 */
public class SimpleAutoShake extends CommandBase {
    private DriveBase drive;
    private Arm arm;
    private double startTime;

    public SimpleAutoShake(DriveBase driveSubsystem, Arm armSubsystem) {
        drive = driveSubsystem;
        arm = armSubsystem;

        addRequirements(drive, arm);
    }

    @Override
    public void initialize() {
        //drive.setCoast(false);

        drive.driveFieldOriented(0.0, -0.6, 0.0);
        arm.setAngle(0.0);
        arm.setExtension(0.0);

        startTime = Timer.getFPGATimestamp();
        DriverStation.reportWarning("== SIMPLE AUTO START ==", false);
    }

    @Override
    public void execute() {
        // 1 second out, 1 second in, 2.5 seconds out

        if (Timer.getFPGATimestamp() - startTime >= 4.5) {
            DriverStation.reportWarning("AUTO STOPPED (time)", false);
            drive.stop();
        } else if (Timer.getFPGATimestamp() - startTime >= 2.0) {
            drive.driveRobotOriented(0.0, -0.6, 0.0);
        } else if (Timer.getFPGATimestamp() - startTime >= 1.0) {
            drive.driveRobotOriented(0.0, 0.6, 0.0);
        } else {
            drive.driveRobotOriented(0.0, -0.6, 0.0);
        }
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupt) {
        drive.stop();
    }
}
