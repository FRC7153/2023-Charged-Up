package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

public class SimpleAutoForward extends CommandBase {
    private DriveBase drive;
    private Arm arm;
    private double startTime;
    private double moveTime;

    public SimpleAutoForward(DriveBase driveSubsystem, Arm armSubsystem, double  moveTime) {
        drive = driveSubsystem;
        arm = armSubsystem;
        this.moveTime = moveTime;

        addRequirements(drive, arm);
    }

    public SimpleAutoForward(DriveBase driveSubsystem, Arm armSubsystem) { this(driveSubsystem, armSubsystem, 2.5); }

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
        if (Timer.getFPGATimestamp() - startTime >= moveTime) {
            DriverStation.reportWarning("AUTO STOPPED (time)", false);
            drive.stop();
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
