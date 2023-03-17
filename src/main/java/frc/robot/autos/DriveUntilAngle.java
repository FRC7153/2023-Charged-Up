package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

/**
 * Drives until robot is at angle
 */
public class DriveUntilAngle extends CommandBase {
    // Subsystems
    private DriveBase drive;

    // Constructor
    public DriveUntilAngle(DriveBase driveSubsys) {
        drive = driveSubsys;
        
        addRequirements(drive);
    }

    // Init
    @Override
    public void initialize() {
        drive.driveTankAbsolute(-0.7, -0.7);
    }

    // Done
    @Override
    public void end(boolean terminated) {
        drive.stop();
    }

    // Check if done
    @Override
    public boolean isFinished() {
        return Math.abs(drive.imu.getPitch()) > 15.0;
    }
}
