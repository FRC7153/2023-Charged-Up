package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

public class BangBalancer extends CommandBase {
    // Drive
    private DriveBase drive;
    private Arm arm;

    // Constructor
    public BangBalancer(DriveBase drive, Arm arm) {
        this.drive = drive;
        this.arm = arm;

        addRequirements(drive, arm);
    }

    // Init
    @Override
    public void initialize() {
        arm.setExtension(ArmConstants.kJOINT_TO_EXT_PT);
        arm.setAngle(45.0);
    }

    // Execute
    @Override
    public void execute() {
        if (drive.imu.getPitch() > 4.0) {
            drive.driveRobotOriented(0, 0.66, 0);
        } else if (drive.imu.getPitch() < -4.0) {
            drive.driveRobotOriented(0, -0.66, 0);
        } else {
            drive.lockWheels();
        }
    }

    // Done
    @Override
    public void end(boolean terminated) {
        drive.lockWheels();
        arm.setAngle(0.0);
    }

    // Is Finished
    @Override
    public boolean isFinished() {
        return Math.abs(drive.imu.getPitch()) < 4.0;
    }
}
