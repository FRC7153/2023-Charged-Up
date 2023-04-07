package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

public class BalanceCommand extends CommandBase {
    // Drive Subsys
    private DriveBase drive;
    private Arm arm;
    
    // Config
    private boolean direction;
    private double startTime;
    private double maxAngle;

    // Constructor
    /**
     * 
     * @param driveBase
     * @param direction true is forward, false is backwards
     */
    public BalanceCommand(DriveBase driveBase, Arm armSubsys, boolean direction) {
        drive = driveBase;
        arm = armSubsys;
        this.direction = direction;

        addRequirements(drive, arm);
    }

    // Init
    @Override
    public void initialize() {
        drive.driveRobotOriented(0.0, (direction) ? -0.7 : 0.7, 0.0);
        //arm.setExtension(ArmConstants.kJOINT_TO_EXT_PT);
        //arm.setAngle((direction) ? 90.0 : -90.0);

        startTime = Timer.getFPGATimestamp();
        maxAngle = drive.imu.getPitch();
    }

    // Execute
    @Override
    public void execute() {
        if (direction) {

        } else {
            if (drive.imu.getPitch() > maxAngle) {
                maxAngle = drive.imu.getPitch();
            }
        }
    }

    // End
    @Override
    public void end(boolean terminated) {
        //drive.lockWheels();
        //arm.setAngle(0.0);
    }

    // Is done
    @Override
    public boolean isFinished() {
        if (direction && false) {
            return (drive.imu.getPitch() >= 10.5) && (Timer.getFPGATimestamp() - startTime >= 0.0);
        } else {
            //return (drive.imu.getPitch() <= -9.5) && (Timer.getFPGATimestamp() - startTime >= 0.0); // 10.5
            //DriverStation.reportWarning(String.format("%s, %s", maxAngle, drive.imu.getPitch()), false);
            return maxAngle - drive.imu.getPitch() > 3.0 && Timer.getFPGATimestamp() - startTime >= 0.5; // 1.5
        }
    }
}
