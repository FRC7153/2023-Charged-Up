package frc.robot.commands;

import com.frc7153.math.MathUtils;
import com.frc7153.math.ShuffleboardPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveBase;

public class BalanceCommand extends CommandBase {
    // PID Control
    //private static ShuffleboardPIDController balancePid = AutoConstants.kBALANCE_PID.toShuffleboardPIDController("balance pid");

    // Drive Subsystem
    private DriveBase drive;

    // Constructor
    public BalanceCommand(DriveBase driveSubsys) {
        drive = driveSubsys;

        addRequirements(drive);
    }

    // Init
    @Override
    public void initialize() {
        drive.driveTankAbsolute(0.0, 0.0);

        RobotContainer.balancePID.setSetpoint(0.0);

        dir = 0.4;
        lastSwitch = Timer.getFPGATimestamp();
    }

    // Run
    double dir = 0.6;
    double lastSwitch = 0.0;

    boolean kBANG = false;

    @Override
    public void execute() {
        if (!kBANG) {
            RobotContainer.balancePID.refresh();

            double speed = RobotContainer.balancePID.calculate(drive.imu.getPitch());

            // MARK kFF
            double kff = RobotContainer.balanceKFF.getDouble(0.0);
            RobotContainer.balanceKFFO.setDouble(kff);

            speed += (drive.imu.getPitch() * kff);
            
            DriverStation.reportWarning(String.format("Angle -> %s, Speed -> %s, kFF -> %s", drive.imu.getPitch(), -speed, drive.imu.getPitch() * kff), false);

            drive.driveRobotOriented(0.0, -speed, 0.0);
        } else {
        // END MARK

        //drive.driveRobotOriented(0.0, -speed, 0.0);
        //double speed = 0.0;

        // MARK BANG
            double speed = 0.0;
            if (drive.imu.getPitch() > 5.0) {
                speed = 0.4;
            } else if (drive.imu.getPitch() < 5.0) {
                speed = -0.4;
            }

            drive.driveRobotOriented(0.0, speed, 0.0);
        }

        // END MARK

        //drive.driveDiag(-speed, drive.imu.getPitch() * 1.5); // 25.0
        //drive.driveRobotOriented(dir, -speed, 0.0);
        /*drive.driveRobotOriented(0.0, -speed, (Timer.getFPGATimestamp() - lastSwitch) * dir);

        if (Timer.getFPGATimestamp() - lastSwitch >= 1.0) {
            dir = -dir;
            lastSwitch = Timer.getFPGATimestamp();
        }*/
    }

    // End
    @Override
    public void end(boolean terminated) {
        drive.stop();
    }

    // Check if balanced
    @Override
    public boolean isFinished() {
        return false;
        //return Math.abs(drive.imu.getPitch()) <= balancePid.getPositionTolerance();
    }
}
