package frc.robot.commands;

import com.frc7153.math.MathUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

public class BalanceCommand extends CommandBase {
    // PID Control
    private static PIDController balancePid = AutoConstants.kBALANCE_PID.toWPIPidController();

    // Drive Subsystem
    private DriveBase drive;
    private Arm arm;

    // Constructor
    public BalanceCommand(DriveBase driveSubsys, Arm armSubsys) {
        drive = driveSubsys;
        arm = armSubsys;

        addRequirements(drive, arm);
    }

    // Init
    @Override
    public void initialize() {
        drive.driveTankAbsolute(0.0, 0.0);
        arm.setAngle(0.0);
        arm.setExtension(0.0);

        balancePid.setSetpoint(0.0);
    }

    // Run
    @Override
    public void execute() {
        double speed = MathUtils.symmetricClamp(balancePid.calculate(drive.imu.getPitch()), AutoConstants.kMAX_BALANCE_SPEED);
        drive.driveTankAbsolute(-speed, -speed);
    }

    // End
    @Override
    public void end(boolean terminated) {
        drive.stop();
    }

    // Check if balanced
    @Override
    public boolean isFinished() {
        return Math.abs(drive.imu.getPitch()) <= balancePid.getPositionTolerance();
    }
}
