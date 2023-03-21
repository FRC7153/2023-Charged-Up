package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class ChassisSpeedTestCommand extends CommandBase {
    private DriveBase drive;

    public ChassisSpeedTestCommand(DriveBase driveSubsys) {
        drive = driveSubsys;
    }

    public void execute() {
        drive.setWheelStates(
            new SwerveModuleState[] {
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(90))
            }
        );
    }
}
