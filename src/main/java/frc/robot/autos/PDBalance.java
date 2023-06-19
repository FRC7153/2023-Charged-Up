package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class PDBalance extends CommandBase {
    // Subsys
    private DriveBase base;

    private PIDController balancePID = new PIDController(0.04, 0.0, 0.08);

    public PDBalance(DriveBase driveSubsys) {
        base = driveSubsys;

        addRequirements(base);

        balancePID.setSetpoint(0.0);
    }

    // Init
    @Override
    public void initialize() {
        base.driveTankAbsolute(0.0, 0.0);
    }

    // Run
    @Override
    public void execute() {
        double speed = balancePID.calculate(base.imu.getPitch());

        base.driveRobotOriented(0.0, -speed, 0.0);
    }

    // Stop
    @Override
    public void end(boolean terminate) {
        base.stop();
    }
}
