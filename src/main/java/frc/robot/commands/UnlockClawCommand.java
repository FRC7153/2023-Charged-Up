package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GrabPositions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class UnlockClawCommand extends CommandBase {
    // Subsystems
    private Claw claw;
    private Arm arm;

    // State
    private Double releaseTime;

    // Constructor
    public UnlockClawCommand(Claw clawSubsys, Arm armSubsys) {
        claw = clawSubsys;
        arm = armSubsys;

        addRequirements(claw, arm);
    }

    // Init
    @Override
    public void initialize() {
        releaseTime = Double.NaN;
        claw.setCoastMode(true);

        arm.setWinchEncPosition(1.0);
        arm.setExtension(0.0);
    }

    // Periodic
    @Override
    public void execute() {
        if (arm.getWinchEncPos() < 0.2) {
            claw.setPosition(GrabPositions.RELEASE);
            releaseTime = Timer.getFPGATimestamp();
        }
    }

    // End
    @Override
    public boolean isFinished() {
        return (!releaseTime.isNaN() && Timer.getFPGATimestamp() - releaseTime >= 0.1);
    }
}
