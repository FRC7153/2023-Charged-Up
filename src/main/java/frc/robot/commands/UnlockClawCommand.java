package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GrabPositions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class UnlockClawCommand extends CommandBase {
    // Subsystems
    private Claw claw;
    private Arm arm;

    // State
    private Double releaseTime;
    private Boolean doNotExtend = false;

    // Constructor
    public UnlockClawCommand(Claw clawSubsys, Arm armSubsys, boolean doNotExtend) {
        claw = clawSubsys;
        arm = armSubsys;
        
        this.doNotExtend = doNotExtend;

        addRequirements(claw, arm);
    }

    public UnlockClawCommand(Claw clawSubsys, Arm armSubsys) { this(clawSubsys, armSubsys, false); }

    // Init
    @Override
    public void initialize() {
        releaseTime = Timer.getFPGATimestamp();
        arm.hasBeenReleased = false;

        claw.setCoastMode(true);

        arm.setAngle((doNotExtend) ? Double.NaN : 0.0);

        arm.setWinchEncPosition(ArmConstants.kWINCH_HOME_ROT_POS); // 2.54
        arm.setExtension(0.0);
    }

    // Periodic
    @Override
    public void execute() {
        if (!releaseTime.isNaN() && Timer.getFPGATimestamp() - releaseTime >= 0.8) {
            claw.setPosition(GrabPositions.RELEASE);
            arm.hasBeenReleased = true;
        }
    }

    @Override
    public boolean isFinished() {
        return arm.hasBeenReleased;
    }

    // More Important!
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
