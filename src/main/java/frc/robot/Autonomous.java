package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GrabPositions;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.PresetArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

/**
 * Manages auto events and commands
 */
public class Autonomous {
    // Subsystems
    private DriveBase drive;
    private Arm arm;
    private Claw claw;

    // Event Map
    public HashMap<String, Command> autoEventMap = new HashMap<>();

    // Constructor
    public Autonomous(DriveBase driveSubsys, Arm armSubsys, Claw clawSubsys) {
        // Save subsystems
        drive = driveSubsys;
        arm = armSubsys;
        claw = clawSubsys;

        // Create event map //
        // Bring claw to front ground and open
        autoEventMap.put("clawToFrontGround", new ParallelCommandGroup(
            new PresetArmCommand(arm, ArmPositions.kFRONT_GROUND),
            new GrabCommand(claw, GrabPositions.RELEASE)
        ));
    }

    // GETTERS //
    public Command getTestSpinAuto() {
        // Note that this will NOT unlock hands
        return drive.getTrajectoryCommand("StraightCurveSpinMove", autoEventMap, true, 2.0, 1.5);
    }
}
