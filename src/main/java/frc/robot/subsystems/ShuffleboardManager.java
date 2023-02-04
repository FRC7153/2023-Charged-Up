package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/**
 * Handles all values and commands put on Shuffleboard
 */
public class ShuffleboardManager extends SubsystemBase {
    // Controller Update Counter
    private GenericEntry controller0Update;

    // Constructor (Init)
    public ShuffleboardManager() {
        // Controller Tab Init
        ShuffleboardTab controllerTab = Shuffleboard.getTab("Controllers");
        ShuffleboardLayout controllerRecalibrate = controllerTab.getLayout("Controllers", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withProperties(Map.of("LABEL POSITION", "HIDDEN"));

        controllerRecalibrate.add(Robot.controller0.new CalibrateOffsetCommand());

        ShuffleboardLayout controllerUpdates = controllerTab.getLayout("Updates", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0)
            .withProperties(Map.of("LABEL POSITION", "TOP"));
        
        controller0Update = controllerUpdates.add("Controller 0", "No updates...").getEntry();
    }

    // Update Values
    @Override
    public void periodic() {
        controller0Update.setString(String.format("%s seconds", Robot.controller0.getLastOffsetUpdate()));
    }
}
