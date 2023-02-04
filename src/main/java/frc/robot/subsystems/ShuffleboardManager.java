package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;

/**
 * Handles all values and commands put on Shuffleboard
 */
public class ShuffleboardManager {
    // Tabs
    private ShuffleboardTab controllerTab = Shuffleboard.getTab("Controllers");

    // Constructor (Init)
    public ShuffleboardManager() {
        // Controller Init
        controllerTab.add(Robot.controller0.new CalibrateOffsetCommand())
            .withPosition(0, 0)
            .withSize(2, 1);
    }
}
