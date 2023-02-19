package frc.robot.subsystems;

import java.util.Map;

import com.fasterxml.jackson.databind.deser.impl.ExternalTypeHandler.Builder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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

    // Raspberry Pi Diagnostics
    private GenericEntry piVoltageStable;
    private GenericEntry piCPUTemp;
    private GenericEntry piCPUUsage;
    private GenericEntry piMemUsage;
    private GenericEntry piFPS;
    private GenericEntry piAge;
    private GenericEntry piCache;

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

        // RaspberryPi Values
        ShuffleboardTab piTab = Shuffleboard.getTab("Pi");

        piCPUTemp = piTab.add("CPU Temp (f)", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(0, 0)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", false, "UPPER BOUND", 185, "LOWER BOUND", 0, "UNIT", "f"))
            .getEntry();
        
        piCPUUsage = piTab.add("CPU Usage (%)", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(3, 0)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", false, "UPPER BOUND", 1.0, "LOWER BOUND", 0.0, "UNIT", "%"))
            .getEntry();

        piMemUsage = piTab.add("Memory (%)", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(6, 0)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", false, "UPPER BOUND", 1.0, "LOWER BOUND", 0.0, "UNIT", "%"))
            .getEntry();

        piFPS = piTab.add("FPS", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(0, 3)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", true, "UNIT", "FPS"))
            .getEntry();

        piAge = piTab.add("Latest Packet Age (s)", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(3, 3)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", true, "UNIT", "s"))
            .getEntry();

        piVoltageStable = piTab.add("Voltage Stable", true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(6, 4)
            .getEntry();
        
        piCache = piTab.add("Latest Cache", "?")
            .withPosition(6, 3)
            .withSize(2, 1)
            .getEntry();
    }

    // Update Values
    @Override
    public void periodic() {
        // Controllers
        controller0Update.setString(String.format("%s seconds", Robot.controller0.getLastOffsetUpdate()));

        // Pi
        piCPUTemp.setDouble(Robot.armPi.getTemp());
        piCPUUsage.setDouble(Robot.armPi.getCPU());
        piMemUsage.setDouble(Robot.armPi.getMemory());
        piFPS.setDouble(Robot.armPi.getFPS());
        piAge.setDouble(Robot.armPi.getAge());
        piVoltageStable.setBoolean(Robot.armPi.getVoltageStable());
        piCache.setString(Robot.armPi.getCache());
    }
}
