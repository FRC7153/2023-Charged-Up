package frc.robot.peripherals;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    // Preset Modes
    public static enum Mode {APRIL_TAGS(0), TAPE(1);
    
        public final int index;
        Mode(int index) { this.index = index; }
    }
    
    // Config
    private String name;
    private Mode mode = Mode.APRIL_TAGS;

    // Init
    public Limelight(String name) {
        this.name = name;
    }

    // Get NT Entry
    private NetworkTable getTable() { return NetworkTableInstance.getDefault().getTable(String.format("limelight-%s", name)); }

    // Force LED mode (off or pipeline)
    public void forceLEDMode(boolean state) {
        getTable().getEntry("ledMode").setNumber(state ? 0 : 1);
    }

    // (Lazy) set mode
    public void setMode(Mode m) {
        if (m.equals(mode)) { return; }

        mode = m;
        getTable().getEntry("pipeline").setNumber(m.index);
    }
}
