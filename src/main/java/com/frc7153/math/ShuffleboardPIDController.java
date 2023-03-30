package com.frc7153.math;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * For putting a Profiled Pid Controller onto Shuffleboard for quick tuning.
 * <br><br>
 * It is recommended to run the {@code refresh()} method periodically while tuning (but not at official competition)
 */
public class ShuffleboardPIDController extends PIDController {
    // Shuffleboard Entries
    private GenericEntry entryP;
    private GenericEntry entryI;
    private GenericEntry entryD;
    private GenericPublisher entryReport;

    // Constructor
    /**
     * Creates a new ShuffleboardProfiledPIDController
     * @param tabName The name of the Shuffleboard tab to use
     * @param Kp Default P constant
     * @param Ki Default I constant
     * @param Kd Default D constant
     */
    public ShuffleboardPIDController(String tabName, double Kp, double Ki, double Kd) {
        // Parent Constructor
        super(Kp, Ki, Kd);

        // Add Shuffleboard Entries
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        entryP = tab.add("kP", Kp).getEntry();
        entryI = tab.add("kI", Ki).getEntry();
        entryD = tab.add("kD", Kd).getEntry();

        entryReport = tab.add("Current Constants", toString()).getEntry();
    }

    // Refresh
    /**
     * Refreshes the P, I, and D constants from Shuffleboard, and reports what values it is using (to verify constants
     * have set correctly).
     * <br><br>
     * <b>Note:</b> while tuning, run this function periodically.
     */
    public void refresh() {
        super.setPID(
            entryP.getDouble(getP()), 
            entryI.getDouble(getI()), 
            entryD.getDouble(getD())
        );

        entryReport.setString(toString());
    }

    // To String
    @Override
    public String toString() { return String.format("P: %s; I: %s; D: %s;", getP(), getI(), getD()); }
}
