package com.frc7153.math;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// TODO javadocs

public class ShuffleboardProfiledPIDController extends ProfiledPIDController {
    // Shuffleboard Entries
    private GenericEntry entryP;
    private GenericEntry entryI;
    private GenericEntry entryD;
    private GenericPublisher entryReport;

    // Constructor
    public ShuffleboardProfiledPIDController(String tabName, double Kp, double Ki, double Kd, Constraints constraints) {
        // Parent Constructor
        super(Kp, Ki, Kd, constraints);

        // Add Shuffleboard Entries
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        entryP = tab.add("kP", Kp).getEntry();
        entryI = tab.add("kI", Ki).getEntry();
        entryD = tab.add("kD", Kd).getEntry();

        entryReport = tab.add("Current Constants", toString()).getEntry();
    }

    // Refresh
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
