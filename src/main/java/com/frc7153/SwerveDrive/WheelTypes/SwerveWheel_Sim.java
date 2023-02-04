package com.frc7153.SwerveDrive.WheelTypes;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveWheel_Sim implements SwerveWheel {
    // States
    private double currentAngle;
    private double currentSpeed;
    private boolean coast;
    private int updates = 0;

    // Config
    private boolean configDriveInvert = false;
    private boolean configSpinInvert = false;

    // Position
    private Translation2d pos;

    // Shuffleboard
    private GenericPublisher shuffle_speed;
    private GenericPublisher shuffle_angle;
    private GenericPublisher shuffle_updates;
    private GenericPublisher shuffle_driveInvert;
    private GenericPublisher shuffle_angleInvert;
    private GenericPublisher shuffle_coast;

    // Constructor
    /**
     * Creates a new simulated serve wheel. 
     * @param shuffleboardTabName Name of tab to output to in Shuffleboard
     * @param shuffleboardColumn Column in Shuffleboard to use (should be different for each wheel, starting with 1)
     * @param x The x position of the wheel
     * @param y The y position of the wheel
     */
    public SwerveWheel_Sim(String shuffleboardTabName, int shuffleboardColumn, double x, double y) {
        pos = new Translation2d(y, -x);

        ShuffleboardTab tab = Shuffleboard.getTab(shuffleboardTabName);
        ShuffleboardLayout column = tab.getLayout(String.format("Wheel %s", shuffleboardColumn), BuiltInLayouts.kList)
            .withPosition(shuffleboardColumn, 0)
            .withSize(1, 4)
            .withProperties(Map.of("Label position", "TOP"));
        
        shuffle_speed = column.add("Speed", 0.0)
            .getEntry();
        
        shuffle_angle = column.add("Angle", 0.0)
            .getEntry();
        
        shuffle_updates = column.add("Number of Updates", 0)
            .getEntry();
        
        shuffle_driveInvert = column.add("Drive Inverted", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
        
        shuffle_angleInvert = column.add("Angle Inverted", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
        
        shuffle_coast = column.add("Coasting?", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
        
        periodic();
    }

    // Position
    @Override
    public Translation2d getPosition() { return pos; }

    // Get
    @Override
    public SwerveModulePosition getState() {
        return new SwerveModulePosition(currentSpeed, Rotation2d.fromDegrees(currentAngle));
    }

    // Set
    @Override
    public void setAngle(double angle) { currentAngle = angle; }

    @Override
    public void setSpeed(double speed) { currentSpeed = speed; }

    @Override
    public void set(SwerveModuleState state) {
        set(state.angle.getDegrees(), state.speedMetersPerSecond);
    }

    @Override
    public void toggleCoastMode(boolean coast) {
        this.coast = coast;
    }

    // Periodic
    @Override
    public void periodic() {
        updates ++;

        shuffle_speed.setDouble(currentSpeed);
        shuffle_angle.setDouble(currentAngle);
        shuffle_updates.setInteger(updates);
        shuffle_driveInvert.setBoolean(configDriveInvert);
        shuffle_angleInvert.setBoolean(configSpinInvert);
        shuffle_coast.setBoolean(coast);
    }
}
