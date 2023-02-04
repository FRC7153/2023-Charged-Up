package com.frc7153.SwerveDrive.WheelTypes;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveWheel {
    // Configs //
    
    /**
     * Gets the position of the wheel, relative to the center of the robot
     * @return The position, in meters
     */
    public Translation2d getPosition();

    // Driving //

    /**
     * Gets the module state
     * @return
     */
    public SwerveModulePosition getState();

    /**
     * Sets the angle of the wheel, from -180 to 180, with 0 degrees the front of the robot. 
     * @param angle
     */
    public void setAngle(double angle);

    /**
     * Sets the speed of the wheel, in MPS
     * @param speed
     */
    public void setSpeed(double speed);

    /**
     * Sets the angle and speed of the wheel
     * @param angle degrees (0 - 360)
     * @param speed Meters per second
     */
    default public void set(double angle, double speed) {
        setAngle(angle);
        setSpeed(speed);
    };

    /**
     * Set the angle and speed of the wheel with a SwerveModuleState
     * @param state
     */
    public void set(SwerveModuleState state);

    /**
     * Enables/disables coast mode
     * @param coast
     */
    public void toggleCoastMode(boolean coast);

    /**
     * Needs to be called periodically to update PID loops
     */
    public void periodic();
}