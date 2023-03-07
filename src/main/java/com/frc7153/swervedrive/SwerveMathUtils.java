package com.frc7153.swervedrive;

public class SwerveMathUtils {
    /**
     * Determines how many times you have to add {@code gearRatio} to {@code setPoint} to get it as close
     * as possible to {@code currentPos} to ensure motor moves to least amount possible.<br><br>
     * Ex: If set-point is 5, gearing ratio is 360 (degrees), and it is currently at 4000, it will return 3965, 
     * which is equivalent to 5 (because degrees in a circle are continuos)
     * @param currentPos The current position
     * @param setPoint The target position
     * @param gearRatio Number of units needed to wrap all the way around (ie, degrees is 360, gears would be the gear ratio)
     * @return Number to move to that is basically (not numerically) equivalent to the {@code setPoint} but near the {@code currentPos}
     */
    public static double calculateContinuousMovement(double currentPos, double setPoint, double gearRatio) {
        //return Math.round(currentPos / gearRatio) * gearRatio + setPoint; // This line is questionable
        setPoint = setPoint / gearRatio;
        return (Math.round((currentPos/gearRatio) - setPoint) + setPoint) * gearRatio;
    }

    /**
     * Apply a deadband to the value
     * @param value
     * @param deadband
     * @return
     */
    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    /**
     * Convert RPMs to Falcon500's encoder velocity
     * @param rpm
     * @return Falcon500's encoder velocity
     */
    public static double rpmToFalcon500Velocity(double rpm) { return rpm * (2048.0 / 600.0); }

    /**
     * Convert Falcon500's position to rotations
     * @param Falcon500's encoder position
     * @return rotations
     */
    public static double falcon500PositionToRotations(double position) { return position / 2048.0; }
}