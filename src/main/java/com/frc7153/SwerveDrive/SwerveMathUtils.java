package com.frc7153.SwerveDrive;

public class SwerveMathUtils {
    /**
     * Takes an angle and normalizes it to a range of -180 to 180
     * @param angle
     * @return The angle, normalized
     */
    public static double normalizeAngle180(double angle) {
        angle = normalizeAngle360(angle);
        if (angle > 180) { angle -= 360.0; }
        return angle;
    }

    /**
     * Takes an angle and normalizes it to a range of 0 - 360
     * @param angle
     * @return The angle, normalizes
     */
    public static double normalizeAngle360(double angle) {
        return angle - (360.0 * Math.floor(angle / 360.0));
    }

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
     * Clamps a speed to the specified range, assuming the lower bound is the opposite of the upper bound
     * @param value The input value
     * @param clamp The upper bound, and opposite of the lower bound
     * @return The clamped value
     */
    public static double symmetricClamp(double value, double clamp) {
        if (clamp < 0) { clamp = -clamp;}
        return Math.min(Math.max(value, -clamp), clamp);
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