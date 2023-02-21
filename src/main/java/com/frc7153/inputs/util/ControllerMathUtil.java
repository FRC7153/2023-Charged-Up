package com.frc7153.inputs.util;

public class ControllerMathUtil {
    /**
     * Any input {@code value} between {@code deadband} and negative {@code deadband}
     * will be rounded down/up to 0.0. Anything outside that range is not modified.
     * @param value (Input value)
     * @param deadband
     * @return The value with the deadband applied.
     */
    public static double applyDeadband(double value, double deadband) {
        return (Math.abs(value) < deadband) ? 0.0 : value;
    }

    /**
     * Translates a range to center around {@code offset}, while scaling both sides.
     * <br><br>
     * Examples:
     * <ul>
     * <li>if value is 0.6 and offset is 0.2, output is 0.5</li>
     * <li>if value is -0.4 and offset is 0.1, output is -0.45</li>
     * <li>if value is 0.3 and offset is 0.3, output is 0.0</li>
     * </ul>
     * <br><br>Assumes scale ranges from and to [-1.0, 1.0]
     * @param value
     * @param offset
     * @return Scaled value
     */
    public static double interpolateRange(double value, double offset) {
        if (value > offset) {
            return (value - offset) / (1.0 - offset);
        } else if (value < offset) {
            return ((value + 1.0) / (offset + 1.0)) * 1.0 - 1.0;
        } else {
            return 0.0;
        }
    }

    /**
     * Clamps number to range of -1.0 to 1.0
     * @param input
     * @return clamped input
     */
    public static double clamp(double value) { return Math.min(Math.max(value, -1.0), 1.0); }
}
