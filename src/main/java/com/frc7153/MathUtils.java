package com.frc7153;

/**
 * Math functions that are commonly repeated
 */
public class MathUtils {
    /**
     * Clamps the {@code input} between {@code min} and {@code max}
     * @param input
     * @param min
     * @param max
     * @return Clamped value
     */
    public static double clamp(double input, double min, double max) {
        return Math.min(Math.max(input, min), max);
    }
}
