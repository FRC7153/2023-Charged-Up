package com.frc7153.math;

import java.util.function.Supplier;

public class Encoder {
    /** Ranges that can be used to wrap the output*/
    public static enum Range {FROM_0_TO_360, FROM_NEGATIVE_180_TO_180, FROM_0_TO_1, INFINITE};

    // Input
    private Supplier<Double> encSupply;
    
    // Config Values
    private boolean inverted = false;
    private double zeroOffset = 0.0;
    private double conversion = 1.0;
    private Range range = Range.INFINITE;

    // Constructor
    public Encoder(Supplier<Double> encSupplier) {
        encSupply = encSupplier;
    }

    // Configure
    /**
     * Sets whether the output is inverted
     * @param inverted
     */
    public void setInverted(boolean inverted) { this.inverted = inverted; }

    /**
     * Sets the zero-position (offset) of the encoder. 
     * This is in the native units, not affected by the conversion factor.
     * @param offset
     */
    public void setZeroOffset(double offset) { zeroOffset = offset; }

    /**
     * Sets the conversion factor of the encoder
     * @param conversion
     */
    public void setConversionFactor(double conversion) { this.conversion = conversion; }

    /**
     * Sets the range of the encoder. The output is forced to this value after other math is done.
     * @param range
     */
    public void setRange(Range range) { this.range = range; }

    // Getters
    /**
     * Gets the position of the encoder.
     * @param altered Whether the inversion, offset, conversion, or continuous range should be accounted for.
     * @return Position
     */
    public double getPosition(boolean alter) {
        if (alter) {
            double angle = (encSupply.get() - zeroOffset) * (inverted ? -1 : 1) * conversion;

            switch (range) {
                case FROM_0_TO_1:
                    return MathUtils.wrap0To1(angle);
                case FROM_0_TO_360:
                    return MathUtils.normalizeAngle360(angle);
                case FROM_NEGATIVE_180_TO_180:
                    return MathUtils.normalizeAngle180(angle);
                case INFINITE:
                    return angle;
            }

            return angle;
        } else {
            return encSupply.get();
        }
    }

    /**
     * Gets the absolute position of the encoder, accounting for the inversion, offset, conversion, and continuous range.
     * @return Position (probably in degrees of continuous range is set)
     */
    public double getPosition() { return getPosition(true); }
}
