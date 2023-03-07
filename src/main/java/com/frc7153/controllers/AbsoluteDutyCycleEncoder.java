package com.frc7153.controllers;

import com.frc7153.math.MathUtils;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Encoder class that wraps WPI's {@code DutyCycleEncoder} to add support for position offset, inversion, and conversion.
 */
public class AbsoluteDutyCycleEncoder extends DutyCycleEncoder {
    // Config Values
    public static enum Range {FROM_0_TO_360, FROM_NEGATIVE_180_TO_180, INFINITE};

    // Config
    private boolean inverted = false;
    private double zeroOffset = 0;
    private double conversion = 1.0;
    private Range range = Range.FROM_0_TO_360;

    // Constructors
    public AbsoluteDutyCycleEncoder(int channel) { super(channel); }
    public AbsoluteDutyCycleEncoder(DutyCycle cycle) { super(cycle); }
    public AbsoluteDutyCycleEncoder(DigitalSource source) { super(source); }

    // Setters
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
     * Gets the absolute position of the encoder.
     * @param altered Whether the inversion, offset, conversion, or continuous range should be accounted for.
     * @return Position
     */
    public double getAbsolutePosition(boolean altered) {
        if (altered) {
            double angle = (super.getAbsolutePosition() - zeroOffset * (inverted ? -1 : 1)) * conversion;

            switch (range) {
                case FROM_0_TO_360:
                    return MathUtils.normalizeAngle360(angle);
                case FROM_NEGATIVE_180_TO_180:
                    return MathUtils.normalizeAngle180(angle);
                default:
                    return angle;
            }
        } else {
            return super.getAbsolutePosition();
        }
    }

    /**
     * Gets the absolute position of the encoder, accounting for the inversion, offset, conversion, and continuous range.
     * @return Position (probably in degrees of continuous range is set)
     */
    @Override
    public double getAbsolutePosition() { return getAbsolutePosition(true); }
}
