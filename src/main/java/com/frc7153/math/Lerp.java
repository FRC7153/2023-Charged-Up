package com.frc7153.math;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Linear interpolation between two values over a specific duration
 */
public class Lerp {
    /**
     * How this should handle when the timestamp exceeds the duration.<br><br>
     * All examples are done with a start of 0 and and end of 3.
     * <ul>
     *  <li><b>CONTINUE: </b>Continue at same slope. Example: 0, 1, 2, 3, 4, 5...</li>
     * <li><b>FLAT_LINE: </b>Stops at endpoint. Example: 0, 1, 2, 3, 3, 3...</li>
     * <li><b>REPEAT: </b>Repeats from beginning. Example: 0, 1, 2, 3, 0, 1...</li>
     * <li><b>BOUNCE: </b>Inverts the slope to return to the start point. Example: 0, 1, 2, 3, 2, 1...</li>
     * </ul>
     */
    public static enum OverflowBehavior {CONTINUE, FLAT_LINE, REPEAT, BOUNCE};

    // Config
    private double startPt, endPt, duration;
    private OverflowBehavior overflow = OverflowBehavior.FLAT_LINE;

    // Run
    private Double start = Double.NaN;

    // Constructor
    /**
     * Creates a  new linear interpolation.
     * @param start The start value
     * @param end The end value
     * @param duration The length of the interpolation (seconds)
     */
    public Lerp(double start, double end, double duration) {
        startPt = start;
        endPt = end;
        this.duration = duration;
    }

    // Start
    /**
     * Starts (or restarts) the timer.
     */
    public void start() {
        start = Timer.getFPGATimestamp();
    }

    // Sample
    private double localSample(double stamp) { return startPt + ((endPt / duration) * stamp); }

    /**
     * Gets the value at the current timestamp. Uses OverflowBehavior to calculate value when its past the duration.
     * @return
     */
    public double sample() {
        if (!start.isNaN()) {
            DriverStation.reportError("Could not sample interpolation, was never started!", false);
            return startPt;
        }

        double stamp = Timer.getFPGATimestamp() - start;

        if (stamp > duration) {
            double localized = stamp - (Math.floor(stamp/duration)*duration);

            switch (overflow) {
                case BOUNCE:
                    if (Math.floor(stamp/duration) % 2 == 0) {
                        // Rising edge
                        return localSample(localized);
                    } else {
                        // Falling edge
                        return localSample(localized + (2 * ((duration/2.0) - localized)));
                    }
                case CONTINUE:
                    return localSample(stamp);
                case FLAT_LINE:
                    return endPt;
                case REPEAT:
                    return localSample(localized);
            }

            return 0.0;
        } else {
            return localSample(stamp);
        }
    }

    /**
     * Returns whether the interpolation is finished.
     * @return
     */
    public boolean isFinished() {
        return !start.isNaN() && (Timer.getFPGATimestamp() - start > duration);
    }
}
