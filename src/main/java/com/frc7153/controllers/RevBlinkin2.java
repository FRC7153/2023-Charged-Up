package com.frc7153.controllers;

import edu.wpi.first.math.MathUtil;

/**
 * Class for controlling REV's Blinking LED Driver over PWM
 */
public class RevBlinkin2 {
    // Preset Configurations
    /**
     * REV Blinkin's preset supported colors
     */
    public static enum BlinkinSolidColor {
        HOT_PINK(10.0), DARK_RED(0.59), RED(0.61), RED_ORANGE(0.63), ORANGE(0.65), GOLD(0.67), 
        YELLOW(0.69), LAWN_GREEN(0.71), LIME(0.73), DARK_GREEN(0.75), GREEN(0.77), BLUE_GREEN(0.79), 
        AQUA(0.81), SKY_BLUE(0.83), DARK_BLUE(0.85), BLUE(0.87), BLUE_VIOLET(0.89), VIOLET(0.91), 
        WHITE(0.93), GRAY(0.95), DARK_GRAY(0.97), BLACK(0.99);

        /**
         * The PWM signal to send for this color
         */
        public final double signal;
        BlinkinSolidColor(double signal) { this.signal = signal; }
    }

    /**
     * REV Blinkin's supported single-color patterns.
     * Some of these are for the 1m 5v addressable light strips only
     */
    public static enum BlinkinPattern {
        END_TO_END_BLEND_TO_BLACK(0), HEARTBEAT_SLOW(3), HEARTBEAT_MEDIUM(4), HEARTBEAT_FAST(5),
        BREATH_SLOW(6), BREATH_FAST(7), SHOT(8), STROBE(9),

        /**
         * <ul>
         * <li>Adjustment 1: Pattern width</li>
         * <li>Adjustment 2: Speed</li>
         * </ul>
         */
        LARSON_SCANNER(1),

        /**
         * <ul>
         * <li>Adjustment 1: Dimming</li>
         * <li>Adjustment 2: Speed</li>
         * </ul>
         */
        LIGHT_CHASE(2);

        /**
         * The index of this pattern. The signal is determine by: <br><br>
         * -0.03 + (index * 0.2) + ((color == 1) ? 0.0 : 0.2)
         */
        public final int index;
        BlinkinPattern(int index) { this.index = index;}
    }

    /**
     * REV Blinkin's supported multicolor patterns (uses both preset color 1 and 2).
     * Some of these are for the 1m 5v addressable light strip only
     */
    public static enum BlinkinMulticolorPattern {
        SPARKLE_1_ON_2(0.37), SPARKLE_2_ON_1(0.39), GRADIENT(0.41), END_TO_END_1_TO_2(0.45),
        END_TO_END(0.47), NO_BLEND_1_TO_2(0.49), TWINKLES(0.51), WAVES(0.53),

        /**
         * <ul>
         * <li>Adjustment 1: Pattern Density</li>
         * <li>Adjustment 2: Speed</li>
         * </ul>
         */
        BEATS_PER_MINUTE(0.43),

        /**
         * <ul>
         * <li>Adjustment 1: Pattern Density</li>
         * <li>Adjustment 2: Speed</li>
         * </ul>
         */
        SINELON(0.55);

        /**
         * The PWM signal to send for this pattern
         */
        public final double signal;
        BlinkinMulticolorPattern(double signal) { this.signal = signal; }
    }

    // PWM Controller
    private DIO_PWM pwm;
    
    /**
     * Creates a new PWM Rev Blinkin LED controller
     * @param pwmChannel
     */
    public RevBlinkin2(int pwmChannel) {
        pwm = new DIO_PWM(pwmChannel, 2.003, 1.55, 1.5, 1.46, 0.999);
    }

    /**
     * Set the PWM signal manually
     * @param signal -1.0 to 1.0
     */
    public void setSignal(double signal) { pwm.setPercent(signal); }

    /**
     * Set the color of the LED strip
     * @param color
     */
    public void setConstantColor(BlinkinSolidColor color) { pwm.setPercent(color.signal); }

    /**
     * Sets the color pattern, using the preset color.
     * Some of these may only work for the 1m 5v addressable LED strips.
     * @param pattern
     * @param color The preset color to use, either 1 or 2
     */
    public void setColorPattern(BlinkinPattern pattern, int color) {
        pwm.setPercent(-0.03 + (0.2 * pattern.index) + ((color == 1) ? 0.0 : 0.2));
    }

    /**
     * Sets a multicolor pattern, using the preset colors.
     * Some of these may only work for the 1m 5v addressable LED strips.
     * @param pattern
     */
    public void setMulticolorPattern(BlinkinMulticolorPattern pattern) { pwm.setPercent(pattern.signal); }

    /**
     * Set hue. This math may have a bug.
     * @param hue (0 - 360 degrees)
     */
    public void setHue(int hue) {
        hue = MathUtil.clamp(hue, 0, 360);
        if (hue > 300) {
            pwm.setPercent(((hue - 300.0) / 60.0) * 0.04 + 0.57);
        } else {
            pwm.setPercent((hue / 300.0) * 0.3 + 0.61);
        }
    }

    /**
     * Set the value to a grayscale color (black to white)
     * @param lightness (1.0 is white, 0.0 is black/off)
     */
    public void setGrayscale(double lightness) {
        lightness = -MathUtil.clamp(lightness, 0.0, 1.0) + 1.0;
        pwm.setPercent(0.06 * lightness + 0.93);
    }
}
