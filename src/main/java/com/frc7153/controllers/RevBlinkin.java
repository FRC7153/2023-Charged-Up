package com.frc7153.controllers;

import com.frc7153.MathUtils;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * Class for controlling REV's Blinking LED Driver over PWM
 */
public class RevBlinkin {
    // Solid Colors
    /**
     * REV Blinkin's supported colors
     */
    public static enum BlinkinSolidColors {
        HOT_PINK(10.0), DARK_RED(0.59), RED(0.61), RED_ORANGE(0.63), ORANGE(0.65), GOLD(0.67), 
        YELLOW(0.69), LAWN_GREEN(0.71), LIME(0.73), DARK_GREEN(0.75), GREEN(0.77), BLUE_GREEN(0.79), 
        AQUA(0.81), SKY_BLUE(0.83), DARK_BLUE(0.85), BLUE(0.87), BLUE_VIOLET(0.89), VIOLET(0.91), 
        WHITE(0.93), GRAY(0.95), DARK_GRAY(0.97), BLACK(0.99);

        /**
         * The PWM signal to send for this color
         */
        public final double signal;

        BlinkinSolidColors(double signal) { this.signal = signal; }
    }

    // PWM Controller
    private PWMSparkMax spark;
    
    /**
     * Creates a new PWM Rev Blinkin LED controller
     * @param pwmChannel
     */
    public RevBlinkin(int pwmChannel) {
        spark = new PWMSparkMax(pwmChannel);
    }

    /**
     * Set the color of the LED strip
     * @param color
     */
    public void setConstantColor(BlinkinSolidColors color) {
        spark.set(color.signal);
    }

    /**
     * Set hue
     * @param hue (0 - 360 degrees)
     */
    public void setHue(int hue) {

    }

    /**
     * Set the value to a grayscale color (black to white)
     * @param lightness (1.0 is white, 0.0 is black/off)
     */
    public void setGrayscale(double lightness) {
        lightness = MathUtils.clamp(lightness, 0.0, 1.0);
    }
}
