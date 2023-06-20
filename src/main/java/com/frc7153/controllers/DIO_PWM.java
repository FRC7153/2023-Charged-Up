package com.frc7153.controllers;

import com.frc7153.math.MathUtils;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Run PWM signal over RoboRIO's PWM ports.
 * This should only be used to control LEDs
 */
public class DIO_PWM {
    // Conversion
    private static double kMS_TO_NANO = 1000000.0; // ms to nanoseconds

    // DIO Object
    private DigitalOutput output;

    // Thread
    private Thread generatorThread;

    // Config (all units in ms)
    private double maxPW = 2.0; // Max pulse width
    private double maxDB_PW = 1.99; // Max deadband pulse width
    private double center = 1.5; // center
    private double minPW = 1.0; // Min pulse width
    private double minDB_PW = 1.01; // Min deadband pulse width
    private static long kPERIOD = (long)(5.005 * kMS_TO_NANO); // PWM pulses occur at this frequency in nanoseconds

    // State (nano seconds)
    private long targetPW = (long)(center * kMS_TO_NANO);

    /**
     * Create PWM generator on DIO pin
     * @param channel DIO channel # (0 - 9)
     */
    public DIO_PWM(int channel) {
        output = new DigitalOutput(channel);
        start();
    }

    /**
     * Create PWM Generator on DIO pin and set config
     * @param channel DIO channel # (0 - 9)
     * @param maxPulseWidth The max PWM pulse width in ms
     * @param maxDeadbandPulseWidth The high end of the deadband range pulse width in ms
     * @param centerPulseWidth The center (off) pulse width in ms
     * @param minPulseWidth The low end of the deadband pulse width in ms
     * @param minDeadbandPulseWidth The minimum pulse width in ms
     */
    public DIO_PWM(int channel, double maxPulseWidth, double maxDeadbandPulseWidth, double centerPulseWidth, double minPulseWidth, double minDeadbandPulseWidth) {
        output = new DigitalOutput(channel);

        maxPW = maxPulseWidth; maxDB_PW = maxDeadbandPulseWidth; center = centerPulseWidth; minPW = minPulseWidth; minDB_PW = minDeadbandPulseWidth;
        setPulseWidth(centerPulseWidth);

        start();
    }

    /**
     * Set pulse width by percentage
     * @param percent -1.0 to 1.0
     */
    public void setPercent(double percent) {
        percent = MathUtils.symmetricClamp(percent, 1.0);

        if (percent > 0.0) {
            setPulseWidth(center + (percent * (maxDB_PW - center)));
        } else {
            setPulseWidth(center - (percent * (minDB_PW - center)));
        }
    }

    /**
     * Manually set pulse width
     * @param pulseWidth Pulse width in ms
     */
    public void setPulseWidth(double pulseWidth) {
        pulseWidth = Math.min(Math.max(pulseWidth, maxPW), minPW);
        pulseWidth *= kMS_TO_NANO; // To nano seconds
        
        synchronized (this) {
            targetPW = (long)pulseWidth;
        }
    }

    // Generator function to bit bang PWM signal. Should be called in a separate thread.
    private void generate() {
        // Init values (nano seconds)
        long pulseHighWidth;
        long pulseLowWidth;
        long releaseTime;

        synchronized (this) {
            pulseHighWidth = targetPW;
            pulseLowWidth = kPERIOD - targetPW;
        }

        // Loop
        while (true) {
            // High
            output.set(true);
            releaseTime = System.nanoTime() + pulseHighWidth;

            if (pulseHighWidth > pulseLowWidth) {
                // This busy-wait is longer, do work
                synchronized (this) {
                    pulseHighWidth = targetPW;
                    pulseLowWidth = kPERIOD - targetPW;
                }

                if (Thread.interrupted()) { break; }
            }

            // Busy-wait
            while (releaseTime > System.nanoTime()) { ; }

            // Low
            output.set(false);
            releaseTime = System.nanoTime() + pulseLowWidth;

            if (pulseLowWidth >= pulseHighWidth) {
                // This busy-wait is longer, do work
                synchronized (this) {
                    pulseHighWidth = targetPW;
                    pulseLowWidth = kPERIOD - targetPW;
                }

                if (Thread.interrupted()) { break; }
            }

            // Busy-wait
            while (releaseTime > System.nanoTime()) { ; }
        }

        // Thread was interrupted
        output.set(false);
    }

    // Thread to generate PWM signal
    private static class GeneratorThread implements Runnable {
        private DIO_PWM dio;

        public GeneratorThread(DIO_PWM dio) {
            this.dio = dio;
        }

        @Override
        public void run() {
            dio.generate();
        }
    }

    /**
     * Start PWM generation. Called by default
     */
    public void start() {
        if (generatorThread != null && generatorThread.isAlive()) {
            DriverStation.reportError("Could not start PWM generator thread because it is already running!", false);
            return;
        }

        generatorThread = new Thread(new GeneratorThread(this));
        generatorThread.start();
    }

    /**
     * Stop PWM generation
     */
    public void stop() {
        if (generatorThread == null || !generatorThread.isAlive()) {
            DriverStation.reportError("Could not stop PWM generator thread because it is not running!", false);
            return;
        }

        generatorThread.interrupt();
    }
}
