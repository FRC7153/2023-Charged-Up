package com.frc7153.Controllers;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Custom Joystick implementation that adds deadband and offset functionality
 * (as well as some other features)
 */
public class Joystick extends edu.wpi.first.wpilibj.Joystick {
    // Config
    private Translation3d offset;
    private double joystickDeadband;
    private double throttleDeadband;
    private boolean throttleInvert;
    private OffsetMode mode;

    /**
     * Creates a new Joystick object
     * @param port The USB port of the joystick
     * @param joystickDeadband The deadband applied to the joystick (any value between this and negative this is rounded down to zero)
     * @param throttleDeadband The deadband applied to the throttle
     * @param invertThrottle Whether the throttle (slider at bottom of the joystick) is inverted
     * @param offsetMode How the offset is handled
     */
    public Joystick(int port, double joystickDeadband, double throttleDeadband, boolean invertThrottle, OffsetMode offsetMode) {
        super(port);

        this.joystickDeadband = joystickDeadband;
        this.throttleDeadband = throttleDeadband;
        throttleInvert = invertThrottle;
        mode = offsetMode;

        calibrateOffset();
    }

    /**
     * Creates a new Joystick object
     * @param port The USB port of the joystick
     */
    public Joystick(int port) { this(port, 0.08, 0.05, true, OffsetMode.INTERPOLATE); }

    /**
     * Determines the offset of the joystick (for if it has been bent or damaged after years of usage)
     * <br><br>
     * When this runs, there should be <b>no force applied</b> to the joystick (it should be free to move
     * to its 0, 0 position)
     */
    public void calibrateOffset() {
        offset = new Translation3d(
            super.getRawAxis(getXChannel()), 
            super.getRawAxis(getYChannel()), 
            super.getRawAxis(getTwistChannel())
        );
    }

    /**
     * Sets the deadband.
     * @param joystick The deadband applied to the joystick (all axes)
     * @param throttle the deadband applied to the throttle (all axes)
     */
    public void setDeadband(double joystick, double throttle) {
        joystickDeadband = joystick;
        throttleDeadband = throttle;
    }

    /**
     * Inverts the throttle
     * @param invert
     */
    public void setThrottleInverted(boolean invert) { throttleInvert = invert; }

    /**
     * Sets how offsets are handled
     * @param mode
     */
    public void setOffsetMode(OffsetMode mode) { this.mode = mode; }

    // Get Axis
    /**
     * Gets the axis, applying any deadband, inversion, or offset needed.
     * @param axis
     * @return The adjusted input
     */
    @Override
    public double getRawAxis(int axis) {
        double value = super.getRawAxis(axis);

        if (axis == getThrottleChannel()) {
            // Throttle Invert
            if (throttleInvert) { value *= -1; }
        } else {
            // Joystick Offset
            double axisOffset = (axis == getXChannel()) ? offset.getX() : ((axis == getYChannel()) ? offset.getY() : offset.getZ());

            switch (mode) {
                case INTERPOLATE:
                    value = ControllerMathUtil.interpolateRange(value, axisOffset);
                    break;
                case TRANSlATE:
                    value = value - axisOffset;
                    break;
            }
        }

        // Apply Deadband
        return ControllerMathUtil.clamp(
            ControllerMathUtil.applyDeadband(
                value, 
                (axis == getThrottleChannel()) ? throttleDeadband : joystickDeadband
            )
        );
    }

    /**
     * A command to recalibrate the joystick. This can be bound to a button, or put on Shuffleboard.
     */
    public class CalibrateOffsetCommand extends CommandBase {
        private boolean running = false;

        @Override
        public void initialize() {
            running = true;
            calibrateOffset();
            System.out.println(String.format("Joystick %s calibrated", getPort()));
            running = false;
        }

        @Override
        public boolean isFinished() { return !running; }

        @Override
        public boolean runsWhenDisabled() { return true; }

        @Override
        public String getName() { return String.format("Recalibrate joystick %s", getPort()); }
    }
}
