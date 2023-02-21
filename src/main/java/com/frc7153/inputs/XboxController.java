package com.frc7153.inputs;

import com.frc7153.inputs.util.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Custom Xbox Controller implementation that adds deadband and offset functionality
 * (as well as some other features)
 */
public class XboxController extends edu.wpi.first.wpilibj.XboxController {
    // Config
    private Translation2d lOffset;
    private Translation2d rOffset;
    private double joystickDeadband;
    private double triggerDeadband;
    private OffsetMode mode;
    private double triggerThresh;

    // Last offset update
    private double lastOffsetUpdate = 0.0;

    /**
     * Creates a new Xbox Controller object
     * @param port the USB port of the xbox controller
     * @param joystickDeadband The deadband applied to the left and right joystick
     * @param triggerDeadband The deadband applied to the left and right triggers
     * @param triggerThreshold How far the trigger must be pushed down to be considered down (true)
     * @param offsetMode How the offset is handled
     */
    public XboxController(int port, double joystickDeadband, double triggerDeadband, double triggerThreshold, OffsetMode offsetMode) {
        super(port);

        this.joystickDeadband = joystickDeadband;
        this.triggerDeadband = triggerDeadband;
        triggerThresh = triggerThreshold;
        mode = offsetMode;

        calibrateOffset();
    }

    /**
     * Creates a new Xbox Controller object
     * @param port The USB port of the joystick
     */
    public XboxController(int port) { this(port, 0.08, 0.08,  0.5, OffsetMode.INTERPOLATE); }

    /**
     * Determines the offset of the Xbox Controller's joysticks (for if it has been bent or damaged after years of usage)
     * <br><br>
     * When this runs, there should be <b>no force applied</b> to the joystick (it should be free to move
     * to its 0, 0 position)
     */
    public void calibrateOffset() {
        lOffset = new Translation2d(
            super.getRawAxis(Axis.kLeftX.value),
            super.getRawAxis(Axis.kLeftY.value)
        );

        rOffset = new Translation2d(
            super.getRawAxis(Axis.kRightX.value),
            super.getRawAxis(Axis.kRightY.value)
        );

        lastOffsetUpdate = Timer.getFPGATimestamp();
    }

    /**
     * @return The last time calibrateOffset() was called, in seconds
     */
    public double getLastOffsetUpdate() { return lastOffsetUpdate; }

    /**
     * Sets the deadband
     * @param joystick the deadband applied to the joystick
     * @param trigger the deadband applied to the trigger
     */
    public void setDeadband(double joystick, double trigger) {
        joystickDeadband = joystick;
        triggerDeadband = trigger;
    }

    /**
     * Sets the threshold value required for the trigger to be considered down
     * @param threshold
     */
    public void setTriggerThreshold(double threshold) { triggerThresh = threshold; }

    /**
     * Sets how offsets are handled
     * @param mode
     */
    public void setOffsetMode(OffsetMode mode) { this.mode = mode; }

    // Get Axis
    /**
     * Gets the axis, applying the deadband and offset
     * @param axis
     * @return The adjusted input
     */
    @Override
    public double getRawAxis(int axis) {
        double value = super.getRawAxis(axis);

        // Apply Offset
        double offset = 0.0;

        if (axis == Axis.kLeftX.value || axis == Axis.kLeftY.value) {
            offset = (axis == Axis.kLeftX.value) ? lOffset.getX() : lOffset.getY();
        } else if (axis == Axis.kRightX.value || axis == Axis.kRightY.value) {
            offset = (axis == Axis.kRightX.value) ? rOffset.getX() : rOffset.getY();
        }

        switch (mode) {
            case INTERPOLATE:
                value = ControllerMathUtil.interpolateRange(value, offset);
                break;
            case TRANSlATE:
                value = value - offset;
                break;
        }

        // Apply Deadband
        return ControllerMathUtil.clamp(
            ControllerMathUtil.applyDeadband(
                value,
                (axis == Axis.kLeftTrigger.value || axis == Axis.kRightTrigger.value) ? triggerDeadband : joystickDeadband
            )
        );
    }

    // Get Trigger As Boolean
    /**
     * Gets the right trigger value
     * @return true, if it is above the trigger threshold
     */
    public boolean getRightTrigger() {
        return getRightTriggerAxis() >= triggerThresh;
    }

    /**
     * Gets the left trigger value
     * @return true, if it is above the trigger threshold
     */
    public boolean getLeftTrigger() {
        return getLeftTriggerAxis() >= triggerThresh;
    }

    /**
     * A command to recalibrate the joysticks. This can be bound to a button, or put on Shuffleboard
     */
    public class CalibrateOffsetCommand extends CommandBase {
        @Override
        public void initialize() {
            calibrateOffset();
            System.out.println(String.format("Xbox Controller %s calibrated", getPort()));
        }

        @Override
        public boolean isFinished() { return true; }

        @Override
        public boolean runsWhenDisabled() { return true; }

        @Override
        public String getName() { return String.format("Recalibrate Xbox Controller %s", getPort()); }
    }
}
