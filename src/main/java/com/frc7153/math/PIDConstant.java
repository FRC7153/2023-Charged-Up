package com.frc7153.math;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * PID values that can be applied to a number of controllers. This is made so that PID constants
 * can be easily defined with other constants in Constants.java (or other constant locations)
 */
public class PIDConstant {
    // Constants
    public int kSLOT = 0;

    public double kP, kI, kD;

    public Double kERR = Double.NaN;
    public Double kFF = Double.NaN;
    public Double kOUTPUT_MIN = Double.NaN;
    public Double kOUTPUT_MAX = Double.NaN;

    // Constructors
    /**
     * Create new PID Constant. Use modifier methods to further modify this in static constant classes.
     * @param p kP coefficient
     * @param i kI coefficient
     * @param d kD coefficient
     */
    public PIDConstant(double p, double i, double d) { kP = p; kI = i; kD = d; }

    // Modifiers
    /**
     * @param slot The slot to save these PID constants to
     * @return This object (modifications are done in place)
     */
    public PIDConstant withSlot(int slot) { kSLOT = slot; return this; }
    /**
     * @param err The acceptable error
     * @return This object (modifications are done in place)
     */
    public PIDConstant withError(double err) { kERR = err; return this; }
    /**
     * ({@code PIDController} does not support this)
     * @param ff The feed-forward gain (kF or kFF)
     * @return This object (modifications are done in place)
     */
    public PIDConstant withFF(double ff) { kFF = ff; return this; }
    /**
     * ({@code SparkMaxPIDController} only)
     * @param min Minimum output
     * @param max Maximum output
     * @return This object (modifications are done in place)
     */
    public PIDConstant withOutputRange(double min, double max) { kOUTPUT_MIN = min; kOUTPUT_MAX = max; return this; } 

    // Apply
    /**
     * Apply these values to a Spark Max's PID controller
     * @param pid Spark Max PID Controller
     */
    public void apply(SparkMaxPIDController pid) {
        pid.setP(kP, kSLOT);
        pid.setI(kI, kSLOT);
        pid.setD(kD, kSLOT);

        if (!kERR.isNaN()) { pid.setSmartMotionAllowedClosedLoopError(kERR, kSLOT); }
        if (!kFF.isNaN()) { pid.setFF(kFF); }
        if (!kOUTPUT_MIN.isNaN() && !kOUTPUT_MAX.isNaN()) { pid.setOutputRange(kOUTPUT_MIN, kOUTPUT_MAX); }
    }

    /**
     * Apply these values to a TalonFX controller (for a Falcon500)
     * @param controller TalonFX Controller
     */
    public void apply(TalonFX controller) {
        controller.config_kP(kSLOT, kP);
        controller.config_kI(kSLOT, kI);
        controller.config_kD(kSLOT, kD);

        if (!kERR.isNaN()) { controller.configAllowableClosedloopError(kSLOT, kERR); }
        if (!kFF.isNaN()) { controller.config_kF(kSLOT, kFF); }
    }

    /**
     * Apply these values to WPI's PID controller. This has no SLOTs.
     * @param controller WPI pid controller
     */
    public void apply(PIDController controller) {
        controller.setPID(kP, kI, kD);

        if (!kERR.isNaN()) { controller.setTolerance(kERR); }
    }

    /**
     * Creates a new WPI PID controller and returns it.
     * @return
     */
    public PIDController toWPIPidController() {
        PIDController pid = new PIDController(0.0, 0.0, 0.0);
        apply(pid);
        return pid;
    }

    /**
     * Creates a new WPI Profiled PID controller and returns it.
     * @return 
     */
    public ProfiledPIDController toWPIProfiledPidController(double maxVelocity, double maxAccel) {
        ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, new Constraints(maxVelocity, maxAccel));

        if (!kERR.isNaN()) { pid.setTolerance(kERR); }

        return pid;
    }
}
