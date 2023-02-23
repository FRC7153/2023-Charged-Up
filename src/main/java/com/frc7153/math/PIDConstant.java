package com.frc7153.math;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.SparkMaxPIDController;

/**
 * PID values that can be applied to a number of controllers.
 */
public class PIDConstant {
    // Constants
    public final int kSLOT;

    public final double kP, kI, kD, kERR;

    public Double kFF = Double.NaN;
    public Double kOUTPUT_MIN = Double.NaN;
    public Double kOUTPUT_MAX = Double.NaN;

    // Constructors
    public PIDConstant(int slot, double p, double i, double d, double error) { kSLOT = slot; kP = p; kI = i; kD = d; kERR = error; }
    public PIDConstant(int slot, double p, double i, double d, double error, double ff) { this(slot, p, i, d, error); kFF = ff; }
    public PIDConstant(int slot, double p, double i, double d, double error, double min, double max) { this(slot, p, i, d, error); kOUTPUT_MIN = min; kOUTPUT_MAX = max; }

    // Apply
    /**
     * Apply these values to a Spark Max's PID controller
     * @param pid Spark Max PID Controller
     */
    public void apply(SparkMaxPIDController pid) {
        pid.setP(kP, kSLOT);
        pid.setI(kP, kSLOT);
        pid.setD(kP, kSLOT);

        pid.setSmartMotionAllowedClosedLoopError(kERR, kSLOT);

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

        controller.configAllowableClosedloopError(kSLOT, kERR);

        if (!kFF.isNaN()) { controller.config_kF(kSLOT, kFF); }
    }
}
