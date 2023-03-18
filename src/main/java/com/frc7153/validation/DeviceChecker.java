package com.frc7153.validation;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Checks devices for validation
 */
public class DeviceChecker {
    /**
     * Validates a CAN Spark Max (does NOT check encoders)
     * @param spark CAN Spark Max speed controller
     * @return Whether there is an error
     */
    public static boolean validateMotor(CANSparkMax spark) { return spark.getFault(FaultID.kCANRX) || spark.getFault(FaultID.kCANTX) || spark.getFault(FaultID.kMotorFault); }

    /**
     * Validates a Talon FX (Falcon 500)
     * @param talon Talon FX speed controller
     * @return Whether there is an error
     */
    public static boolean validateMotor(TalonFX talon) {
        Faults faults = new Faults();
        talon.getFaults(faults);
        return faults.hasAnyFault();
    }

    /**
     * Validates a CAN Spark Max's encoder(s) (does NOT check motor)
     * @param spark CAN Spark Max speed controller
     * @return Whether there is an error
     */
    public static boolean validateEncoder(CANSparkMax spark) { return spark.getFault(FaultID.kSensorFault); }

    /**
     * Validates a Power Distribution Hub (CAN and hardware)
     * @param power Power Distribution Hub (REV or CTRE)
     * @return Whether there is an error
     */
    public static boolean validatePowerDistribution(PowerDistribution power) {
        PowerDistributionFaults faults = power.getFaults();
        return faults.CanWarning || faults.HardwareFault;
    }

    /**
     * Validates a Power Distribution Hub's channels
     * @param power Power Distribution Hub (REV or CTRE)
     * @param channelsToCheck List of channels to check (0 - 23)
     * @return Whether there is an error
     */
    public static boolean validatePowerDistributionChannels(PowerDistribution power, int[] channelsToCheck) {
        PowerDistributionFaults faults = power.getFaults();

        for (int channel : channelsToCheck) {
            try {
                if ((boolean)PowerDistributionFaults.class.getField(String.format("Channel%sBreakerFault", channel)).get(faults)) {
                    return true;
                }
            } catch (Exception e) {
                DriverStation.reportError(String.format("Could not check Power Distribution channel %s for faults: %s", channel, e), true);
            }
        }
        
        return false;
    }
}
