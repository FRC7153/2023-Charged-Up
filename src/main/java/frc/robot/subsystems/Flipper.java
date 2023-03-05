// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlipperConstants;

public class Flipper extends SubsystemBase {
    // Motor, encoder PID
    private CANSparkMax flipperMotor = new CANSparkMax(20, MotorType.kBrushless);
    private RelativeEncoder flipperRelEnc = flipperMotor.getEncoder();
    private DutyCycleEncoder flipperAbsEnc = new DutyCycleEncoder(9);
    //private PIDController flipperPID = flipperMotor.getPIDController();

    // Init motors
    public Flipper() {
        flipperRelEnc.setPosition(flipperAbsEnc.getAbsolutePosition() * FlipperConstants.kGEAR_RATIO);
    }

    // Set position
    public void setAngle(double angle) {
        if (angle < 0.0 || angle > FlipperConstants.kMAX_ANGLE) {
            DriverStation.reportWarning(String.format("Angle of flipper set too high! Tried to set to %s", angle), false);
            angle = Math.min(Math.max(angle, 0.0), FlipperConstants.kMAX_ANGLE);
        }


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
