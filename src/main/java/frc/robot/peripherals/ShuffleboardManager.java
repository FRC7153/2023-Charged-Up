// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.peripherals;
import java.util.Map;

import com.frc7153.commands.ConfigCommand;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.OI.Controller0;
import frc.robot.OI.Controller1;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

/**
 * Handles all values and commands put on Shuffleboard
 */
public class ShuffleboardManager {
    // Objects
    private ArmPI armPi;
    private IMU imu;
    private Arm arm;
    private Claw claw;

    // Controller Update Counter
    private GenericEntry controller0Update;
    private GenericEntry controller1Update;

    // Raspberry Pi Diagnostics
    private GenericEntry piVoltageStable;
    private GenericEntry piCPUTemp;
    private GenericEntry piCPUUsage;
    private GenericEntry piMemUsage;
    private GenericEntry piFPS;
    private GenericEntry piAge;
    private GenericEntry piCache;
    private GenericEntry piTarget;

    // Gyro & Accelerometer
    private GenericEntry gyroCalibrated;

    // Arm
    private GenericEntry armAngle;
    private GenericEntry armSP;
    private GenericEntry armVolt;
    private GenericEntry armWinchPos;
    private GenericEntry armLHand;
    private GenericEntry armRHand;
    private GenericEntry armPose;

    // Constructor (Init)
    public ShuffleboardManager(RobotContainer container, ArmPI armPi, IMU imu, Arm arm, Claw claw) {
        // Store objects
        this.armPi = armPi;
        this.imu = imu;
        this.arm = arm;
        this.claw = claw;

        // Drive Tab (Main)
        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

        ShuffleboardLayout brakeLayout = driveTab.getLayout("Full Brakes", BuiltInLayouts.kList)
            .withSize(1, 2);
        
        brakeLayout.add("Enable", new ConfigCommand(() -> {container.toggleBrakes(true);}, "Enable"));
        brakeLayout.add("Disable", new ConfigCommand(() -> {container.toggleBrakes(false);}, "Disable"));

        if (ShuffleboardConstants.kCAMERA_STREAMS) {
            driveTab.addCamera("Front LL", "Front LL", "http://limelight-front.local:5800/")
                .withSize(3, 3)
                .withPosition(1, 0)
                .withProperties(Map.of("SHOW CONTROLS", "OFF"));
            driveTab.addCamera("Back LL", "Back LL", "http://limelight-back.local:5800/")
                .withSize(3, 3)
                .withPosition(4, 0)
                .withProperties(Map.of("SHOW CONTROLS", "OFF"));
        }

        // Controller Tab Init
        ShuffleboardTab controllerTab = Shuffleboard.getTab("Controllers");
        ShuffleboardLayout controllerRecalibrate = controllerTab.getLayout("Controllers", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withProperties(Map.of("LABEL POSITION", "HIDDEN"));

        controllerRecalibrate.add(Controller0.calibrate);
        controllerRecalibrate.add(Controller1.calibrate);

        ShuffleboardLayout controllerUpdates = controllerTab.getLayout("Updates", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0)
            .withProperties(Map.of("LABEL POSITION", "TOP"));
        
        controller0Update = controllerUpdates.add("Controller 0", "No updates...").getEntry();
        controller1Update = controllerUpdates.add("Controller 1", "No updates...").getEntry();

        // RaspberryPi Values
        ShuffleboardTab piTab = Shuffleboard.getTab("Pi");
        WidgetType piGraph = (ShuffleboardConstants.kPI_GRAPHS) ? BuiltInWidgets.kGraph : BuiltInWidgets.kTextView;

        piCPUTemp = piTab.add("CPU Temp (f)", 0.0)
            .withWidget(piGraph)
            .withPosition(0, 0)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", false, "UPPER BOUND", 185, "LOWER BOUND", 0, "UNIT", "f"))
            .getEntry();
        
        piCPUUsage = piTab.add("CPU Usage (%)", 0.0)
            .withWidget(piGraph)
            .withPosition(3, 0)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", false, "UPPER BOUND", 1.0, "LOWER BOUND", 0.0, "UNIT", "%"))
            .getEntry();

        piMemUsage = piTab.add("Memory (%)", 0.0)
            .withWidget(piGraph)
            .withPosition(6, 0)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", false, "UPPER BOUND", 1.0, "LOWER BOUND", 0.0, "UNIT", "%"))
            .getEntry();

        piFPS = piTab.add("FPS", 0.0)
            .withWidget(piGraph)
            .withPosition(0, 3)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", true, "UNIT", "FPS"))
            .getEntry();

        piAge = piTab.add("Latest Packet Age (s)", 0.0)
            .withWidget(piGraph)
            .withPosition(3, 3)
            .withSize(3, 3)
            .withProperties(Map.of("VISIBLE TIME", 25, "AUTOMATIC BOUNDS", true, "UNIT", "s"))
            .getEntry();

        piVoltageStable = piTab.add("Voltage Stable", true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(6, 3)
            .getEntry();
        
        piCache = piTab.add("Latest Cache", "?")
            .withPosition(7, 3)
            .withSize(2, 1)
            .getEntry();

        piTarget = piTab.add("Target Info", "No target")
            .getEntry();
        
        piTab.add("Camera Server", new ConfigCommand(armPi::startCameraServer, "Start CS")).withPosition(6, 4);
        piTab.add("Pause", new ConfigCommand(armPi::pauseProcessing, "Pause")).withPosition(7, 4);
        piTab.add("Resume", new ConfigCommand(armPi::resumeProcessing, "Resume")).withPosition(8, 4);
        piTab.add("Shutdown", new ConfigCommand(armPi::shutdown, "Shutdown")).withPosition(6, 5);
        piTab.add("Reboot", new ConfigCommand(armPi::reboot, "Reboot")).withPosition(7, 5);
        
        // Accelerometer & Gyro
        ShuffleboardTab gyro = Shuffleboard.getTab("Gyro");

        gyroCalibrated = gyro.add("Calibrated", false).getEntry();
        
        gyro.add("Calibrate", imu.imu.new CalibrateIMU()).withPosition(1, 0);

        // Arm
        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

        armSP = armTab.add("Angle Setpoint", 0.0)
            .withPosition(3, 0)
            .getEntry();
        
        armAngle = armTab.add("Angle Actual", 0.0)
            .withPosition(3, 1)
            .getEntry();

        armVolt = armTab.add("Angle Voltage", 0.0)
            .withSize(3, 3)
            .withWidget((ShuffleboardConstants.kARM_GRAPHS) ? BuiltInWidgets.kGraph : BuiltInWidgets.kTextView)
            .getEntry();

        armWinchPos = armTab.add("Arm Winch Rot", 0.0)
            .getEntry();

        armLHand = armTab.add("Left Hand Claw", 0.0)
            .getEntry();
        
        armRHand = armTab.add("Right Hand Claw", 0.0)
            .getEntry();
        
        armPose = armTab.add("Arm Position (inches)", "(?, ?)")
            .getEntry();
    }

    // Update Values
    public void periodic() {
        // Controllers
        controller0Update.setString(String.format("%s seconds", Controller0.getLastOffsetUpdate()));
        controller1Update.setString(String.format("%s seconds", Controller1.getLastOffsetUpdate()));

        // Pi
        piCPUTemp.setDouble(armPi.getTemp());
        piCPUUsage.setDouble(armPi.getCPU());
        piMemUsage.setDouble(armPi.getMemory());
        piFPS.setDouble(armPi.getFPS());
        piAge.setDouble(armPi.getAge());
        piVoltageStable.setBoolean(armPi.getVoltageStable());
        piCache.setString(armPi.getCache());

        if (armPi.hasTarget()) {
            piTarget.setString(String.format("Sees %s at %s, %s", armPi.getIsCone() ? "cone" : "cube", armPi.getXTargetAngle(), armPi.getYTargetAngle()));
        } else {
            piTarget.setString("No target");
        }

        // Gyro
        gyroCalibrated.setBoolean(imu.isCalibrated());

        // Arm
        armSP.setDouble(arm.getAngleSetpoint());
        armAngle.setDouble(arm.getAngleActual());
        armVolt.setDouble(arm.getAngleVoltage());
        armWinchPos.setDouble(arm.getWinchEncPos());

        armLHand.setDouble(claw.getLHandPos());
        armRHand.setDouble(claw.getRHandPos());

        armPose.setString(String.format("%s, %s (in)", arm.getPose().getX(), arm.getPose().getY()));
    }
}
