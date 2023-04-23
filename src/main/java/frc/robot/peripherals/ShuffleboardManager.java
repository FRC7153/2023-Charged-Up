// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.peripherals;
import java.util.Map;

import com.frc7153.commands.ConfigCommand;
import com.frc7153.validation.ValidationManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.OI.Controller0;
import frc.robot.OI.Controller1;
import frc.robot.autos.Autonomous;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

/**
 * Handles all values and commands put on Shuffleboard
 */
public class ShuffleboardManager {
    // Objects
    private ArmPI armPi;
    private Arm arm;
    private Claw claw;
    private DriveBase drive;

    private ValidationManager validator;

    // Drive Tab
    private GenericEntry driveGyroConnected;
    private GenericEntry driveHandUnlocked;
    private GenericEntry drivePos;
    private GenericEntry driveThrottleDown;
    private Field2d driveField;

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
    private GenericEntry piDistance;

    // Gyro & Accelerometer
    private GenericEntry gyroRoll;
    private GenericEntry gyroPitch;
    private GenericEntry gyroYaw;

    // Arm
    private GenericEntry armAngle;
    private GenericEntry armSP;
    private GenericEntry armVolt;
    private GenericEntry armCurrent;
    private GenericEntry armWinchPos;
    private GenericEntry armExtSP;
    private GenericEntry armLHand;
    private GenericEntry armRHand;
    private GenericEntry armPose;
    private GenericEntry armApplied;
    private GenericEntry armAtSP;
    private GenericEntry armAngleDutyCycle;

    // Temp
    private GenericEntry tempAngle;
    private GenericEntry tempExt;
    private GenericEntry tempHand;

    // Constructor (Init)
    public ShuffleboardManager(RobotContainer container, Autonomous auto, ArmPI armPi, Arm arm, Claw claw, DriveBase drive, PDH pdh) {
        // Store objects
        this.armPi = armPi;
        this.arm = arm;
        this.claw = claw;
        this.drive = drive;

        // Init Validator
        if (ShuffleboardConstants.kVALIDATE) {
            validator = new ValidationManager();

            validator.register(drive, arm, claw, pdh, armPi);
            validator.start();
        }

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

        driveGyroConnected = driveTab.add("Gyro Connected", false)
            .withPosition(4, 3)
            .getEntry();

        driveHandUnlocked = driveTab.add("Hand Unlocked", false)
            .withPosition(5, 3)
            .getEntry();

        driveTab.add("Calibrate Gyro", new ConfigCommand(() -> { drive.imu.calibrate(); }, "Calibrate Command"))
            .withPosition(7, 0);

        drivePos = driveTab.add("Odometry Position", "?")
            .withPosition(0, 3)
            .withSize(4, 1)
            .getEntry();
        
        driveTab.add("Auto", auto.getChooser())
            .withPosition(7, 3);

        driveThrottleDown = driveTab.add("Throttle Down (!)", false)
            .withPosition(7, 1)
            .getEntry();
        
        if (ShuffleboardConstants.kFIELD_PLOT) {
            driveField = new Field2d();
            driveTab.add("Odometry Pose (not alliance dependant)", driveField)
                .withPosition(7, 0)
                .withSize(4, 3);
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
        
        piDistance = piTab.add("Distance (in)", 0.0)
            .withPosition(9,0)
            .getEntry();
        
        piTab.add("Camera Server", new ConfigCommand(armPi::startCameraServer, "Start CS")).withPosition(6, 4);
        piTab.add("Pause", new ConfigCommand(armPi::pauseProcessing, "Pause")).withPosition(7, 4);
        piTab.add("Resume", new ConfigCommand(armPi::resumeProcessing, "Resume")).withPosition(8, 4);
        piTab.add("Shutdown", new ConfigCommand(armPi::shutdown, "Shutdown")).withPosition(6, 5);
        piTab.add("Reboot", new ConfigCommand(armPi::reboot, "Reboot")).withPosition(7, 5);
        
        // Accelerometer & Gyro
        ShuffleboardTab gyro = Shuffleboard.getTab("Gyro");
        
        gyroRoll = gyro.add("Roll", 0.0).withPosition(1, 0).getEntry();
        gyroPitch = gyro.add("Pitch", 0.0).withPosition(2, 0).getEntry();
        gyroYaw = gyro.add("Yaw", 0.0).withPosition(3, 0).getEntry();

        // Arm
        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

        armSP = armTab.add("Angle Setpoint", 0.0)
            .withPosition(3, 0)
            .getEntry();
        
        armAngle = armTab.add("Angle Actual", 0.0)
            .withPosition(3, 1)
            .getEntry();

        armCurrent = armTab.add("Angle Current", 0.0)
            .getEntry();

        armVolt = armTab.add("Angle Voltage", 0.0)
            .withSize(3, 3)
            .withWidget((ShuffleboardConstants.kARM_GRAPHS) ? BuiltInWidgets.kGraph : BuiltInWidgets.kTextView)
            .getEntry();

        armWinchPos = armTab.add("Arm Winch Rot", 0.0)
            .getEntry();
        
        armExtSP = armTab.add("Ext Setpoint", 0.0)
            .getEntry();

        armLHand = armTab.add("Left Hand Claw", 0.0)
            .getEntry();
        
        armRHand = armTab.add("Right Hand Claw", 0.0)
            .getEntry();
        
        armPose = armTab.add("Arm Position (inches)", "(?, ?)")
            .getEntry();

        armApplied = armTab.add("Arm (l, r) Applied Output", "?, ?")
            .getEntry();
        
        armAtSP = armTab.add("Arm at setpoint?", false)
            .getEntry();
        
        armAngleDutyCycle = armTab.add("Angle Duty Cycle", 0.0)
            .getEntry();
        
        // Temperature
        ShuffleboardTab tempTab = Shuffleboard.getTab("Temps");

        tempAngle = tempTab.add("Angle (f)", 0.0)
            .getEntry();

        tempExt = tempTab.add("Extension (f)", 0.0)
            .getEntry();

        tempHand = tempTab.add("L, R Hand (f)", "?, ?")
            .getEntry();

        // Warn
        if (ShuffleboardConstants.kMINIMIZE_TRAFFIC) {
            DriverStation.reportWarning("Shuffleboard traffic minimization is enabled, only the DRIVE tab will be kept up-to-date throughout the match.", false);
        }
    }

    // Update Values
    public void periodic() {
        // Drive
        driveGyroConnected.setBoolean(drive.imu.isCalibrated());
        driveHandUnlocked.setBoolean(arm.hasBeenReleased);
        drivePos.setString(drive.getPose().toString());
        driveThrottleDown.setBoolean(Controller1.getThrottle() <= -0.9);

        if (ShuffleboardConstants.kFIELD_PLOT) {
            Pose2d robotPose = drive.getPose();
            driveField.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
        }

        if (!ShuffleboardConstants.kMINIMIZE_TRAFFIC) {
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
            piDistance.setDouble(Units.metersToInches(armPi.getDistance() / 1000.0)); // convert to inches

            if (armPi.hasTarget()) {
                piTarget.setString(String.format("Sees %s at %s, %s", armPi.getIsCone() ? "cone" : "cube", armPi.getXTargetAngle(), armPi.getYTargetAngle()));
            } else {
                piTarget.setString("No target");
            }

            // Gyro
            gyroRoll.setDouble(drive.imu.getRoll());
            gyroPitch.setDouble(drive.imu.getPitch());
            gyroYaw.setDouble(drive.imu.getYaw());

            // Arm
            armSP.setDouble(arm.getAngleSetpoint());
            armAngle.setDouble(arm.getAngleActual());
            armVolt.setDouble(arm.getAngleVoltage());
            armCurrent.setDouble(arm.getAngleCurrent());
            armWinchPos.setDouble(arm.getWinchEncPos());
            armExtSP.setDouble(arm.getWinchSetpoint());
            armAngleDutyCycle.setDouble(arm.getAngleDutyCycle());

            armLHand.setDouble(claw.getLHandPos());
            armRHand.setDouble(claw.getRHandPos());

            armPose.setString(String.format("%s, %s (in)", arm.getPose().getX(), arm.getPose().getY()));
            armApplied.setString(String.format("%s, %s",
                claw.getLeftOutput(),
                claw.getRightOutput()
            ));

            armAtSP.setBoolean(arm.atSetpoint());

            // Temps
            tempAngle.setDouble(arm.getAngleTemp());
            tempExt.setDouble(arm.getExtTemp());
            tempHand.setString(String.format("%s, %s", claw.getLTemp(), claw.getRTemp()));
        }
    }
}
