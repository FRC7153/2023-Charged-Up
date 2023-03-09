package frc.robot.subsystems;

import java.util.Map;

import com.frc7153.commands.ConfigCommand;
import com.frc7153.inputs.XboxController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.peripherals.ArmPI;
import frc.robot.peripherals.IMU;

/**
 * Handles all values and commands put on Shuffleboard
 */
public class ShuffleboardManager extends SubsystemBase {
    // Objects
    private XboxController controller0;
    private ArmPI armPi;
    private IMU imu;
    private Arm arm;
    private Claw claw;

    // Controller Update Counter
    private GenericEntry controller0Update;

    // Raspberry Pi Diagnostics
    private GenericEntry piVoltageStable;
    private GenericEntry piCPUTemp;
    private GenericEntry piCPUUsage;
    private GenericEntry piMemUsage;
    private GenericEntry piFPS;
    private GenericEntry piAge;
    private GenericEntry piCache;

    // Gyro & Accelerometer
    private GenericEntry gyroCalibrated;

    // Arm
    private GenericEntry armAngle;
    private GenericEntry armSP;
    private GenericEntry armVolt;
    private GenericEntry armWinchPos;
    private GenericEntry armLHand;
    private GenericEntry armRHand;

    // Constructor (Init)
    public ShuffleboardManager(XboxController controller0, ArmPI armPi, IMU imu, Arm arm, Claw claw) {
        // Store objects
        this.controller0 = controller0;
        this.armPi = armPi;
        this.imu = imu;
        this.arm = arm;
        this.claw = claw;

        // Controller Tab Init
        ShuffleboardTab controllerTab = Shuffleboard.getTab("Controllers");
        ShuffleboardLayout controllerRecalibrate = controllerTab.getLayout("Controllers", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withProperties(Map.of("LABEL POSITION", "HIDDEN"));

        controllerRecalibrate.add(controller0.new CalibrateOffsetCommand());

        ShuffleboardLayout controllerUpdates = controllerTab.getLayout("Updates", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0)
            .withProperties(Map.of("LABEL POSITION", "TOP"));
        
        controller0Update = controllerUpdates.add("Controller 0", "No updates...").getEntry();

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
    }

    // Update Values
    @Override
    public void periodic() {
        // Controllers
        controller0Update.setString(String.format("%s seconds", controller0.getLastOffsetUpdate()));

        // Pi
        piCPUTemp.setDouble(armPi.getTemp());
        piCPUUsage.setDouble(armPi.getCPU());
        piMemUsage.setDouble(armPi.getMemory());
        piFPS.setDouble(armPi.getFPS());
        piAge.setDouble(armPi.getAge());
        piVoltageStable.setBoolean(armPi.getVoltageStable());
        piCache.setString(armPi.getCache());

        // Gyro
        gyroCalibrated.setBoolean(imu.isCalibrated());

        // Arm
        armSP.setDouble(arm.getAngleSetpoint());
        armAngle.setDouble(arm.getAngleActual());
        armVolt.setDouble(arm.getAngleVoltage());
        armWinchPos.setDouble(arm.winchEnc.getPosition());

        armLHand.setDouble(claw.getLHandPos());
        armRHand.setDouble(claw.getRHandPos());
    }
}
