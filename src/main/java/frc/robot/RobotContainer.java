package frc.robot;

import java.util.function.BooleanSupplier;

import com.frc7153.controllers.RevBlinkin.BlinkinSolidColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.OI.Controller0;
import frc.robot.OI.Controller1;
import frc.robot.autos.Autonomous;
import frc.robot.autos.TestCommand;
import frc.robot.commands.TeleopClawCommand;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.commands.PresetArmCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.UnlockClawCommand;
import frc.robot.peripherals.ArmPI;
import frc.robot.peripherals.Limelight;
import frc.robot.peripherals.PDH;
import frc.robot.peripherals.ShuffleboardManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LED;

public class RobotContainer {
    // Peripherals
    private final ArmPI armPi = new ArmPI();
    private final Limelight frontLL = new Limelight("front");
    private final Limelight rearLL = new Limelight("back");
    private final PDH pdh = new PDH();
    //public FileDump limeDump = new FileDump("limeDump");

    // Subsystems
    private final DriveBase driveBase = new DriveBase();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();
    private final LED led = new LED(0);

    // Autonomous
    private Autonomous auto = new Autonomous(driveBase, arm, claw);

    // Shuffleboard + Commands
    private final ShuffleboardManager shuffleboard;
    public final Command unlockClawCommand = new UnlockClawCommand(claw, arm);

    // Constructor
    public RobotContainer() {
        // Create command bindings
        configureBindings();

        // Start Shuffleboard
        shuffleboard = new ShuffleboardManager(this, auto, armPi, arm, claw, driveBase, pdh);
    }

    // Configure Button Bindings (teleop drive are defined in getTeleopCommand())
    private void configureBindings() {
        // Not Disabled supplier
        BooleanSupplier notEnabled = () -> { return !DriverStation.isTeleopEnabled(); };
        
        // Default/Teleop Drive Command
        driveBase.setDefaultCommand(new TeleopDriveCommand(
            driveBase,
            Controller0::getLeftX,
            Controller0::getLeftY,
            Controller0::getRightX
        ).unless(notEnabled));

        // Default/Teleop Arm Command (position setpoint)
        arm.setDefaultCommand(new TeleopArmCommand(
            arm, 
            Controller1::getY, 
            Controller1::getThrottle, 
            60.0
        ).unless(notEnabled));

        // Default/Teleop Claw Command
        claw.setDefaultCommand(new TeleopClawCommand(
            arm, 
            claw, 
            Controller0::getRightTrigger
        ).unless(notEnabled));

        // Force wide release
        //Controller1.trigger.whileTrue(new GrabCommand(claw, GrabPositions.WIDE_RELEASE));

        // Arm Preset Positions
        Controller1.button7.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kFRONT_CONE_HIGH, ArmPositions.kREAR_CONE_HIGH).repeatedly());
        Controller1.button8.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kFRONT_CUBE_HIGH, ArmPositions.kREAR_CUBE_HIGH).repeatedly());
        Controller1.button9.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kFRONT_CONE_MID, ArmPositions.kREAR_CONE_MID).repeatedly());
        Controller1.button10.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kFRONT_CUBE_MID, ArmPositions.kREAR_CUBE_MID).repeatedly());
        Controller1.button11.whileTrue(new PresetArmCommand(arm, ArmPositions.kFRONT_GROUND).repeatedly());
        Controller1.button12.whileTrue(new PresetArmCommand(arm, ArmPositions.kREAR_GROUND).repeatedly());

        Controller1.trigger.whileTrue(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kREAR_LOADING_STATION, ArmPositions.kFRONT_LOADING_STATION).repeatedly()); // Opposite of the above commands

        // Loading Station Recovery
        //Controller1.trigger.onFalse(new PresetArmCommand(arm, driveBase.imu, ArmPositions.kREAR_LOADING_STATION_RECOVER, ArmPositions.kFRONT_LOADING_STATION_RECOVER));
        Controller1.trigger.onFalse(new InstantCommand(arm::tempDisableHeightLimit));

        // Control LED Colors
        Controller1.lowerLeftTopButton.onTrue(new InstantCommand(() -> { led.setHue(18); }, led));
        Controller1.lowerRightTopButton.onTrue(new InstantCommand(() -> { led.setColor(BlinkinSolidColor.VIOLET); }, led));

        // Auto Balance
        //Controller0.aButton.whileTrue(new BalanceCommand(driveBase));
        //Controller0.aButton.whileTrue(new ChassisSpeedTestCommand(driveBase));

        // Stow Position (arm 34 degrees, claw stowed)
        /*Controller1.button2.whileTrue(new ParallelCommandGroup(
            new PresetArmCommand(arm, 34.0, 0.0),
            new GrabCommand(claw, GrabPos.STOW)
        ));*/
    }

    // Toggle Brakes (in drive and claw)
    public void toggleBrakes(boolean brake) {
        driveBase.setCoast(!brake);
        claw.setCoastMode(!brake);
        arm.setBrake(brake);
    }

    // Turn off limelights LED
    public void setLimelightLED(boolean state) {
        frontLL.forceLEDMode(state);
        rearLL.forceLEDMode(state);
    }

    public void refreshLEDAlliance() {
        led.setAllianceColor();
    }

    // Run Shuffleboard (even when disabled)
    public void shuffleboardUpdate() {
        shuffleboard.periodic();
        
        // TODO temporary
        //double[] lfPos = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        //double[] lrPos = NetworkTableInstance.getDefault().getTable("limelight-rear").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

        /*limeDump.log(String.format("F LL -> %s, %s, %s, %s, %s, %s", lfPos[0], lfPos[1], lfPos[2], lfPos[3], lfPos[4], lfPos[5]));
        limeDump.log(String.format("R LL -> %s, %s, %s, %s, %s, %s", lrPos[0], lrPos[1], lrPos[2], lrPos[3], lrPos[4], lrPos[5]));
        limeDump.log(driveBase.getPose().toString());
        limeDump.log(String.format("Gyro: r %s, p %s, y %s", driveBase.imu.getRoll(), driveBase.imu.getPitch(), driveBase.imu.getYaw()));*/
    }

    // Get Auto Command
    public Command getAutonomousCommand() {
        return auto.getSelectedAuto();
    }

    // Get Testing Command
    public Command getTestingCommand() {
        return new TestCommand(arm, claw, shuffleboard, Controller1::getThrottle);
    }

    // Check if arms are locked
    public boolean checkHandsLocked() { return !arm.hasBeenReleased; }
}
