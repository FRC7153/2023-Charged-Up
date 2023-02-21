package frc.robot.subsystems;

import java.util.BitSet;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * For communicating with the Raspberry Pi on the arm (over CAN bus)
 */
public class ArmPI {
    GenericEntry output = Shuffleboard.getTab("outputPi").add("output", "#").getEntry();

    // CAN objects
    private CAN pi = new CAN(19, 8, 10);
    private CANStreamMessage message = new CANStreamMessage();

    // Threading
    private ScheduledExecutorService CANRefreshExecutor = Executors.newSingleThreadScheduledExecutor();

    // Retrieved Data
    private String cache = "";
    private boolean cache_hasTarget = false;
    private int cache_xAngle = 0;
    private int cache_yAngle = 0;
    private int cache_dist = 0;
    private boolean cache_target = false;
    private boolean cache_ls = false;
    private boolean cache_voltage = true;
    private double cache_temp = 0.0;
    private double cache_cpu = 0.0;
    private double cache_mem = 0.0;
    private int cache_fps = 0;
    private double cache_age = 0.0;

    /*
     * Creates object for communicating with the RaspberryPi on the arm.
     * This will automatically start receiving messages from it in a separate thread
     */
    public ArmPI() {
        CANRefreshExecutor.execute(new UpdateThread());
    }

    // BitSet to Int
    private static int bitsetToInt(BitSet set, int assumedLength) {
        int val = 0;

        for (int x = 0; x < assumedLength; x++) {
            if (set.get(x)) {
                val += Math.pow(2, (assumedLength - 1) - x);
            }
        }

        return val;
    }

    // BitSet to String
    private static String bitsetToString(BitSet set) {
        String str = "";

        for (int x = 0; x < set.length(); x++) {
            str += (set.get(x)) ? "1" : "0";
        }

        return str;
    }

    // Refresh Thread
    private class UpdateThread implements Runnable {
        @Override
        public void run() {
            CANData canData = new CANData();
            System.out.println(canData.data);

            while (true) {
                // Needs to reverse each byte individually (idk why lol)
                if (pi.readPacketNew(0b0000010000, canData)) {
                    BitSet data = BitSet.valueOf(canData.data);

                    for (int x = 0; x < 8; x++) {
                        boolean[] newByte = new boolean[8];

                        for (int b = 0; b < 8; b++) {
                            newByte[b] = data.get((x*8) + (7 - b));
                        }

                        for (int b = 0; b < 8; b++) {
                            data.set((x*8) + b, newByte[b]);
                        }
                    }

                    cache = bitsetToString(data); // TODO big problem below

                    if (bitsetToInt(data.get(0, 16), 16) == 0) {
                        cache_hasTarget = false;
                    } else {
                        cache_xAngle = bitsetToInt(data.get(0, 8), 8);
                        cache_yAngle = bitsetToInt(data.get(8, 16), 8);
                        cache_target = data.get(26);
                        cache_hasTarget = true;
                    }

                    cache_dist = bitsetToInt(data.get(16, 26), 10);
                    cache_ls = data.get(27);
                    cache_voltage = data.get(28);
                    cache_temp = bitsetToInt(data.get(29, 36), 7) * (9.0/5.0) + 32.0;
                    cache_cpu = bitsetToInt(data.get(36, 43), 7) / 100.0;
                    cache_mem = bitsetToInt(data.get(43, 50), 7) / 100.0;
                    cache_fps = bitsetToInt(data.get(50, 56), 6);

                    cache_age = Timer.getFPGATimestamp();
                }

                Timer.delay(0.2);
            }
        }
    }

    // Run Command
    /**
     * Different commands that can be run on the Pi with the {@code RaspberryPiCommand} command class
     */
    public static enum PiCommandType {RUN_CAMERA_SERVER, PAUSE_PROCESSING, RESUME_PROCESSING, REBOOT_PI, SHUTDOWN_PI};

    /**
     * Run Command on Raspberry Pi
     */
    public class RaspberryPiCommand extends CommandBase {
        private int command;
        private String title;

        /**
         * Create new command to execute the specified {@code PiCommandType}
         * @param commandType
         */
        public RaspberryPiCommand(PiCommandType commandType) {
            switch (commandType) {
                case PAUSE_PROCESSING:
                    command = 0b0000100001;
                    title = "Pause processing";
                    break;
                case REBOOT_PI:
                    command = 0b0000100011;
                    title = "Reboot Pi";
                    break;
                case RESUME_PROCESSING:
                    command = 0b0000100010;
                    title = "Resume processing";
                    break;
                case RUN_CAMERA_SERVER:
                    command = 0b0000100101;
                    title = "Run camera server";
                    break;
                case SHUTDOWN_PI:
                    command = 0b0000100100;
                    title = "Shutdown Pi";
                    break;
            }
        }

        // Config
        @Override
        public String getName() { return title; }

        @Override
        public boolean runsWhenDisabled() { return true; }

        @Override
        public boolean isFinished() { return true; }

        // Run
        @Override
        public void execute() { pi.writePacket(null, command); }
    }

    // Read Values (Getters)
    /**
     * @return The string value of the last update
     */
    public String getCache() { return cache; }

    /**
     * @return Whether the Pi sees a target
     */
    public boolean hasTarget() { return cache_hasTarget; }

    /**
     * @return The cached x value of the target (degrees)
     */
    public int getXTargetAngle() { return cache_xAngle; }

    /**
     * @return The cached y value of the target (degrees)
     */
    public int getYTargetAngle() { return cache_yAngle; }

    /**
     * @return The cached distance to the target (mm)
     */
    public int getDistance() { return cache_dist; }
    
    /**
     * @return Whether the the target is a cone (otherwise a cube)
     */
    public boolean getIsCone() { return cache_target; }
    
    /**
     * @return The cached value of the limit switch
     */
    public boolean getLimitSwitch() { return cache_ls; }
    
    /**
     * @return Whether the Pi has stable power input
     */
    public boolean getVoltageStable() { return cache_voltage; }
    
    /**
     * @return The cached CPU temp of the Pi (f)
     */
    public double getTemp() { return cache_temp; }
    
    /**
     * @return The cached CPU usage of the Pi (%)
     */
    public double getCPU() { return cache_cpu; }
    
    /**
     * @return The cached memory usage of the Pi (5)
     */
    public double getMemory() { return cache_mem; }
    
    /**
     * @return The cached FPS being processed
     */
    public int getFPS() { return cache_fps; }
    
    /**
     * @return The age of the values in cache
     */
    public double getAge() { return Timer.getFPGATimestamp() - cache_age; }
}
