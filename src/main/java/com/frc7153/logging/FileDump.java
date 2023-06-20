package com.frc7153.logging;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

/**
 * Dumps strings to a specified file so they can be viewed later.
 */
public class FileDump {
    // File Headers
    private static String kHEADER = "-- Log File '%s', Created %s --";

    // Path
    private Path logPath;
    private String name;

    /**
     * Create new file dump.<br><br>
     * Logged files can be viewed at <i>/tmp/{@code name}.txt</i>
     * @param name The name of the file. This should be filepath-safe
     */
    public FileDump(String name) {
        this.name = name;
        logPath = Paths.get(String.format("/tmp/%s.txt", name));

        if (!logPath.toFile().exists()) {
            try {
                Files.write(logPath, String.format(kHEADER, name, DateUtils.getInlineDate()).getBytes(), StandardOpenOption.CREATE);
            } catch (IOException e) {
                DriverStation.reportError(String.format("Could not create new log file for %s: %s", name, e), false);
            }
        }
    }

    /**
     * Adds a message to the file. A timestamp will be added with this, and it is added on a blank line.
     * @param msg
     * @return Success
     */
    public boolean log(Object msg) {
        if (Robot.isSimulation()) {
            //System.out.println(String.format("LOG %s -> %s -> %s", name, DateUtils.getInlineDate(), msg));
            return true;
        }

        try {
            Files.write(logPath, String.format("%s -> %s\n", DateUtils.getInlineDate(), msg).getBytes(), StandardOpenOption.APPEND);
            return true;
        } catch (IOException e) {
            DriverStation.reportError(String.format("Could not append message to %s: %s", name, e), false);
            return false;
        }
    }

    /**
     * Clears the log file.<br><br>
     * Note that if {@code deleteFile == true} then future {@code log(msg)} calls may fail.
     * @param deleteFile Whether the file should be deleted, rather than just cleared
     * @return Success
     */
    public boolean clear(boolean deleteFile) {
        try {
            Files.write(logPath, "".getBytes(), StandardOpenOption.DELETE_ON_CLOSE);
            
            if (!deleteFile) {
                Files.write(logPath, String.format(kHEADER, name, DateUtils.getInlineDate()).getBytes(), StandardOpenOption.CREATE);
            }

            return true;
        } catch (IOException e) {
            DriverStation.reportError(String.format("Could not clear log file for %s: %s", name, e), false);
            return false;
        }
    }

    /**
     * Clears the log file, without deleting it.
     * @return Success
     */
    public boolean clear() { return clear(false); }
}
