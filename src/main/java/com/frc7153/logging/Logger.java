package com.frc7153.logging;

import java.nio.file.Path;

/**
 * Logging class for logging errors and warnings to a file
 */
public class Logger {
    /**
     * Different levels of log messages
     */
    public static enum Level {INFO, WARNING, ERROR, CRITICAL};

    // Path
    private Path logPath;

    /**
     * Create new logger, with date and time as file name.
     * Default headers are added to the top of this file.
     */
    public Logger() {
        
    }
}
