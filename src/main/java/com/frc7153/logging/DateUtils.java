package com.frc7153.logging;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class DateUtils {
    // Formats
    private static DateTimeFormatter inlineFormat = DateTimeFormatter.ofPattern("dd-MM-yyyy HH:mm:ss");
    private static DateTimeFormatter filenameFormat = DateTimeFormatter.ofPattern("dd_MM_yy-HH_mm_ss");

    // Get date
    /**
     * @return the date in an inline format (dd-MM-yyyy HH:mm:ss)
     */
    public static String getInlineDate() { return LocalDateTime.now().format(inlineFormat); }
    
    /**
     * @return the date in a filename-safe format (dd_MM_yy-HH_mm_ss)
     */
    public static String getFilenameDate() { return LocalDateTime.now().format(filenameFormat); }
}
