package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Stores constants the robot needs for math. 
 * Standard units are:
 * <ul>
 * <li>Meters for distance</li>
 * <li>Degrees for angles</li>
 * <li>Pounds for weight</li>
 * </ul>
 */
public class Constants {
    /* SHUFFLEBOARD CONFIG */
    public static boolean kPI_GRAPHS = false;

    /* SWERVE DRIVE CONSTANTS */
    public static Translation2d kWHEEL_DISTANCE = new Translation2d(
        Units.inchesToMeters(20.0)/2.0,
        Units.inchesToMeters(30.5)/2.0
    );

    public static double kFL_SWERVE_OFFSET = 181.67;
    public static double kFR_SWERVE_OFFSET = 178.77;
    public static double kRL_SWERVE_OFFSET = 8.35;
    public static double kRR_SWERVE_OFFSET = 17.139;

    /* ARM MOVEMENT CONSTANTS */
    public static double kJOINT_TO_FLOOR_DIST = Units.inchesToMeters(10.0);
    public static double kJOINT_TO_BUMPER_DIST = Units.inchesToMeters(7.0);

    public static double kARM_ANGLE_RATIO = 12.0; // TODO add chain ratio
    public static double kARM_0_ANGLE = 100.0;

    public static double kARM_MIN_EXTENSION = Units.inchesToMeters(12.0);
    public static double kARM_MAX_EXTENSION = Units.inchesToMeters(50.0);

    public static double kMAX_ARM_ANGLE = 95.0; // symmetrical
    
    public static double kMAX_EXTENSION = Units.feetToMeters(4.0);
    public static double kMAX_HEIGHT = Units.feetToMeters(6.0);
}
