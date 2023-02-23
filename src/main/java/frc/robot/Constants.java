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
public final class Constants {
    /* SHUFFLEBOARD CONFIG */
    public static final class ShuffleboardConstants {
        public static final boolean kPI_GRAPHS = false;
    }

    /* SWERVE DRIVE CONSTANTS */
    public static final class SwerveConstants {
        public static final Translation2d kWHEEL_DISTANCE = new Translation2d(
            Units.inchesToMeters(20.0)/2.0,
            Units.inchesToMeters(30.5)/2.0
        );

        public static final double kFL_OFFSET = 181.67;
        public static final double kFR_OFFSET = 178.77;
        public static final double kRL_OFFSET = 8.35;
        public static final double kRR_OFFSET = 17.139;
    }

    /* ARM MOVEMENT CONSTANTS */
    /**
     * Note that all distance measurements are in INCHES!
     */
    public static final class ArmConstants {
        public static final double kJOINT_TO_FLOOR_DIST = 10.0;
        public static final double kJOINT_TO_BUMPER_DIST = 7.0;

        public static final double kANGLE_RATIO = 12.0; // TODO add chain ratio
        public static final double kANGLE_0_ANGLE = 100.0;

        public static final double kMIN_EXTENSION = 12.0;
        public static final double kMAX_EXTENSION = 50.0;

        public static final double kMAX_ARM_ANGLE = 95.0; // symmetrical
        
        public static final double kMAX_REACH = 4.0 * 12.0;
        public static final double kMAX_HEIGHT = 6.0 * 12.0;
    }
}
