package frc.robot;

import com.frc7153.math.PIDConstant;

import edu.wpi.first.math.controller.PIDController;
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
        public static final boolean kARM_GRAPHS = true;
    }

    /* SWERVE DRIVE CONSTANTS */
    public static final class SwerveConstants {
        public static final Translation2d kWHEEL_DISTANCE = new Translation2d(
            Units.inchesToMeters(19.0)/2.0,
            Units.inchesToMeters(29.5)/2.0
        );

        public static final double kFL_OFFSET = 16.523+180.0;
        public static final double kFR_OFFSET = 9.58+180.0;
        public static final double kRL_OFFSET = 177.803+180.0;
        public static final double kRR_OFFSET = 181.143-180.0;
    }

    /* ARM MOVEMENT CONSTANTS */
    /**
     * Note that all distance measurements are in INCHES!
     */
    public static final class ArmConstants {
        public static final PIDConstant kARM_PID = new PIDConstant(0.0084, 1e-6, 0.0).withOutputRange(-11.0, 11.0);
        public static final PIDConstant kEXT_PID = new PIDConstant(0.01, 0, 0).withError(0.05).withOutputRange(-10.0, 10.0);

        public static final double kJOINT_TO_FLOOR_DIST = 10.0;
        public static final double kJOINT_TO_BUMPER_DIST = 7.0;

        public static final double kANGLE_RATIO = 96.0;
        public static final double kWINCH_RATIO = 20.0;

        public static final double kANGLE_0_ANGLE = 100.0;

        public static final double kMIN_EXTENSION = 12.0;
        public static final double kMAX_EXTENSION = 50.0;

        public static final double kMAX_ARM_ANGLE = 95.0; // symmetrical
        
        public static final double kMAX_REACH = 4.0 * 12.0;
        public static final double kMAX_HEIGHT = 6.0 * 12.0;
    }

    /* CLAW CONSTANTS */
    public static final class ClawConstants {
        public static final PIDConstant kHAND_PID = new PIDConstant(0.5, 0, 0).withError(0.05).withOutputRange(-8.0, 8.0);
        
        public static final double kANGLE_RATIO = 12.0; // TODO not right

        public static final double kLHAND_OFFSET = 0.0;
        public static final double kRHAND_OFFSET = 0.0;

        public static final double kMIN_ANGLE = -180.0;
        public static final double kMAX_ANGLE = 10.0;
    }

    /* FLIPPER CONSTANTS */
    public static final class FlipperConstants {
        public static final double k0_ANGLE = 0.4;
        public static final double kGEAR_RATIO = 25.0;

        public static final double kMAX_ANGLE = 150.0; // Assumed that min is 0
    }
}
