package frc.robot;

import com.frc7153.math.PIDConstant;

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
    public static final boolean kTEST_DEPLOY = true;

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
        public static final PIDConstant kARM_PID = new PIDConstant(0.09, 0.01, 0.0).withOutputRange(-12.0, 12.0);
        public static final PIDConstant kEXT_PID = new PIDConstant(0.01, 0.000001, 0).withError(0.05).withOutputRange(-8.0, 8.0);

        public static final double kJOINT_TO_FLOOR_DIST = 26.0;
        public static final double kJOINT_TO_BUMPER_DIST = 20.0;

        public static final double kHAND_LENGTH = 12.0; // TODO these (and 3 below)

        public static final double kWINCH_MAX_POSITION = 36.0; // Max extension of the winch, from the extension point to the edge, physically and legally
        public static final double kJOINT_TO_EXT_PT = 28.0;
        public static final double kMAX_ANGLE = 125.0;

        public static final double kANGLE_RATIO = 96.0;
        public static final double kWINCH_RATIO = 20.0;
        
        public static final double kMAX_REACH = 4.0 * 12.0;
        public static final double kMAX_HEIGHT = 6.0 * 12.0;

        public static final double kCLEARANCE = 1.0;

        /**
         * Uses polynomial regression to calculate the number of rotations of the winch motor to achieve specific lengths
         * @param ext Joint to claw edge extension, inches
         * @return Number of rotations
         */
        public static final double extToWinchRots(double ext) {
            return (0.0216 * Math.pow(ext, 2)) + (3.967 * ext) - 127.8875;
        }

        /**
         * Uses polynomial regression to calculate the extension in inches from the winch rotations.
         * @param rots Number of rotations
         * @return Inches, joint to edge
         */
        public static final double winchRotsToTargetExt(double rots) {
            return (-0.0001 * Math.pow(rots, 2)) + (0.19 * rots) + 28.0018;
        }
    }

    /* CLAW CONSTANTS */
    public static final class ClawConstants {
        public static final double kANGLE_RATIO = 25.0 * (36.0 / 16.0); 
        
        public static final PIDConstant kHAND_PID = new PIDConstant(1.5, 0.0, 0.0).withOutputRange(-8.0, 8.0);

        public static final double kL_HAND_OFFSET = 0.9;
        public static final double kR_HAND_OFFSET = 0.2;

        public static final int kCURRENT_LIMIT = 5;
    }

    /* FLIPPER CONSTANTS */
    public static final class FlipperConstants {
        public static final double k0_ANGLE = 0.4;
        public static final double kGEAR_RATIO = 25.0;

        public static final double kMAX_ANGLE = 150.0; // Assumed that min is 0
    }
}
