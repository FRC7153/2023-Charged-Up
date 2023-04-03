package frc.robot;

import com.frc7153.math.PIDConstant;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Arm.ArmState;;

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
    // Testing program
    public static final boolean kTEST_DEPLOY = false;

    /* SHUFFLEBOARD CONFIG */
    public static final class ShuffleboardConstants {
        public static final boolean kPI_GRAPHS = false;
        public static final boolean kARM_GRAPHS = false;
        public static final boolean kCAMERA_STREAMS = true;
        public static final boolean kFIELD_PLOT = false;
        public static final boolean kVALIDATE = false;
        public static final boolean kPITCH_GRAPHS = true;
    }

    /* SWERVE DRIVE CONSTANTS */
    public static final class SwerveConstants {
        public static final Translation2d kWHEEL_DISTANCE = new Translation2d(
            Units.inchesToMeters(19.0)/2.0,
            Units.inchesToMeters(29.5)/2.0
        );

        public static final double kFL_OFFSET = 358.594;
        public static final double kFR_OFFSET = 285.381;
        public static final double kRL_OFFSET = 259.980;
        public static final double kRR_OFFSET = 117.773 + 180.0; // 300.938
    }

    /* ARM MOVEMENT CONSTANTS */
    /**
     * Note that all distance measurements are in INCHES!
     */
    public static final class ArmConstants {
        // Arm PID
        public static final PIDConstant kARM_PID = new PIDConstant(0.15, 0.15, 0.0).withOutputRange(-12.0, 12.0).withError(1.2).withIntegratorRange(-100.0, 100.0);
        public static final PIDConstant kEXT_PID = new PIDConstant(0.01, 0.000001, 0).withError(0.05).withOutputRange(-8.0, 8.0);

        public static final double kARM_FF = -0.009;

        public static final double kWINCH_TOLERANCE = 5.0;
        public static final double kANGLE_TOLERANCE = 12.0;

        public static final double kWINCH_HOME_ROT_POS = 6.5;

        public static final double kMAX_ANGLE_VELOCITY = 100.0;
        public static final double kMAX_ANGLE_ACCELERATION = 100.0;

        public static final double kJOINT_TO_FLOOR_DIST = 26.0;
        public static final double kJOINT_TO_BUMPER_DIST = 20.0;

        public static final double kHAND_LENGTH = 14.0;

        public static final double kWINCH_MAX_POSITION = 5.0 * 12.0; // Max extension of the winch, from the extension point to the edge, physically and legally
        public static final double kJOINT_TO_EXT_PT = 28.0;
        public static final double kMAX_ANGLE = 125.0;

        public static final double kANGLE_RATIO = 160.0;
        public static final double kWINCH_RATIO = 20.0;
        
        public static final double kMAX_REACH = 4.0 * 12.0 + 2.0; // + 2.0
        public static final double kMAX_HEIGHT = 6.0 * 12.0 + 6.0;

        public static final double kCLEARANCE = 2.0;

        public static final boolean kUSE_POSITION_NOT_VELOCITY = true;

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

    /* ARM PRE-CONFIGURED POSITIONS */
    public static final class ArmPositions {
        // Ground
        public static final Translation2d kFRONT_GROUND = new Translation2d(35.98, 4.34);
        public static final Translation2d kREAR_GROUND = new Translation2d(-44.840, 12.048);

        // Front Placement
        public static final ArmState kFRONT_CUBE_MID = ArmState.fromRots(84.25, 1.7);
        public static final ArmState kFRONT_CUBE_HIGH = ArmState.fromRots(74.42, 156.9);
        public static final ArmState kFRONT_CONE_MID = ArmState.fromRots(66, 64.0); // 53.4
        //public static final ArmState kFRONT_CONE_HIGH = ArmState.fromRots(67.81, 189.8); // wrong
        public static final ArmState kFRONT_CONE_HIGH = ArmState.fromRots(64.0, 192.0);


        // Rear Placement
        public static final ArmState kREAR_CUBE_MID = ArmState.fromRots(-76.01, 42.9);
        public static final ArmState kREAR_CUBE_HIGH = ArmState.fromRots(-67.9, 157.04);
        public static final ArmState kREAR_CONE_MID = ArmState.fromRots(-64.83, 62.85);
        public static final ArmState kREAR_CONE_HIGH = ArmState.fromRots(-58.5, 199.39);

        // Loading Station
        public static final ArmState kFRONT_LOADING_STATION = ArmState.fromRots(68.45, 34.02);
        public static final ArmState kREAR_LOADING_STATION = ArmState.fromRots(-57.13, 25.14);
    }

    /* CLAW CONSTANTS */
    public static final class ClawConstants {
        public static final double kANGLE_RATIO = 25.0 * (36.0 / 16.0);
        
        public static final PIDConstant kHAND_PID = new PIDConstant(1.5, 0.0, 0.0).withOutputRange(-8.0, 8.0);

        public static final double kL_HAND_OFFSET = 0.9;
        public static final double kR_HAND_OFFSET = 0.2;

        public static final int kCURRENT_LIMIT = 20;
    }

    /* CLAW GRAB POSITIONS */
    public static enum GrabPositions { GRAB(0.338 - 0.05, 0.934 + 0.05), RELEASE(0.431, 0.844), WIDE_RELEASE(0.427, 0.850), STOW(0.89, 0.39);
        public final double lPos, rPos;
        GrabPositions(double l, double r) { lPos = l; rPos = r; }
    }

    /* AUTO CONSTANTS */
    public static final class AutoConstants {
        // Drive Pid
        public static final PIDConstant kDRIVE_PID = new PIDConstant(3.25, 1.28, 0.0).withIntegratorRange(-10.0, 10.0); // 5.0, 2.1, 0.0
        public static final PIDConstant kTHETA_PID = new PIDConstant(4.0, 2.5, 0.0).withIntegratorRange(-10.0, 10.0);

        // Auto-Specific Presets
        public static final ArmState kREAR_CONE_HIGH = ArmState.fromRots(-56.7, 215.39);
        public static final Translation2d kFRONT_CUBE_GROUND = new Translation2d(37.98, 3.0);
        public static final Translation2d kFRONT_LOW_CUBE_GROUND = new Translation2d(37.98, 0.9);

        // Balance
        public static final PIDConstant kBALANCE_PID = new PIDConstant(5.0, 0.0, 0.1);
    }

    /* PDH CONSTANTS */
    public static final class PDHConstants {
        public static final int[] kCHANNELS = {0, 1, 2, 3, 5, 6, 7, 8, 9, 15, 16, 17, 18, 20, 21, 22};
    }
}
