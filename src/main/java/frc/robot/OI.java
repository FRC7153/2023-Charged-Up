package frc.robot;

import com.frc7153.inputs.Joystick;
import com.frc7153.inputs.XboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Handles all Operator Input
 */
public class OI {
    /** Primary Xbox Drive Controller */
    public static final class Controller0 {
        private static final XboxController controller = new XboxController(0);

        // Config
        public static final double getLastOffsetUpdate() { return controller.getLastOffsetUpdate(); }
        public static final CommandBase calibrate = controller.new CalibrateOffsetCommand();

        // Getters
        public static final double getLeftX() { return controller.getLeftX(); }
        public static final double getLeftY() { return controller.getLeftY(); }
        public static final double getRightX() { return controller.getRightX(); }
        public static final boolean getRightTrigger() { return controller.getRightTrigger(); }

        // Buttons
        public static final JoystickButton aButton = new JoystickButton(controller, XboxController.Button.kA.value);
        public static final JoystickButton lBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        public static final Trigger lTrigger = new Trigger(() -> controller.getRightTrigger());
    }

    /** Secondary Joystick Arm Controller */
    public static final class Controller1 {
        private static final Joystick controller = new Joystick(1);

        // Config
        public static final double getLastOffsetUpdate() { return controller.getLastOffsetUpdate(); }
        public static final CommandBase calibrate = controller.new CalibrateOffsetCommand();

        // Getters
        public static final double getY() { return controller.getY(); }
        public static final double getThrottle() { return controller.getThrottle(); }

        // Buttons
        public static final JoystickButton trigger = new JoystickButton(controller, Joystick.ButtonType.kTrigger.value);

        public static final JoystickButton button2 = new JoystickButton(controller, 2);
        public static final JoystickButton button5 = new JoystickButton(controller, 5);
        public static final JoystickButton button7 = new JoystickButton(controller, 7);
        public static final JoystickButton button8 = new JoystickButton(controller, 8);
        public static final JoystickButton button9 = new JoystickButton(controller, 9);
        public static final JoystickButton button10 = new JoystickButton(controller, 10);
        public static final JoystickButton button11 = new JoystickButton(controller, 11);
        public static final JoystickButton button12 = new JoystickButton(controller, 12);
    }
}
