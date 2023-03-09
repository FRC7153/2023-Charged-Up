package frc.robot;

import com.frc7153.inputs.XboxController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Handles all Operator Input
 */
public class OI {
    /** Primary Xbox Drive Controller */
    public static final class Controller0 {
        public static final XboxController controller = new XboxController(0);

        // Getters
        public static final double getLeftX() { return controller.getLeftX(); }
        public static final double getLeftY() { return controller.getLeftY(); }
        public static final double getRightX() { return controller.getRightX(); }

        // Buttons
        public static final JoystickButton lBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        public static final Trigger lTrigger = new Trigger(() -> controller.getRightTrigger());
    }

    /** Secondary Joystick Arm Controller */
    public static final class Controller1 {
        public static final Joystick controller = new Joystick(1);

        // Getters
        public static final double getY() { return controller.getY(); }
        public static final double getThrottle() { return controller.getThrottle(); }

        // Buttons
        public static final Trigger trigger = new Trigger(() -> controller.getTrigger());

        public static final JoystickButton button7 = new JoystickButton(controller, 7);
        public static final JoystickButton button8 = new JoystickButton(controller, 8);
        public static final JoystickButton button9 = new JoystickButton(controller, 9);
        public static final JoystickButton button10 = new JoystickButton(controller, 10);
        public static final JoystickButton button11 = new JoystickButton(controller, 11);
        public static final JoystickButton button12 = new JoystickButton(controller, 12);
    }
}
