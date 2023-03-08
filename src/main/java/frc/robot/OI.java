package frc.robot;

import com.frc7153.inputs.Joystick;
import com.frc7153.inputs.XboxController;

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
    }
}
