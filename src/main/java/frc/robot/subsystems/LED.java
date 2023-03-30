package frc.robot.subsystems;

import com.frc7153.controllers.RevBlinkin;
import com.frc7153.controllers.RevBlinkin.BlinkinSolidColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    // Controller
    private RevBlinkin controller;

    // Constructor
    public LED(int port) {
        controller = new RevBlinkin(port);

        // Lights should start OFF
        setColor(BlinkinSolidColor.BLACK);
    }
    
    // Set Color
    public void setColor(BlinkinSolidColor color) {
        controller.setConstantColor(color);
    }
}
