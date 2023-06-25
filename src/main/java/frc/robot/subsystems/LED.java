package frc.robot.subsystems;

import com.frc7153.controllers.RevBlinkin;
import com.frc7153.controllers.RevBlinkin2;
import com.frc7153.controllers.RevBlinkin.BlinkinSolidColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    // Controller
    private RevBlinkin controller;
    //private RevBlinkin2 controller2;

    // Constructor
    public LED(int port) {
        controller = new RevBlinkin(port);

        //controller2 = new RevBlinkin2(0);
        //controller2.setConstantColor(com.frc7153.controllers.RevBlinkin2.BlinkinSolidColor.BLUE);

        // Lights should start OFF
        setColor((DriverStation.getAlliance() == Alliance.Blue) ? BlinkinSolidColor.BLUE : BlinkinSolidColor.RED);
    }
    
    // Set Color
    public void setColor(BlinkinSolidColor color) {
        controller.setConstantColor(color);
    }

    public void setHue(int hue) {
        controller.setHue(hue);
    }

    public void setAllianceColor() {
        setColor((DriverStation.getAlliance() == Alliance.Blue) ? BlinkinSolidColor.BLUE : BlinkinSolidColor.RED);
    }
}
