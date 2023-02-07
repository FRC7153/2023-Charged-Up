package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.wpilibj.CAN;

/**
 * For communicating with the Raspberry Pi on the arm (over CAN bus)
 */
public class ArmPI {
    private CAN pi = new CAN(19, 8, 10);

    public void ping() {
        pi.writePacket(new byte[] {0}, 0b0000100001);
        
    }
}
