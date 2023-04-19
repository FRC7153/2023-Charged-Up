package frc.robot.peripherals;

import java.util.HashMap;

import com.frc7153.validation.DeviceChecker;
import com.frc7153.validation.Validatable;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.PDHConstants;

public class PDH implements Validatable {
    // PDH
    private PowerDistribution hub = new PowerDistribution(2, ModuleType.kRev);

    // Validate
    private HashMap<String, Boolean> validationMap = new HashMap<>(2);

    @Override
    public HashMap<String, Boolean> validate() {
        validationMap.put("PDH CAN + Hardware", DeviceChecker.validatePowerDistribution(hub));
        validationMap.put("PDH Channels", DeviceChecker.validatePowerDistributionChannels(hub, PDHConstants.kCHANNELS));

        return validationMap;
    }
}