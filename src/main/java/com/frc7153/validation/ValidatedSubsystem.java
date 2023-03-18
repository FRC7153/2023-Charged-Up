package com.frc7153.validation;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ValidatedSubsystem extends SubsystemBase {
    /**
     * Validates the subsystem by checking everything. The maps returned should be the same size,<br>
     * with the same keys, each time.
     * @return Map of category name ({@code String}) and success ({@code Boolean})
     */
    public abstract HashMap<String, Boolean> validate();

    // Shuffleboard values
    private HashMap<String, GenericPublisher> shuffleboardEntries = new HashMap<>();
    private ShuffleboardLayout shuffleboardLayout = null;

    /**
     * Assign this validated subsystem to a Shuffleboard Layout. Each subsystem can only be on ONE layout.
     * This will probably be a Grid layout.
     * @param layout Shuffleboard layout
     */
    public void assignShuffleboardValidationLayout(ShuffleboardLayout layout) {
        if (shuffleboardLayout != null) {
            DriverStation.reportWarning("Overwrote shuffleboard layout for validated subsystem!", false);
            shuffleboardEntries = new HashMap<>();
        }

        shuffleboardLayout = layout;
        publishValidations();
    }

    /**
     * Publishes the validations.<br><br>
     * Make sure {@code assignShuffleboardLayout(ShuffleboardLayout layout)} has been called already.
     * @param warn Whether it should warn if the state is false.
     */
    public void publishValidations(boolean warn) {
        if (shuffleboardLayout == null) {
            DriverStation.reportError("publishValidations() called before Shuffleboard layout has been assigned!", false);
            return;
        }

        // Update
        HashMap<String, Boolean> state = validate();

        for (String key : state.keySet()) {
            if (shuffleboardEntries.get(key) == null) {
                // Add if entry does not yet exist
                shuffleboardEntries.put(key, shuffleboardLayout.add(key, false).getEntry());
            }

            // Publish
            shuffleboardEntries.get(key).setBoolean(state.get(key));

            // Warn if not valid
            if (!state.get(key) && warn) {
                DriverStation.reportWarning(String.format("Did not pass validation: %s", key), false);
            }
        }
    }

    /**
     * Publishes the validations, adding warnings if the state is false.<br><br>
     * Make sure {@code assignShuffleboardLayout(ShuffleboardLayout layout)} has been called already.
     */
    public void publishValidations() { publishValidations(true); }
}
