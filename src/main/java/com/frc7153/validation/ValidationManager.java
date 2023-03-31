package com.frc7153.validation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Handles all validatable objects, publishing their values to Shuffleboard.
 */
public class ValidationManager {
    // Objects
    private ArrayList<Validatable> toValidate;
    private HashMap<String, GenericPublisher> ntEntries; // For pushing to dashboards
    private ShuffleboardLayout layout; // layout to publish to

    private ScheduledExecutorService threadRunner = Executors.newSingleThreadScheduledExecutor(); // Validate Thread runner

    // Config
    private int pollTimeout;
    private boolean running = false;
    private boolean outputErrors;

    // Constructors
    /**
     * Creates a new ValidationManager. There should only be one per robot project.
     * @param tabName Name of tab to use in Shuffleboard
     * @param pollTimeout Amount of time between polls. Default is 10 (seconds)
     * @param outputErrors Whether errors should go to the driver station's warning (default is true)
     */
    public ValidationManager(String tabName, int pollTimeout, boolean outputErrors) {
        this.pollTimeout = pollTimeout; 
        this.outputErrors = outputErrors;

        layout = Shuffleboard.getTab(tabName).getLayout("Validation", BuiltInLayouts.kGrid)
            .withPosition(0, 0)
            .withSize(7, 4);
    }

    /**
     * Creates a new ValidationManager, with default values. There should only be one per robot project.
     */
    public ValidationManager() { this("Validation", 10, true); }

    // Register
    /**
     * Registers new validatable object(s), if they haven't been added yet.
     * @param obj The objects to validate
     */
    public void register(Validatable... objs) {
        for (Validatable obj : objs) {
            if (!toValidate.contains(obj)) {
                toValidate.add(obj);
            }
        }
    }

    // Start/Stop run thread
    /**
     * Starts the validation thread.<br><br>
     * <b>It will not start until this is called</b>
     */
    public void start() {
        if (running) { stop(); }
        threadRunner.scheduleWithFixedDelay(new ValidateThread(), 0, pollTimeout, TimeUnit.SECONDS);
        running = true;
    }

    /**
     * Stops the validation thread.
     */
    public void stop() {
        threadRunner.shutdown();
        running = false;
    }

    // Thread
    private class ValidateThread implements Runnable {
        // Preallocated space for map
        private HashMap<String, Boolean> checks;

        @Override
        public void run() {
            for (Validatable obj : toValidate) {
                checks = obj.validate();

                for (String id : checks.keySet()) {
                    // Add entry if it does not exist
                    if (!ntEntries.containsKey(id)) {
                        ntEntries.put(id, layout.add(id, false).getEntry());
                    }

                    // Update value
                    ntEntries.get(id).setBoolean(checks.get(id));

                    // Error
                    if (outputErrors && checks.get(id)) {
                        DriverStation.reportWarning(String.format("Check '%s' did not pass validation!", id), false);
                    }
                }
            }
        }
    }
}
