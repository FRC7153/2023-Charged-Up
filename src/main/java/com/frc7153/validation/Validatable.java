package com.frc7153.validation;

import java.util.HashMap;

public interface Validatable {
    /**
     * Checks all devices (motors, sensors) and other failure points, then returns
     * a dictionary with all their statuses. This dictionary will be the same size
     * each time.<br><br>
     * <b>All IDs should be UNIQUE!</b>
     * @return HashMap with all the statuses (thread safe)
     */
    public HashMap<String, Boolean> validate();
}
