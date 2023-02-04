package com.frc7153.Controllers;

/**
 * How controllers should handle offsets to analog inputs (ie joysticks)
 * <ul>
 *  <li><b>TRANSLATE: </b>moves the 0, 0 position without scaling it. (e.g. if the offset is 0.2 on the y axis, 
 *  moving the joystick fully up will return a value of 0.8). In most configurations, this could make it impossible
 *  to reach a position of 1.0 or -1.0</li>
 *  <li><b>INTERPOLATE: </b>moves the 0, 0 position and scales the distance. (e.g. if the offset is 0.2 on the y
 *  axis, moving the joystick fully up will still return a value of 1.0). In most configurations, this will make one
 *  side more slightly more sensitive than the other </li>
 * </ul>
 * Default mode is INTERPOLATE to allow robot to reach maximum speed.
 */
public enum OffsetMode {
    TRANSlATE,
    INTERPOLATE
}
