package frc.robot.constants.miscConstants;

/**
 * Container class for falcon constants.
 */
public class FalconConstants {

    public static final int FIRMWARE_VERSION = 5633; // version 22.1.1.0
    public static final boolean BREAK_ON_WRONG_FIRMWARE = false; // TODO: fix issue that make the robot break

    public static final double RESOLUTION = 2048;
    public static final double MAX_RPM = 6380.0; // Rotations per minute


    /*
     * Talon Stator / Supply Limits explanation
     * Supply current is current that’s being drawn at the input bus voltage. Stator
     * current is current that’s being drawn by the motor.
     * Supply limiting (supported by Talon FX and SRX) is useful for preventing
     * breakers from tripping in the PDP.
     * Stator limiting (supported by Talon FX) is useful for limiting
     * acceleration/heat.
     */

    // These are the default values

    // Stator
    public static final boolean STATOR_LIMIT_ENABLE = false; // enabled?
    public static final double STATOR_CURRENT_LIMIT = 100; // Limit(amp)
    public static final double STATOR_TRIGGER_THRESHOLD = 100; // Trigger Threshold(amp)
    public static final double STATOR_TRIGGER_DURATION = 0; // Trigger Threshold Time(s)

    // Supply
    public static final boolean SUPPLY_LIMIT_ENABLE = false; // enabled?
    public static final double SUPPLY_CURRENT_LIMIT = 40; // Limit(amp), current to hold after trigger hit
    public static final double SUPPLY_TRIGGER_THRESHOLD = 55; // (amp), amps to activate trigger
    public static final double SUPPLY_TRIGGER_DURATION = 3; // (s), how long after trigger before reducing

    private FalconConstants() {
    }

}