package frc.robot.constants;

/**
 * Container class for general constants.
 */
public final class Constants {
    public static final double GRAVITY_ACCELERATION = 9.8;
    public static final double ROBOT_VOLTAGE = 12.0;
    public static final double LOOP_TIME = 0.02;

    public static final double CANCODER_RESOLUTION = 4096;

    // CAN bus names
    public static final String CANIVORE_CAN = "CANivore";
    public static final String RIO_CAN = "rio";

    /**
     * The key used to access the RobotId name in the RoboRIO's persistent memory.
     */
    public static final String ROBOT_ID_KEY = "RobotId";

    public static final boolean DO_LOGGING = true;
    public static final boolean USE_TELEMETRY = false;

    // port for the LED controller, the Blinkin
    public static final int BLINKIN_PORT = 0;
}
