package frc.robot.constants.miscConstants;

/**
 * Container class for operator input constants.
 */
public class OIConstants {

    public static final int DRIVER_JOY = 0;

    public static final int OPERATOR_JOY = 1;
    public static final int TEST_JOY = 2;
    public static final int MANUAL_JOY = 3;
    public static final double DEADBAND = 0.005;

    //TODO: change sensitivity to 1?

    public static final double TRANSLATIONAL_SENSITIVITY = 1;
    public static final double TRANSLATIONAL_EXPO = 2;
    public static final double TRANSLATIONAL_DEADBAND = 0.05;
    public static final double TRANSLATIONAL_SLEWRATE = 20;
    public static final boolean FIELD_RELATIVE = true;
    public static final double ROTATION_SENSITIVITY = 1;

    public static final double ROTATION_EXPO = 4;
    public static final double ROTATION_DEADBAND = 0.01;
    public static final double ROTATION_SLEWRATE = 10;
    public static final double HEADING_SENSITIVITY = 4;

    public static final double HEADING_EXPO = 2;
    public static final double HEADING_DEADBAND = 0.05;
    public static final boolean CONSTANT_HEADING_MAGNITUDE = false;
    public static final boolean INVERT = false;

    private OIConstants() {
    }

}
