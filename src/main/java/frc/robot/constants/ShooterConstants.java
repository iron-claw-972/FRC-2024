package frc.robot.constants;

public class ShooterConstants {

    // motor Ids (99 is placeholder for now)
    public static final int BOTTOM_MOTOR_ID = 5;
    public static final int TOP_MOTOR_ID = 6;

    // constants for Pids
    public static final double TOP_P = 0.00005;
    public static final double TOP_I = 0;
    public static final double TOP_D = 0;

    public static final double S = 0;
    public static final double V = 1.0/6000;

    public static final double BOTTOM_P = 0.00005;
    public static final double BOTTOM_I = 0;
    public static final double BOTTOM_D = 0;

    /**
     * In RPM
     */
    public static final double TOLERANCE = 10;

    public static final double massColson = 0.245;
    public static final double radiusColson = 2.0 * 0.0254;
    public static final double moiColson = 0.5 * massColson * radiusColson * radiusColson;
    public static final double moiShaft = moiColson * 4;

}
