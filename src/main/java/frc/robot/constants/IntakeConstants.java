package frc.robot.constants;

public final class IntakeConstants {

    // motor port
    public static final int MOTOR_ID = 13; 
    public static final int SENSOR_ID = 4;

    // intake speeds
    public static final double INTAKE_POWER = 0.8;

    // Current limits
    public static final int CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int PEAK_CURRENT_LIMIT = 55;
    public static final double PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double INTAKE_STALL_TIME = 0.2;
    public static final double INTAKE_CURRENT_STOP = 10;

    private IntakeConstants() {
    }

}