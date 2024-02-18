package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

public final class IntakeConstants {

    // motor port
    public static final int MOTOR_ID = 13;
    public static final int SENSOR_ID = 4;
    public static final int CENTERING_MOTOR_ID = 14;

    // intake speeds
    public static final double INTAKE_POWER = 0.8;
    public static final double CENTERING_POWER = 0.3;

    // Current limits
    public static final int CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int PEAK_CURRENT_LIMIT = 55;
    public static final double PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double INTAKE_STALL_TIME = 0.2;
    public static final double INTAKE_CURRENT_STOP = 10;

    public static final IdleMode idleMode = IdleMode.kBrake;

}