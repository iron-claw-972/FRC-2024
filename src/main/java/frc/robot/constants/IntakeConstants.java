package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public final class IntakeConstants {

    // motor port
    public static final int MOTOR_ID = 9;

    // intake speeds
    public static final double INTAKE_POWER = 0.8;
    public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

    // Current limits
    public static final int CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int PEAK_CURRENT_LIMIT = 55;
    public static final double PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    private IntakeConstants() {
    }

}