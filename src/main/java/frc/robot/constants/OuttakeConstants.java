package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public final class OuttakeConstants {
    public static final int MOTOR_ID = 0;
    public static final double kP = 1.5;
    public static final double kI = 0;
    public static final double kD = 0.0;

    public static final int CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int PEAK_CURRENT_LIMIT = 55;
    public static final double PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

    public static final TalonFXInvertType MOTOR_INVERT = TalonFXInvertType.CounterClockwise;

}
