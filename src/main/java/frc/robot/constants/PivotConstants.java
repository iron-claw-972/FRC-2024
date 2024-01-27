package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class PivotConstants {

    public static final double kSetpointOffsetRads = 0; 

    public static final int MOTOR_ID = 2;

    public static final int GEAR_RATIO = 1;
    public static final double MOMENT_OF_INERTIA = 0.01403;
    public static final double RADIUS = 0.127;

    public static final int CONTINUOUS_CURRENT_LIMIT = 20;
    public static final int PEAK_CURRENT_LIMIT = 50;
    public static final double PEAK_CURRENT_DURATION = 0.5;

    public static final double P = 1.0;
    public static final double I = 0;
    public static final double D = 0.5;
    public static final double F = 0;

    public static final double GRAVITY_COMPENSATION = 0.02;

    public static final double TOLERANCE = 0.02;
    public static final double MOTOR_POWER_CLAMP = 0.8;

    public static final TalonFXInvertType MOTOR_INVERT = TalonFXInvertType.CounterClockwise;

    public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

    /** RoboRIO digital input port for the wrist absolute encoder */
    public static final int ABS_ENCODER_PORT = 7;
    /** wrist absolute encoder offset (rotations). */
    public static final double ENCODER_OFFSET = 0.687;

    public static final double STOW_POS = (5*Math.PI)/6;
    public static final double INTAKE_POS = 0;

    // min/max, in radians
    public static final double MIN_POS = 0;
    public static final double MAX_POS = (5*Math.PI)/6;


    private PivotConstants() {
    }
}
