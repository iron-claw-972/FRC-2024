package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class PivotSimConstants {

    public static final int MOTOR_ID = 2;

    public static final double GEAR_RATIO = 1.0;
    public static final DCMotor GEAR_BOX = DCMotor.getVex775Pro(2);
    public static final double MOMENT_OF_INERTIA = 0.01403;
    public static final double ARM_LENGTH = 0.127;

    public static final int CONTINUOUS_CURRENT_LIMIT = 20;
    public static final int PEAK_CURRENT_LIMIT = 50;
    public static final double PEAK_CURRENT_DURATION = 0.5;

    public static final double kP = 1.5;
    public static final double kI = 0;
    public static final double kD = 0.0;

    public static final double MIN_POS = -0.16;
    public static final double MAX_POS = 1.97;

    public static final double STARTING_POS = 0.0;

    public static final double GRAVITY_COMPENSATION = 0.02;

    public static final double TOLERANCE = 0.02;
    public static final double MIN_POW = 0;
    public static final double MAX_POW = .8;

    public static final double kSetpointOffsetRads = Math.PI/2;

    public static final double kEncoderTicksToRadsConversion = 2*Math.PI/2048;

}
