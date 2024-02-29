package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ArmConstants {

    /** 
     * Arm motor ids.
     * <p>
     * left side motors: KR69(9), KR71(11);
     * <p>
     * right side motors: KR63(8), KR58(58)
     */
    public static final int[] MOTOR_IDS = new int[] {9, 11, 8, 58};

    public static final double S = 0;
    public static final double V = 0;
    public static final double P = 3400;
    public static final double I = 0;
    public static final double D = 500;

    // All angle measurements in radians
    public static final double OFFSET = 0;
    public static final double TOLERANCE = Units.degreesToRadians(1.0);
    public static final double DISTANCE_PER_ROTATION = 2 * Math.PI;

    // TODO use the real gearing
    public static final double GEARING = 172.8;
    // TODO use the real moi
    // guess the MOI as radius = 0.5 meter and the mass is 10 kg
    public static final double MOMENT_OF_INERTIA = 2.5;
    /** Arm length in meters */
    public static final double ARM_LENGTH = .5;
    /** minimum arm angle in radians -- temporarily zero */
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-10.0);
    /** maximum arm angle in radians */
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(78);
    /** starting angle in radians */
    public static final double START_ANGLE_RADS = Units.degreesToRadians(-10.0);

    public static final double intakeSetpoint = MIN_ANGLE_RADS;
    public static final double stowedSetpoint = MIN_ANGLE_RADS;
    public static final double standbySetpoint = MIN_ANGLE_RADS;
    public static final double subwooferSetpoint = MIN_ANGLE_RADS;
    public static final double preClimbSetpoint = MAX_ANGLE_RADS;
    public static final double climbSetpoint = MIN_ANGLE_RADS;
    public static final double ampSetpoint = MAX_ANGLE_RADS;

    public static final double START_ANGLE_RADS = 0.0;

    // TODO: temporary reduction from 40 A to 4 A. I snapped both chains.
    public static CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(30);
}