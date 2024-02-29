package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

    /** The REV Duty Cycle encoder DIO channel */
    public static final int ENCODER_ID = 3;

    // TODO: use the real gearing
    // 1728 = 4 * 4 * 4 * 27
    public static final double GEARING = 172.8;

    // TODO: use the real moment of inertia
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

    public static final double PIVOT_HEIGHT = Units.inchesToMeters(16.75);
    public static final double PIVOT_X = Units.inchesToMeters(-10);

    // TODO: temporary reduction from 40 A to 4 A. I snapped both chains.
    public static CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(4);
}