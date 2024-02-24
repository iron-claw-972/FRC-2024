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
     * right side motors: KR63(3), KR58(58)
     */
    public static final int[] MOTOR_IDS = new int[] {9, 11, 3, 58};

    // TODO: use real encoder id
    //set to 5 to prevent error described by Johann
    public static final int ENCODER_ID = 5;

    // TODO: use the real gearing
    public static final double GEARING = 172.8;

    // TODO: use the real moi
    // guess the MOI as radius = 0.5 meter and the mass is 10 kg
    public static final double MOMENT_OF_INERTIA = 2.5;
    /** Arm length in meters */
    public static final double ARM_LENGTH = .5;
    /** minimum arm angle in radians */
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-20.0);
    /** maximum arm angle in radians */
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(80.0);
    /** starting angle in radians */
    public static final double START_ANGLE_RADS = Units.degreesToRadians(-20.0);

    public static final double intakeSetpoint = 0;
    public static final double stowedSetpoint = 0;
    public static final double subwooferSetpoint = 0;
    public static final double preClimbSetpoint = 0;
    public static final double climbSetpoint = 0;
    public static final double ampSetpoint = 0;


    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public static final boolean inverted = false;

    public static CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(40);
}