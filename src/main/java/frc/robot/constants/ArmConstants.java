package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ArmConstants {

    public static final int MOTOR_ID = 0;
    public static final int[] SLAVE_IDS = new int[] {
            0, 0, 0
    };
    // TODO use real encoder id
    //set to 5 to prevent error described by Johann
    public static final int ENCODER_ID = 5;

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
    public static final double ARM_LENGTH = .5;
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-20.0);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(120.0);
    public static final double intakeSetpoint = 0;
    public static final double stowedSetpoint = 0;
    public static final double subwooferSetpoint = 0;
    public static final double preClimbSetpoint = 0;
    public static final double climbSetpoint = 0;
    public static final double ampSetpoint = 0;

    public static final double START_ANGLE_RADS = 0.0;

    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

    // these might be useless because MIN_ANGLE_RADS etc.
    public static final double Duty_Cycle_Max = 0;
    public static final double Duty_Cycle_Min = 0;

    public static final boolean inverted = false;

    public static CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.SupplyCurrentLimit = 40;


}