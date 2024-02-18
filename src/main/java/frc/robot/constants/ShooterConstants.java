package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

public class ShooterConstants {

    // motor Ids
    public static final int RIGHT_MOTOR_ID = 1;
    public static final int LEFT_MOTOR_ID = 2;

    public static final IdleMode idleMode = IdleMode.kBrake;

    // RPMs

    public static final double LEFT_SHOOT_RPM = 6000;
    public static final double RIGHT_SHOOT_RPM = 4000;
    public static final double INTAKE_RPM = 0; // may not be neccessary if indexer spins fast enough
    public static final double EJECT_RMP = 0; // may not be neccessary if indexer spins fast enough

}
