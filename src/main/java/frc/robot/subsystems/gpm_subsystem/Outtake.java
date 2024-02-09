package frc.robot.subsystems.gpm_subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.util.MotorFactory;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.OuttakeConstants;
import frc.robot.constants.Constants;

public class Outtake {

    public enum OuttakeMode {
        AIM_SPEAKER, AIM_AMP, AIM_TRAP, SHOOT, OFF
    }

    private OuttakeMode m_mode;

    WPI_TalonFX m_motor;
    private final PIDController m_pid;

    public void setSetpoint(double setpoint) {
        // set the PID integration error to zero.
        m_pid.reset();
        // set the PID desired position
        m_pid.setSetpoint(setpoint);
    }

    public Outtake(){
        m_motor = MotorFactory.createTalonFXSupplyLimit(OuttakeConstants.MOTOR_ID, Constants.RIO_CAN, OuttakeConstants.CONTINUOUS_CURRENT_LIMIT, OuttakeConstants.PEAK_CURRENT_LIMIT, 
        OuttakeConstants.PEAK_CURRENT_DURATION, OuttakeConstants.PEAK_CURRENT_LIMIT, OuttakeConstants.PEAK_CURRENT_DURATION);
        m_motor.setNeutralMode(OuttakeConstants.NEUTRAL_MODE);
        m_motor.enableVoltageCompensation(true);





    }
