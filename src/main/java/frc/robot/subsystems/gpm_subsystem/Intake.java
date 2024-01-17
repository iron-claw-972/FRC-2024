package frc.robot.subsystems.gpm_subsystem;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.globalConst;
import frc.robot.util.MotorFactory;


public class Intake extends SubsystemBase {

    public enum IntakeMode {
        INTAKE(IntakeConstants.INTAKE_POWER), DISABLED(0)
        ;

        private double power;

        IntakeMode(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    private final WPI_TalonFX motor;

    private boolean hasNote;
    private IntakeMode mode;


    public Intake() {
        motor = MotorFactory.createTalonFX(IntakeConstants.MOTOR_ID, globalConst.RIO_CAN);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                IntakeConstants.ENABLE_CURRENT_LIMIT,
                IntakeConstants.CONTINUOUS_CURRENT_LIMIT,
                IntakeConstants.PEAK_CURRENT_LIMIT,
                IntakeConstants.PEAK_CURRENT_DURATION
        ));
        motor.setNeutralMode(IntakeConstants.NEUTRAL_MODE);
        motor.enableVoltageCompensation(true);

        hasNote = false;
        mode = IntakeMode.DISABLED;
    }

    public void setMode(IntakeMode mode) {
        this.mode = mode;
    }

    @Override
    public void periodic() {
        motor.set(mode.getPower());
    }

    public boolean hasNote() {
        return hasNote;
    }

}
