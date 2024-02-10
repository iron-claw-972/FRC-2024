package frc.robot.subsystems.gpm_subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.MotorFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StorageIndexConstants;
import frc.robot.constants.Constants;

public class StorageIndex extends SubsystemBase {

    private final CANSparkMax m_indexmotor;
    // private final SparkAbsoluteEncoder encoder;

    DigitalInput m_indexBeamBreak;

    public StorageIndex() {
        m_indexmotor = MotorFactory.createSparkMAX(StorageIndexConstants.indexMotorID, MotorType.kBrushless,
                StorageIndexConstants.stallLimit);
        // encoder = m_indexmotor.getAbsoluteEncoder(Type.kDutyCycle);

        m_indexBeamBreak = new DigitalInput(StorageIndexConstants.indexBeamBreak);

        m_indexmotor.setInverted(false);

        // they did something to do with CAN Frames here

    }

    public void runIndex(double speed) {
        m_indexmotor.set(speed);
    }

    public void runIndex() {
        m_indexmotor.set(StorageIndexConstants.intakeSpeed);
    }

    public void stopIndex() {
        m_indexmotor.set(0);
    }

    public void ejectBack(double speed) {
        if (hasNote()) {
            this.runIndex((-1.0) * speed);
        }
    }

    public void ejectFront(double speed) {
        if (hasNote()) {
            this.runIndex(speed);
        }
    }

    public boolean hasNote() {
        return m_indexBeamBreak.get() ? false : true;
    }

}
