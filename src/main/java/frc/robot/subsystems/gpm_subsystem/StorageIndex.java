package frc.robot.subsystems.gpm_subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.MotorFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StorageIndexConstants;
import frc.robot.constants.Constants;

/**
 * The StorageIndex subsystem handles the control of the storage index
 * mechanism.
 */
public class StorageIndex extends SubsystemBase {

  private final CANSparkFlex m_indexmotor;
  DigitalInput m_indexBeamBreak;

  /**
   * Constructs the StorageIndex subsystem, initializing the motor and beam break
   * sensor.
   */
  public StorageIndex() {
    m_indexmotor = new CANSparkFlex(StorageIndexConstants.indexMotorID, MotorType.kBrushless);
    m_indexBeamBreak = new DigitalInput(StorageIndexConstants.indexBeamBreak);
    m_indexmotor.setInverted(false);

    m_indexmotor.setIdleMode(IdleMode.kBrake);
    // Additional setup, possibly related to CAN Frames, could be documented here.
  }

  /**
   * Runs the storage index mechanism at the specified speed.
   *
   * @param speed The speed at which to run the storage index mechanism.
   */
  public void runIndex(double speed) {
    m_indexmotor.set(speed);
  }

  /**
   * Runs the storage index mechanism at the default intake speed.
   */
  public void runIndex() {
    m_indexmotor.set(StorageIndexConstants.intakeSpeed);
  }

  /**
   * Stops the storage index mechanism.
   */
  public void stopIndex() {
    m_indexmotor.set(0);
  }

  /**
   * Ejects notes backward at the specified speed, if a note is present.
   *
   * @param speed The speed at which to eject notes backward.
   */
  public void ejectBack(double speed) {
    if (hasNote()) {
      this.runIndex((-1.0) * speed);
    }
  }

  /**
   * Ejects notes forward at the specified speed, if a note is present.
   *
   * @param speed The speed at which to eject notes forward.
   */
  public void ejectFront(double speed) {
    if (hasNote()) {
      this.runIndex(speed);
    }
  }

  /**
   * Checks if a note is present using the beam break sensor.
   *
   * @return True if a note is present, false otherwise.
   */
  public boolean hasNote() {
    return m_indexBeamBreak.get();
  }

}
