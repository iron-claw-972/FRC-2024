package frc.robot.subsystems.gpm_subsystem;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {
    
    private final TalonFX motor = new TalonFX(ArmConstants.MOTOR_ID);
    private final TalonFX[] slaves = new TalonFX[ArmConstants.SLAVE_IDS.length];
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);
    private final PIDController pid = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ArmConstants.S, ArmConstants.V);

    public Arm() {
        pid.setTolerance(ArmConstants.TOLERANCE);
        encoder.setDistancePerRotation(ArmConstants.DISTANCE_PER_ROTATION);
        encoder.setPositionOffset(ArmConstants.OFFSET);

        for (int i = 0; i < slaves.length; i++) {
            slaves[i] = new TalonFX(ArmConstants.SLAVE_IDS[i]);
            slaves[i].setControl(new Follower(motor.getDeviceID(), false));
        }
    }

    @Override
    public void periodic() {
        // TODO: To be honest i forgot what should go in the place where the encoder.get() call is,
        // so I just put encoder.get() there for now - Tyrus
        motor.set(pid.calculate(encoder.get()) + feedforward.calculate(pid.getSetpoint()));
    }

    /**
     * Sets the angle of the wrist in radians.
     */
    public void setAngle(double angle) { pid.setSetpoint(angle);
    }

    /**
     * Returns the angle of the wrist in radians.
     */
    public double getAngle() {
        return encoder.getAbsolutePosition();
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }
}