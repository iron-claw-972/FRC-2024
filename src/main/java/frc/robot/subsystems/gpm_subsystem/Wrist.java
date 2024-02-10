package frc.robot.subsystems.gpm_subsystem;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

import java.util.Arrays;

public class Wrist extends SubsystemBase {
    
    private final CANSparkFlex motor = new CANSparkFlex(WristConstants.MOTOR_ID, MotorType.kBrushless);
    private final CANSparkFlex[] slaves = new CANSparkFlex[WristConstants.SLAVE_IDS.length]
    private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    private final PIDController pid = new PIDController(WristConstants.P, WristConstants.I, WristConstants.D);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(WristConstants.S, WristConstants.V);

    public Wrist() {
        Arrays.stream(WristConstants.SLAVE_IDS).forEach((id) ->
                {
                    slaves[id] = new CANSparkFlex(id, MotorType.kBrushless);
                    slaves[id].follow(motor);
                }
        );

        pid.setTolerance(WristConstants.TOLERANCE);
        encoder.setPositionConversionFactor(WristConstants.CONVERSION_FACTOR);
        encoder.setZeroOffset(WristConstants.OFFSET);
        motor.setSecondaryCurrentLimit(0.5);
    }

    @Override
    public void periodic() {
        motor.set(pid.calculate(encoder.getVelocity()) + feedforward.calculate(pid.getSetpoint()));
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
        return encoder.getPosition();
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }
}
