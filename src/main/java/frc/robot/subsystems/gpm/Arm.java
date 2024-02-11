package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {
    
    private final TalonFX motor = new TalonFX(ArmConstants.MOTOR_ID);
    private final TalonFX[] slaves = new TalonFX[ArmConstants.SLAVE_IDS.length];
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);
    private final PIDController pid = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ArmConstants.S, ArmConstants.V);

    private DCMotor motorModel;
    private SingleJointedArmSim simulation;
    private DutyCycleEncoderSim encoderSim;

    public Arm() {
        pid.setTolerance(ArmConstants.TOLERANCE);
        encoder.setDistancePerRotation(ArmConstants.DISTANCE_PER_ROTATION);
        encoder.setPositionOffset(ArmConstants.OFFSET);

        for (int i = 0; i < slaves.length; i++) {
            slaves[i] = new TalonFX(ArmConstants.SLAVE_IDS[i]);
            slaves[i].setControl(new Follower(motor.getDeviceID(), false));
        }

        if (RobotBase.isSimulation()) {
            motorModel = DCMotor.getFalcon500(4);
            simulation = new SingleJointedArmSim(motorModel,
                    ArmConstants.GEARING,
                    ArmConstants.MOMENT_OF_INERTIA,
                    ArmConstants.ARM_LENGTH,
                    ArmConstants.MIN_ANGLE_RADS,
                    ArmConstants.MAX_ANGLE_RADS,
                    true,
                    ArmConstants.START_ANGLE_RADS
            );
            encoderSim = new DutyCycleEncoderSim(encoder);
        }
    }

    @Override
    public void periodic() {
        // TODO: To be honest i forgot what should go in the place where the encoder.get() call is,
        // so I just put encoder.get() there for now - Tyrus
        motor.set(pid.calculate(encoder.get()) + feedforward.calculate(pid.getSetpoint()));
    }

    @Override
    public void simulationPeriodic() {
        simulation.setInputVoltage(motor.getSimState().getMotorVoltage());
        simulation.update(0.02);

        encoderSim.setDistance(simulation.getAngleRads());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(simulation.getCurrentDrawAmps()));
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