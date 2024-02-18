package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {

    /** the master motor */
    private final TalonFX motor = new TalonFX(ArmConstants.MOTOR_ID);
    /** the slave motors */
    private final TalonFX[] slaves = new TalonFX[ArmConstants.SLAVE_IDS.length];
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);
    private final PIDController pid = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ArmConstants.S, ArmConstants.V);

    private SingleJointedArmSim simulation;
    private DutyCycleEncoderSim encoderSim;

    private Mechanism2d wristDisplay = new Mechanism2d(90, 90);
    private MechanismRoot2d pivot = wristDisplay.getRoot("ArmPivot", 45, 45);
    private MechanismLigament2d moving;

    public Arm() {
        motor.setNeutralMode(ArmConstants.neutralMode);
        motor.setInverted(ArmConstants.inverted);

        // setting the PID tolerance
        pid.setTolerance(ArmConstants.TOLERANCE);

        // encoder reports arm angle in radians
        encoder.setDistancePerRotation(ArmConstants.DISTANCE_PER_ROTATION);
        encoder.setPositionOffset(ArmConstants.OFFSET);
        encoder.setDutyCycleRange(ArmConstants.Duty_Cycle_Min, ArmConstants.Duty_Cycle_Max);

        for (int i = 0; i < slaves.length; i++) {
            slaves[i] = new TalonFX(ArmConstants.SLAVE_IDS[i]);
            slaves[i].setControl(new Follower(motor.getDeviceID(), false));
            slaves[i].setNeutralMode(ArmConstants.neutralMode);
            slaves[i].setInverted(ArmConstants.inverted);
        }

        if (RobotBase.isSimulation()) {
            // DCMotor model is 4 Falcon 500s
            DCMotor motorModel = DCMotor.getFalcon500(4);

            simulation = new SingleJointedArmSim(motorModel,
                    ArmConstants.GEARING,
                    ArmConstants.MOMENT_OF_INERTIA,
                    ArmConstants.ARM_LENGTH,
                    ArmConstants.MIN_ANGLE_RADS,
                    ArmConstants.MAX_ANGLE_RADS,
                    true,
                    ArmConstants.START_ANGLE_RADS);
            moving = pivot.append(
            new MechanismLigament2d(
                    "Moving",
                    30,
                    Units.radiansToDegrees(/* m_wristPhysicsSim */ simulation.getAngleRads()),
                    6,
                    new Color8Bit(Color.kYellow)));
            // encodersim allows us to set the encoder values
            encoderSim = new DutyCycleEncoderSim(encoder);

            SmartDashboard.putData("ArmSim", wristDisplay);
            SmartDashboard.putData(pid);

        }
        SmartDashboard.putData("Set Angle to 0.0", new InstantCommand(() -> setAngle(0.0)));
        SmartDashboard.putData("Set Angle to 0.001 Rad", new InstantCommand(() -> setAngle(0.001)));
        SmartDashboard.putData("Set Angle to 1.0 Rad", new InstantCommand(() -> setAngle(1.0)));
    }

    @Override
    public void periodic() {
        // use the scaled distance (which is radians)
        motor.set(
                MathUtil.clamp(
                        pid.calculate(encoder.getDistance()) + feedforward.calculate(pid.getSetpoint()),
                        -1,
                        1
                )
        );
    }

    @Override
    public void simulationPeriodic() {
        // Assuming the volts
        double voltsBattery = 12.8;

        simulation.setInputVoltage(motor.get() * voltsBattery);

        simulation.update(0.02);

        moving.setAngle(Units.radiansToDegrees(simulation.getAngleRads()));
        // update the DutyCycleEncoder
        encoderSim.setDistance(simulation.getAngleRads());
    }

    /**
     * Sets the angle of the wrist in radians.
     */
    public void setAngle(double angle) {
        pid.reset();
        pid.setSetpoint(angle * ArmConstants.GEARING);
    }

    /**
     * Returns the angle of the wrist in radians.
     */
    public double getAngleRad() {
        return encoder.getDistance() / ArmConstants.GEARING;
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }
}