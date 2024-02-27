package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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

/**
 * Subsystem that controls the arm.
 * <p>
 * The indexer and shooter are at the end of the arm.
 * <p>
 * WARNING: If the absolute encoder slips, then it must be recalibrated:
 * <ol>
 * <li>Limit the motor current to 4 amperes.
 * <li>Power up the robot but do not enable it; the start value may be out of range.
 * <li>While disabled, move the arm and read the absolute encoder values in rotations to find the appropriate offset value.
 * <li>Set the OFFSET value.
 * <li>Then powercycle and find the MIN and MAX values in radians.
 * </ol>
 */
public class Arm extends SubsystemBase {

    /**
     * The motors are stored in an array.
     * <p>
     * The first entry is the master motor; remaining motors are the slaves (followers).
     * <p>
     * The first set of entries are the left side motors, and the second set are the right side.
     */
    private final TalonFX[] motors = new TalonFX[ArmConstants.MOTOR_IDS.length];
    // DCMotor model is 4 Kraken X60
    protected static final DCMotor motorModel = DCMotor.getKrakenX60(ArmConstants.MOTOR_IDS.length);

    /**
     * Motor current limiting.
     * <p>
     * see https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html
     * <p>
     * TODO: temporary reduction from 40 A to 4 A. I snapped both chains.
    */
    private static CurrentLimitsConfigs currentConfig = 
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(4)
            .withSupplyCurrentLimitEnable(true);

    /**
     * Gearbox ratio
     * <p>
     * The gearing starting from the motor is:
     * <p>
     * 8T : 72T
     * <p>
     * 18T : 84T
     * <p>
     * 16T : 64T
     */
    protected static final double GEARING = (72.0 / 8.0) * (84.0 / 18.0) * (64.0 / 16.0);

    // Encoder in the TalonFX....
    //   check docs. Normal update is 4 Hz.
    StatusSignal<Double> rotorPositionSignal;

    // DutyCycle control
    // TODO: change to voltage control
    private final DutyCycleOut m_request = new DutyCycleOut(0);

    /**
     * REV absolute encoder.
     * <p>
     *  It is mounted on the right hand side. CCW rotation is arm raises.
     */
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);
    /** this instance sets the REV absolute encoder value during simulations */
    private DutyCycleEncoderSim encoderSim;
    /** 
     * REV encoder offset in rotations.
     * <p>
     * WARNING: This value will change if the belt driving the REV encoder slips!
     */
    protected static final double OFFSET = 0.54 + Units.degreesToRotations(ArmConstants.MIN_ANGLE_RADS);
    /** REV encoder scale factor. This is fixed. */
    private static final double DISTANCE_PER_ROTATION = -2 * Math.PI;

    // Motor PID control
    private static final double TOLERANCE = Units.degreesToRadians(1.0);
    // P = 5 worked during simulation but broke chain when shooter was not mounted.
    private static final double P = 0.4;
    private static final double I = 0;
    private static final double D = 0;
    private final PIDController pid = new PIDController(P, I, D);

    // Motor feedforward control
    public static final double S = 0;
    public static final double G = 0;
    public static final double V = 0;
    private final ArmFeedforward feedforward = new ArmFeedforward(S, G, V);

    /** arm simulator */
    private SingleJointedArmSim simulation;

    // Mechanism display.
    // The display is always built, but it only changes during simulation.
    private Mechanism2d wristDisplay = new Mechanism2d(90, 90);
    private MechanismRoot2d pivot = wristDisplay.getRoot("ArmPivot", 45, 45);
    private MechanismLigament2d moving = pivot.append(
        new MechanismLigament2d(
            "Moving",
            30,
            Units.radiansToDegrees(ArmConstants.START_ANGLE_RADS),
            6,
            new Color8Bit(Color.kYellow)));

    public Arm() {
        // set the PID tolerance
        pid.setTolerance(TOLERANCE);

        // set the PID the starting position
        pid.setSetpoint(ArmConstants.START_ANGLE_RADS);

        // make the encoder report arm angle in radians
        encoder.setDistancePerRotation(DISTANCE_PER_ROTATION);
        encoder.setPositionOffset(OFFSET);
   
        // consider each of the motors
        for (int i = 0; i < motors.length; i++) {
            // create the motor
            motors[i] = new TalonFX(ArmConstants.MOTOR_IDS[i]);

            // i==0 is the master; the others are slaves
            if (i > 0) {
                // set slave mode
                // the master is MOTOR_IDS[0]
                // invert master for ids 2 and 3
                motors[i].setControl(new Follower(ArmConstants.MOTOR_IDS[0], (i >= motors.length / 2)));
            }
        }

        // common configuration for each motor
        // configure the master after the slaves have been linked so slaves will copy the same settings.
        // we want brake mode for the climb...
        motors[0].setNeutralMode(NeutralModeValue.Brake);
        motors[0].setInverted(false);
        motors[0].getConfigurator().apply(currentConfig);

        // Phoenix v6 rotor position signal
        rotorPositionSignal = motors[0].getRotorPosition();

        // possibly set up simulations
        if (RobotBase.isSimulation()) {
            simulation = new SingleJointedArmSim(motorModel,
                    GEARING,
                    ArmConstants.MOMENT_OF_INERTIA,
                    ArmConstants.ARM_LENGTH,
                    ArmConstants.MIN_ANGLE_RADS,
                    ArmConstants.MAX_ANGLE_RADS,
                    true,
                    ArmConstants.START_ANGLE_RADS);

            // encodersim allows us to set the encoder values
            encoderSim = new DutyCycleEncoderSim(encoder);

            // put the display on the SmartDashboard
            SmartDashboard.putData("ArmSim", wristDisplay);
            SmartDashboard.putData(pid);
        }

        // TODO: remove when not needed.
        // Add some test commands
        SmartDashboard.putData("Set Angle to 0.0", new InstantCommand(() -> setAngle(0.0)));
        SmartDashboard.putData("Set Angle to 1.0 Rad", new InstantCommand(() -> setAngle(0.6)));
        SmartDashboard.putData("arm pid", pid);
    }

    /**
     * Get the arm angle using the absolute sensor or the motor encoder.
     * @return
     */
    double getRadians() {
        // using the REV absolute sensor is easy
        return encoder.getDistance();

        // using the motor encoder is more difficult
        // refresh the position (needed because we want it more often than 4 Hz)
        // rotorPositionSignal.refresh();
        // return Units.rotationsToRadians(rotorPositionSignal.getValue() / ArmConstants.GEARING);
    }

    @Override
    public void periodic() {
        double setpoint = pid.getSetpoint();

        // clamp the setpoint to avoid trouble ...
        if (setpoint < ArmConstants.MIN_ANGLE_RADS) {
            setpoint = ArmConstants.MIN_ANGLE_RADS;
            pid.setSetpoint(ArmConstants.MIN_ANGLE_RADS);
        } else if (setpoint > ArmConstants.MAX_ANGLE_RADS) {
            setpoint = ArmConstants.MAX_ANGLE_RADS;
            pid.setSetpoint(ArmConstants.MAX_ANGLE_RADS);
        }
        // setpoint is now in the safe range...

        // TODO: turn off feedforward when we are close to the stow position

        // calculate the desired duty cycle
        double dutyCycle = MathUtil.clamp(
                        pid.calculate(getRadians()) + feedforward.calculate(setpoint, 0),
                        -1,
                        1);

        // use the Phoenix 6 version of setting the motor "power"
        motors[0].setControl(m_request.withOutput(dutyCycle));

        // report the arm angle in radians
        SmartDashboard.putNumber("Arm angle", encoder.getDistance());

        // TODO: Clean these up when not needed.
        // report dutycycle
        SmartDashboard.putNumber("arm pow", dutyCycle);
        // the absolute position (rotations) is the one used to set the offset.
        SmartDashboard.putNumber("REV ABS", encoder.getAbsolutePosition());
        // report the motor's rotor position.
        SmartDashboard.putNumber("Rotor Signal", Units.rotationsToRadians(rotorPositionSignal.getValue()));
        // check the latency (will be slow unless .refresh() is executed)
        SmartDashboard.putNumber("Rotor delay", rotorPositionSignal.getTimestamp().getLatency());
    }

    @Override
    public void simulationPeriodic() {
        // Assuming the volts
        double voltsBattery = 12.8;

        // set the simulator inputs
        simulation.setInputVoltage(motors[0].get() * voltsBattery);

        // run the simulator for 20 milliseconds
        simulation.update(0.02);

        // set the arm angle in the display
        moving.setAngle(Units.radiansToDegrees(simulation.getAngleRads()));

        // update the DutyCycleEncoder
        encoderSim.setDistance(simulation.getAngleRads());

        // simulation angle seems to be off.
        SmartDashboard.putNumber("sim amp", simulation.getCurrentDrawAmps()/4);
    }

    /**
     * Sets the angle of the wrist in radians.
     * @param angle (in radians)
     */
    public void setAngle(double angle) {
        // zero the integrator portion of the PID controller
        pid.reset();

        // clamp the angle to avoid problems
        angle = MathUtil.clamp(angle, ArmConstants.MIN_ANGLE_RADS, ArmConstants.MAX_ANGLE_RADS);

        // change the setpoint
        pid.setSetpoint(angle);
    }

    /**
     * Returns the angle of the wrist in radians.
     * @returns arm angle in radians
     */
    public double getAngleRad() {
        return encoder.getDistance();
    }

    /**
     * Whether the arm has reached its setpoint
     * @return true when the setpoint is reached.
     */
    public boolean atSetpoint() {
        return pid.atSetpoint();
    }
}
